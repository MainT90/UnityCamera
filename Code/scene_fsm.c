#include <xy_inc_x.h>

#include "scene_fsm.h"
#include "scene_fsm_impl.h"

#include <string.h>
#include "xy_def.h"
#include "xy_errcode.h"
#include "xy_global_cfg.h"
#include "xy_shm_mempool_new.h"
#include "xy_assert.h"
#include "xy_errcode.h"
#include "xy_tbus_cfg.h"
#include "scene_svr_cfg.h"

#include "xy_tbus_def.h"
#include "xy_tbus_op.h"
#include "scene_send.h"
#include "scene_psp.h"


struct scene_fsm_data* g_scene_fsm_data = NULL;

inline struct scene_fsm_data* get_scene_fsm_data()
{
    return g_scene_fsm_data;
}

inline void set_scene_fsm_data(struct scene_fsm_data* fsm_data)
{
    xy_assert_retnone(fsm_data != NULL);
    g_scene_fsm_data = fsm_data;
}

// scene fsm state

static struct scene_fsm_entry* scene_fsm_find(int state)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    int i;

    if (fsm_data == NULL)
        return NULL;

    for (i = 0; i < SCENE_FSM_MAXSTATE; ++ i)
    {
        struct scene_fsm_entry* entry = &fsm_data->state_array[i];
        if (entry->state == state)
            return entry;
    }

    return NULL;
}

int scene_fsm_trans()
{
    struct scene_fsm_entry* curr_entry = NULL;
    struct scene_fsm_entry* new_entry = NULL;
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();

    if (fsm_data == NULL)
        return XYERR_PSP_INVALIDSTATE;

    if (fsm_data->current_state == fsm_data->new_state)
        return XYERR_PSP_SAMESTATE;

    curr_entry = scene_fsm_find(fsm_data->current_state);
    new_entry = scene_fsm_find(fsm_data->new_state);
    if (new_entry == NULL)
        return XYERR_PSP_INVALIDSTATE;

    if ((curr_entry == NULL) && (fsm_data->current_state != SCENE_FSM_NOSTATE))
        return XYERR_PSP_INVALIDSTATE;

    // do trans...

    if (curr_entry != NULL)
        if (curr_entry->on_leave != NULL)
            curr_entry->on_leave(curr_entry);

    fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_1;

    fsm_data->current_state = fsm_data->new_state;

    fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_2;

    if (new_entry->on_enter != NULL)
        new_entry->on_enter(new_entry);

    fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_0;

    fsm_data->current_entry = new_entry;

    return XYERR_PSP_OK;
}

static int scene_fsm_set_start(int state)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();

    if ((state < 0) || (state >= SCENE_FSM_MAXSTATE))
        return XYERR_PSP_INVALIDSTATE;

    if (fsm_data == NULL)
        return XYERR_PSP_INVALIDPARAM;

    // simulate a trans from NOSTATE to the new state
    fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_0;
    fsm_data->current_entry = NULL;
    fsm_data->current_state = SCENE_FSM_NOSTATE;
    fsm_data->new_state = state;

    return scene_fsm_trans();
}

int scene_fsm_action(int id_from, struct scene_fsm_xypkg* xypkg)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();

    if (fsm_data == NULL)
        return XYERR_PSP_INVALIDPARAM;

    struct scene_fsm_entry* entry = fsm_data->current_entry; //scene_fsm_find(fsm_data->current_state);
    xy_assert_retval(entry != NULL, XYERR_XY_ASSERT_DEFAULT);

    // do action...
    if (entry->on_handle != NULL)
    {
        entry->on_handle(entry, id_from, xypkg);
        if (fsm_data->new_state != fsm_data->current_state)
            scene_fsm_trans();
    }

    return XYERR_PSP_OK;
}

int scene_fsm_trans_req(int new_state)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    if (fsm_data == NULL)
    {
        error_tlog("scene_fsm_trans_req error, g_scene_fsm_data = NULL");
        return XYERR_PSP_NOTINITIALIZE;
    }

    if (fsm_data->trans_pending != 0)
    {
        error_tlog("scene_fsm_trans_req error, trans was pending");
        return XYERR_PSP_TRANSPENDING;
    }

    if (new_state < 0 || new_state >= SCENE_FSM_MAXSTATE)
        return XYERR_PSP_INVALIDSTATE;

    fsm_data->new_state = new_state;

    return XYERR_PSP_OK;
}

int scene_fsm_current_state()
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    
    xy_assert_retval(fsm_data != NULL, XYERR_XY_ASSERT_DEFAULT);

    return fsm_data->current_state;
}

static int scene_fsm_alloc(XY_MEMPOOL_INIT_MODE mode, struct scene_fsm_data** fsm_data)
{
    u64 scene_fsm_mid = INVALID64;
    struct scene_fsm_data* l_fsm_data = NULL;

    if (fsm_data == NULL)
    {
        error_tlog("scene_fsm_alloc error, fsm_data == NULL");
        return XYERR_PSP_INVALIDPARAM;
    }

    // get the pointer from shm
    if (mode == XY_MEMPOOL_INIT_START)
    {
        scene_fsm_mid = xy_memunit_alloc(MEM_DATATYPE_SCENE_FSM);
        if (INVALID64 == scene_fsm_mid)
        {
            error_tlog("scene_fsm_alloc error, xy_memunit_alloc return INVALID64");
            return XYERR_PSP_ALLOCFAIL;
        }
    }
    else if (mode == XY_MEMPOOL_INIT_RESUME)
    {
        size_t position = 0;
        if (0 != xy_memunit_get_first(MEM_DATATYPE_SCENE_FSM, &position))
            xy_assert_retval(0, XYERR_XY_ASSERT_DEFAULT);

        scene_fsm_mid = xy_memunit_get_next(MEM_DATATYPE_SCENE_FSM, &position);
        if (INVALID64 == scene_fsm_mid)
        {
            error_tlog("scene_fsm_alloc error, shm_mempool_find_used_next return INVALID64");
            return XYERR_PSP_ALLOCFAIL;
        }
    }
    else
    {
        xy_assert_retval(0, XYERR_XY_ASSERT_DEFAULT);
    }

    l_fsm_data = (struct scene_fsm_data *) xy_memunit_get(scene_fsm_mid);
    if (l_fsm_data == NULL)
    {
        error_tlog("scene_fsm_alloc error, xy_memunit_get return 0");
        return XYERR_PSP_ALLOCFAIL;
    }

    *fsm_data = l_fsm_data;

    return XYERR_PSP_OK;
}

static int scene_fsm_fill_state_entry()
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    struct scene_fsm_entry* entry = NULL;

    xy_assert_retval(fsm_data != NULL, XYERR_XY_ASSERT_DEFAULT);

    // UNREGISTER
    entry = &fsm_data->state_array[SCENE_FSM_UNREGISTER];
    entry->state = SCENE_FSM_UNREGISTER;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_unregister;
    entry->on_leave = scene_fsm_leave_unregister;
    entry->on_handle = scene_fsm_handle_unregister;

    // REGISTERING
    entry = &fsm_data->state_array[SCENE_FSM_REGISTERING];
    entry->state = SCENE_FSM_REGISTERING;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_registering;
    entry->on_leave = scene_fsm_leave_registering;
    entry->on_handle = scene_fsm_handle_registering;

    // REGISTERED
    entry = &fsm_data->state_array[SCENE_FSM_REGISTERED];
    entry->state = SCENE_FSM_REGISTERED;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_registered;
    entry->on_leave = scene_fsm_leave_registered;
    entry->on_handle = scene_fsm_handle_registered;

    // DATASYNING
    entry = &fsm_data->state_array[SCENE_FSM_DATASYNING];
    entry->state = SCENE_FSM_DATASYNING;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_datasyning;
    entry->on_leave = scene_fsm_leave_datasyning;
    entry->on_handle = scene_fsm_handle_datasyning;

    // DATASYNED
    entry = &fsm_data->state_array[SCENE_FSM_DATASYNED];
    entry->state = SCENE_FSM_DATASYNED;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_datasyned;
    entry->on_leave = scene_fsm_leave_datasyned;
    entry->on_handle = scene_fsm_handle_datasyned;

    // PREPARESERVICE
    entry = &fsm_data->state_array[SCENE_FSM_PREPARESERVICE];
    entry->state = SCENE_FSM_PREPARESERVICE;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_prepareservice;
    entry->on_leave = scene_fsm_leave_prepareservice;
    entry->on_handle = scene_fsm_handle_prepareservice;
    
    // INSERVICE
    entry = &fsm_data->state_array[SCENE_FSM_INSERVICE];
    entry->state = SCENE_FSM_INSERVICE;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_inservice;
    entry->on_leave = scene_fsm_leave_inservice;
    entry->on_handle = scene_fsm_handle_inservice;

    // STANDALONE
    entry = &fsm_data->state_array[SCENE_FSM_STANDALONE];
    entry->state = SCENE_FSM_STANDALONE;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_standalone;
    entry->on_leave = scene_fsm_leave_standalone;
    entry->on_handle = scene_fsm_handle_standalone;

    // STOPPING
    entry = &fsm_data->state_array[SCENE_FSM_STOPPING];
    entry->state = SCENE_FSM_STOPPING;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_stopping;
    entry->on_leave = scene_fsm_leave_stopping;
    entry->on_handle = scene_fsm_handle_stopping;

    // STOPPED
    entry = &fsm_data->state_array[SCENE_FSM_STOPPED];
    entry->state = SCENE_FSM_STOPPED;
    entry->state_timer = INVALID64;
    entry->on_enter = scene_fsm_enter_stopped;
    entry->on_leave = scene_fsm_leave_stopped;
    entry->on_handle = scene_fsm_handle_stopped;

    return XYERR_PSP_OK;
}

// scene fsm keep-alive data access

struct scene_keepalive_entry* scene_kplv_find(int id_from, int* index)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    struct scene_keepalive_entry* entry = NULL;
    int i;

    if (fsm_data == NULL)
        return NULL;

    for (i = 0; i < fsm_data->kplv_count; ++ i)
    {
        entry = &fsm_data->kplv_array[i];
        if (entry->id_from == id_from)
        {
            if (index != NULL)
                *index = i;
            return entry;
        }
    }

    return NULL;
}

int scene_kplv_reset_timeout(int id_from)
{
    struct scene_keepalive_entry* entry = scene_kplv_find(id_from, NULL);
    if (entry == NULL)
    {
#if 0
        debug_tlog("scene_fsm_reset_signal_timeout find %x : not found",
            id_from);
#endif
        return XYERR_PSP_KPLVNOTFOUND;
    }

    entry->signal_timeout = 0;

    return XYERR_PSP_OK;
}

static int scene_fsm_pull_kplv_timeout(int time_delta, int* ids, int* ids_len)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    int l_ids_len = 0;
    int i = 0, j = 0;

    if (fsm_data == NULL)
        return XYERR_PSP_NOTINITIALIZE;

    if (fsm_data->kplv_count <= 0)
        return XYERR_PSP_KPLVNOENTRY;

    l_ids_len = (fsm_data->kplv_count <= *ids_len) ? (fsm_data->kplv_count) : (*ids_len);

    for (i = 0; i < l_ids_len; ++ i)
    {
        struct scene_keepalive_entry* entry = &fsm_data->kplv_array[i];
        entry->signal_timeout += time_delta;
        if (entry->signal_timeout >= entry->timeout_max)
        {
            infor_tlog("world %x was possibily in exception-state, sig_timeout=%d, max_timeout=%d",
                       entry->id_from, entry->signal_timeout, entry->timeout_max);
            ids[j ++] = entry->id_from;
        }
    }

    *ids_len = j;

    return XYERR_PSP_OK;
}

int scene_kplv_update(int time_delta)
{
    int ids[SCENE_KEEPALIVE_ENTRY_MAX];
    int ids_len = SCENE_KEEPALIVE_ENTRY_MAX;

    int ret = scene_fsm_pull_kplv_timeout(time_delta, ids, &ids_len);
    if ((ret == XYERR_PSP_OK) && (ids_len > 0))
    {
        int i = 0, j = ids_len;
        for (; i < ids_len; ++ i)
        {
            if (get_type_by_tbus_id(ids[i]) == TBUS_ID_TYPE_WORLD)
                j = i;
        }

        if (j < ids_len)
        {
            XYPKG_SS xypkg;
            
            init_s2w_xypkg(&xypkg, CMD_PSP_WORLD_OFFLINE);// 模拟world 通知 scene,但用的协议套子正相反,但无妨
            xypkg.stBody.stPsp_pkg.stPkg.stWorld_offline.dwReason = PSP_OFFLINE_KPLV_TIMEOUT;

            scene_fsm_action(ids[j], &xypkg);
        }
    }

    return ret;
}

int scene_kplv_add(int id_from, int timeout, u64 timer_mid)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    struct scene_keepalive_entry* entry = NULL;
    int i = 0;

    if (fsm_data == NULL)
        return XYERR_PSP_NOTINITIALIZE;

    if (fsm_data->kplv_count >= SCENE_KEEPALIVE_ENTRY_MAX)
        return XYERR_PSP_KPLVENTRYFULL;

    entry = scene_kplv_find(id_from, NULL);
    if (entry != NULL)
        return XYEER_PSP_KPLVEXIST;

    i = fsm_data->kplv_count ++;

    entry = &fsm_data->kplv_array[i];
    entry->id_from = id_from;
    entry->timer_mid = timer_mid;
    entry->timeout_max = timeout;

    return XYERR_PSP_OK;
}

int scene_kplv_del(int id_from)
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    struct scene_keepalive_entry* entry = NULL;
    int index = 0;

    if (fsm_data == NULL)
        return XYERR_PSP_NOTINITIALIZE;

    entry = scene_kplv_find(id_from, &index);
    if (entry == NULL)
        return XYERR_PSP_KPLVNOTFOUND;

    if (index >= fsm_data->kplv_count)
        return XYERR_PSP_KPLVNOTFOUND;

    if (fsm_data->kplv_count > 1)
    {
        // copy the last entry to overwrite this
        struct scene_keepalive_entry* entry_swappee = &fsm_data->kplv_array[fsm_data->kplv_count - 1];
        memcpy(entry, entry_swappee, sizeof * entry);
    }

    -- fsm_data->kplv_count;

    return XYERR_PSP_OK;
}

u64 scene_fsm_kplv_get_timerid(int id_from)
{
    struct scene_keepalive_entry* entry = scene_kplv_find(id_from, NULL);
    if (entry == NULL)
        return INVALID64;

    return entry->timer_mid;
}

static int scene_kplv_init()
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();

    xy_assert_retval(fsm_data != NULL, XYERR_XY_ASSERT_DEFAULT);

    fsm_data->kplv_count = 0;

    memset(fsm_data->kplv_array, 0, sizeof(fsm_data->kplv_array));

    fsm_data->kplv_enabled = 0;
    fsm_data->psp_enabled = 0;
    fsm_data->shutdown_gracefully = 1;

    return XYERR_PSP_OK;
}

static int scene_fsm_resume()
{
    struct scene_fsm_data* fsm_data = get_scene_fsm_data();
    struct scene_fsm_entry* curr_entry = NULL;
    struct scene_fsm_entry* new_entry = NULL;

    xy_assert_retval(fsm_data != NULL, XYERR_XY_ASSERT_DEFAULT);
    
    curr_entry = scene_fsm_find(fsm_data->current_state);
    new_entry = scene_fsm_find(fsm_data->new_state);

    if ((curr_entry == new_entry) &&
        (fsm_data->trans_pending == SCENE_FSM_TRANS_PENDING_0))
    {
        fsm_data->current_entry = new_entry;
        debug_tlog("scene fsm resume, same state, data is OK");
        return XYERR_PSP_OK;
    }

    switch (fsm_data->trans_pending)
    {
    case SCENE_FSM_TRANS_PENDING_0:
        if (curr_entry != NULL)
            if (curr_entry->on_leave != NULL)
                curr_entry->on_leave(curr_entry);
        fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_1;

    case SCENE_FSM_TRANS_PENDING_1:
        fsm_data->current_state = fsm_data->new_state;
        fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_2;

    case SCENE_FSM_TRANS_PENDING_2:
        if (new_entry != NULL)
            if (new_entry->on_enter != NULL)
                new_entry->on_enter(new_entry);
        fsm_data->trans_pending = SCENE_FSM_TRANS_PENDING_0;
        break;

    default:
        error_tlog("world trans_pending corruption: %d, should be in [%d, %d)",
            fsm_data->trans_pending, SCENE_FSM_TRANS_PENDING_0,
            SCENE_FSM_TRANS_PENDING_MAX);
        break;
    }

    fsm_data->current_entry = new_entry;

    return XYERR_PSP_OK;
}

// scene fsm init & stop

int scene_fsm_init(XY_MEMPOOL_INIT_MODE mode)
{
    int ret = XYERR_PSP_OK;
    struct scene_fsm_data* fsm_data = NULL;

    ret = scene_fsm_alloc(mode, &fsm_data);
    if ((ret != XYERR_PSP_OK) || (fsm_data == NULL))
    {
        error_tlog("scene_fsm_init error, alloc failed");
        return XYERR_PSP_ALLOCFAIL;
    }

    set_scene_fsm_data(fsm_data);

    scene_fsm_fill_state_entry();

    if (mode == XY_MEMPOOL_INIT_START)
    {
        scene_kplv_init();

        scene_fsm_set_start(SCENE_FSM_UNREGISTER);
    }
    else if (mode == XY_MEMPOOL_INIT_RESUME)
    {
        scene_fsm_resume();

        //  向world同步第一次开服时间
        scene_svr_cfg_init();
        
        // 不要其这么早发送反而好一点?
        //scene_psp_on_resume();// 立即发一个探测分节给world
    }
    else
    {
        xy_assert_retval(0, XYERR_XY_ASSERT_DEFAULT);
    }

    fsm_data->mem_mode = mode;

    return XYERR_PSP_OK;
}

int scene_fsm_stop()
{
    //  目前服务器停止是延时的
    //  向状态机发送延时消息
    XYPKG_SS xypkg;
    init_s2w_xypkg(&xypkg, CMD_PSP_DELAY_STOP_SELF_REQ);
    scene_fsm_action(get_tbus_id_by_type_index(TBUS_ID_TYPE_WORLD, 1), &xypkg);
    
    return XYERR_PSP_OK;
}

void scene_delay_stop_self_timeout(void * data, size_t data_len)
{
    debug_tlog("delay stop self timeout");
    
    // 延时给状态机发送一个stop请求
    // scene_fsm_action的第一个参数将不会被使用，放在这里是为了
    // 可能的向后兼容
    XYPKG_SS xypkg;
    init_s2w_xypkg(&xypkg, CMD_PSP_STOP_SELF_REQ);
    scene_fsm_action(get_tbus_id_by_type_index(TBUS_ID_TYPE_WORLD, 1), &xypkg);
}

int scene_fsm_delay_stop_req()
{
    infor_tlog("scene %x delay stop req", get_tbus_id());
    
    //  1. 增加timer执行发送STOP消息
    u64 timer_mid = xy_add_timer(get_global_value_cfg(GLBCFG_TYPE_CLOSE_DELAY) * xy_get_tick_per_second()
                            , 1, TIMEOUT_STOP_SELF_PROC, NULL, 0);
    if (timer_mid == INVALID64)
    {
        debug_tlog("add timer failed");
        scene_delay_stop_self_timeout(NULL, 0);
    }
    
    return XYERR_PSP_OK;
}



