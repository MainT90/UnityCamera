#include <xy_inc_x.h>
#include "xy_stat.h"
#include "xy_res_mng.h"
#include "scene_fsm_impl.h"
#include "tapp/tapp.h"
#include "xy_tbus_def.h"
#include "xy_tbus_op.h"
#include "scene_send.h"
#include "scene_actor.h"
#include "scene_actor_op.h"
#include "xy_proto_inc_ss.h"
#include "login_proto.h"
#include "xy_timer.h"
#include "scene_psp.h"
#include "scene_id_maker.h"
#include "scene_svr_cfg.h"
#include "scene_recv.h"
#include "scene_auction_send.h"
#include "router_cfgd_api.h"

#define DATA_SYNC_SPEED

#ifdef DATA_SYNC_SPEED
struct timeval s_tv1;
struct timeval s_tv2;
#endif // DATA_SYNC_SPEED

// scene receive buffer size
static size_t const g_recvbuf_sz = 1024 * 1024;

inline int offset_of_state_entry(struct scene_fsm_data* fsm_data, int state)
{
    return ((char *)&fsm_data->state_array[state] - (char *)fsm_data);
}

inline struct scene_fsm_data* container_of_state_entry(struct scene_fsm_entry* entry)
{
    return (scene_fsm_data *)((char *)entry - offset_of_state_entry(NULL, entry->state));
}

// functions
static inline void on_scene_regreq()
{
    XYPKG_SS xypkg;
    init_s2w_xypkg(&xypkg, CMD_PSP_SCENE_REGREQ_WORLD);
    scene_send_to_world(&xypkg);
}

static inline void on_scene_stopping_notify_world()
{
    XYPKG_SS xypkg;
    init_s2w_xypkg(&xypkg, CMD_PSP_SCENE_OFFLINE);
    xypkg.stBody.stPsp_pkg.stPkg.stScene_offline.dwReason = PSP_OFFLINE_OUT_OF_SERVICE;
    scene_send_to_world(&xypkg);    
}

static void on_world_regack(struct scene_fsm_entry* entry, int id_from,
                            struct scene_fsm_xypkg* xypkg)
{
    struct scene_fsm_data* fsm_data = container_of_state_entry(entry);
    int ret = 0;
    int id = get_tbus_id();

    char result = xypkg->stBody.stPsp_pkg.stPkg.stWorld_regack_scene.bReg_result;
    if (result != PSP_WS_REG_OK)
    {
        error_tlog("scene %x register to world failed, reason=%d", id, result);
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        return ;
    }
    
    fsm_data->psp_enabled = xypkg->stBody.stPsp_pkg.stPkg.stWorld_regack_scene.bPsp_enable;
    fsm_data->kplv_enabled= xypkg->stBody.stPsp_pkg.stPkg.stWorld_regack_scene.bKeepalive_enable;
    if (fsm_data->kplv_enabled != 0)
    {
        u64 timer_mid = INVALID64;
        int interval = xypkg->stBody.stPsp_pkg.stPkg.stWorld_regack_scene.dwKeepalive_interval;
        int timeout = xypkg->stBody.stPsp_pkg.stPkg.stWorld_regack_scene.dwKeepalive_timeout;
        
        // keep-alive feature will cover almost all states except STANDALONE and DATAEXPORT
        timer_mid = scene_keepalive_open(id_from, interval, timeout);
        if (timer_mid != INVALID64)
        {
            infor_tlog("scene %x keep-alive enabled for %x, interval=%d, timeout=%d", id, id_from, interval, timeout);
            ret = scene_kplv_add(id_from, timeout, timer_mid);
            if(ret != 0)
            {
                xy_del_timer(timer_mid);
            }
        }
        else
        {
            error_tlog("scene %x keep-alive enable failed, scene_keepalive_open add timer error", id);
        }
    }
    
    infor_tlog("scene %x enter registered state", id);
    scene_fsm_trans_req(SCENE_FSM_REGISTERED);
}


// scene in unregister state
int scene_fsm_enter_unregister(struct scene_fsm_entry* entry)
{
    on_scene_regreq();
    return XYERR_PSP_OK;
}

int scene_fsm_handle_unregister(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_BRDCST_SCENE:    
        // 1 world 要求所有scene 注册
        // 2 见 world_fsm_on_scene_offline() 为以后进程不清 bus 做准备
        infor_tlog("scene %x enter registering state", id);
        on_scene_regreq();
        scene_fsm_trans_req(SCENE_FSM_REGISTERING);
        break;

    case CMD_PSP_WORLD_REGACK_SCENE:    //??? 有可能在这个阶段收到这个信息么?
        on_world_regack(entry, id_from, xypkg);
        break;

    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", id, entry->state);
        on_scene_stopping_notify_world();
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    default:
        // todo: protocol protection, deny it
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_unregister(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}

// scene in registering state
int scene_fsm_enter_registering(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}

int scene_fsm_handle_registering(struct scene_fsm_entry* entry, int id_from,
                                 struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_REGACK_SCENE:
        on_world_regack(entry, id_from, xypkg);
        break;

    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", id, entry->state);
        on_scene_stopping_notify_world();
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    default:
        // todo: protocol protection, deny it
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_registering(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}

// scene in registered state
int scene_fsm_enter_registered(struct scene_fsm_entry* entry)
{
    struct scene_fsm_data* fsm_data = container_of_state_entry(entry);
    
    // check whether dirty-data of actor exist or not
    xy_assert_retval(fsm_data != NULL, XYERR_XY_ASSERT_DEFAULT);

    if (fsm_data->psp_enabled != 0)
        scene_psp_import_data();
    else
        scene_psp_erase_data_disk();    // world run without psp enabled, erase forcely

    return XYERR_PSP_OK;
}

int scene_fsm_handle_registered(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_SYN_STATUS:
        {
            int has_data_to_sync = scene_psp_has_data_to_sync();

            has_data_to_sync = 1;

			XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_STATUS);
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_status.wSyn_count = has_data_to_sync;
            scene_send_to_world(&xypkg_ss);

            if (has_data_to_sync > 0)
            {
                infor_tlog("world run with psp mode, scene %x now start data-syncing", id);
                scene_fsm_trans_req(SCENE_FSM_DATASYNING);
            }
            else if (has_data_to_sync == 0)
            {
                infor_tlog("world run with psp mode, but scene %x no data to sync", id);
            }
            else
            {
                xy_assert_retval(0, XYERR_XY_ASSERT_DEFAULT);
            }
        }
        break;

    case CMD_PSP_WORLD_SYN_SCENE_END:
        {
            //infor_tlog("scene %x trans to INSERVICE, and send end ack to world ", id);
            infor_tlog("scene %x trans to PREPARESERVICE, and send end ack to world ", id);

            XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_STATUS);
            //send_package_to_tbus_ss(&id_from, &xypkg_ss);
            scene_send_to_world(&xypkg_ss);

            //if( 0 == xypkg.stBody.stPsp_pkg.stPkg.stWorld_syn_scene_end.dwReserved )
            // 这里可以确定是resume 还是 start,但不能确定是第一次start还是后面的start
            {
                // 当是第一次初始化的时候,需要进入prepare, 等待world 统一发号施令,scene才申请guid 和instance
                scene_fsm_trans_req(SCENE_FSM_PREPARESERVICE);
            }
            //else
            //{
            //    scene_fsm_trans_req(SCENE_FSM_INSERVICE, NULL, NULL);
            //}
        }
        break;

    case CMD_PSP_KPLV_PING:
        break;

    case CMD_PSP_WORLD_OFFLINE:
        infor_tlog("scene %x detect world offline in state=%d, just exit", id, entry->state);
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", id, entry->state);
        on_scene_stopping_notify_world();
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    default:
        // todo: protocol protection, deny it
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_registered(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}

// scene in data synchronizing state
int scene_fsm_enter_datasyning(struct scene_fsm_entry* entry)
{
#ifdef DATA_SYNC_SPEED
    gettimeofday(&s_tv1, NULL);
#endif

    return XYERR_PSP_OK;
}

int scene_fsm_handle_datasyning(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_SYN_ACTORS:
        {
            // 进行数据同步
            int req_count = xypkg->stBody.stPsp_pkg.stPkg.stWorld_syn_actors.wCount;
            int syn_count = scene_psp_sync_actors(id_from, req_count);
            int data_remain = scene_psp_has_data_to_sync(); // 获取要同步的数据

            // 发送一个应答消息
            XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_ACTORS);
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wCount = syn_count;
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wRemain = data_remain;
            //send_package_to_tbus_ss(&id_from, &xypkg_ss);
            scene_send_to_world(&xypkg_ss);
            
            infor_tlog("scene %x do %d actors syned", id, syn_count);

            // 如无数据剩余，则进入同步完成状态
            if (data_remain == 0)
                scene_fsm_trans_req(SCENE_FSM_DATASYNED);
        }
        break;

    case CMD_PSP_KPLV_PING:
        break;

    case CMD_PSP_WORLD_OFFLINE:
        infor_tlog("scene %x detect world offline in state=%d, just exit", id, entry->state);
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;
        
    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", id, entry->state);
        on_scene_stopping_notify_world();
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    default:
        // todo: protocol protection, deny it
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_datasyning(struct scene_fsm_entry* entry)
{
    int id = get_tbus_id();

#ifdef DATA_SYNC_SPEED
    gettimeofday(&s_tv2, NULL);
    
    infor_tlog("scene %x do %d actor(s) sync consume %d millisecond", id,
               scene_psp_syned_count(), time_diff(&s_tv2, &s_tv1));
#endif

    return XYERR_PSP_OK;
}

// scene in data synchronized state
int scene_fsm_enter_datasyned(struct scene_fsm_entry* entry)
{
    // erase the actors' dump data
    scene_psp_erase_data_import();

    return XYERR_PSP_OK;
}

int scene_fsm_handle_datasyned(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_SYN_SCENE_END:
        {
            //infor_tlog("scene %x trans to INSERVICE, and send end ack to world ", id);
            infor_tlog("scene %x trans to PREPARESERVICE, and send end ack to world ", id);

            XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_WORLD_END);
            //send_package_to_tbus_ss(&id_from, &xypkg_ss);
            scene_send_to_world(&xypkg_ss);

            scene_fsm_trans_req(SCENE_FSM_PREPARESERVICE);
        }
        break;

    case CMD_PSP_KPLV_PING:
        break;

    case CMD_PSP_WORLD_OFFLINE:
        infor_tlog("scene detect world offline in state=%d, just exit", entry->state);
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", id, entry->state);
        on_scene_stopping_notify_world();
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    default:
        // todo: protocol protection, deny it
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_datasyned(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}


// scene in prepare service state, providing game service
int scene_fsm_enter_prepareservice(struct scene_fsm_entry* entry)
{

    return XYERR_PSP_OK;
}

int scene_fsm_handle_prepareservice(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_NOTIFY_SCENE_PREPARE:// 如果world 已经进入 prepare  // 这个协议会反复发
        {
            //  向world同步第一次开服时间
            scene_svr_cfg_init();
            
            struct scene_fsm_data* fsm_data = container_of_state_entry(entry);
            if(XY_MEMPOOL_INIT_RESUME == fsm_data->mem_mode)
            // 如果本次是resume,freeid, fort, capitol 的 shm还在,直接进入 inservice 状态
            {
                scene_fsm_trans_req(SCENE_FSM_INSERVICE);
                break;
            }

            //// 有可能此时已经可以进入inservice了(当前是 world 在 reinit 时)
            //// 不仅不需要 scene 向 world 申请 freeguid, fort, capitol 等数据,
            //// 而且在 datasync的时候,world 应该已经向 scene 同步了这些信息
            //on_scene_fsm_fully_prepare();
        
            int ret = scene_freeid_prepare();
            if (ret != 0)
            {
                error_tlog("free id prepare failed, ret = %d", ret);
                return ret;
            }
            ret = router_cfgd_api_prepare(fsm_data->mem_mode);
            if ( ret != 0 )
            {
                error_tlog("router_cfgd_api_prepare failed, ret = %d", ret);
                return ret;
            }
        }
        break;
        
    case SCENE_MSGID_FREEID_RSP:
    case SCENE_MSGID_SERVER_CONFIG_SYNC:
    case SCENE_MSGID_PREP_INSTANCE_REQ: // 创建地图
    case FORT_MSGID_SCENE_CREATE_FORT_REQ:  // 创建fort
    case FORT_MSGID_SCENE_GET_DATA_RSP:    
    case SCENE_MSGID_BOSS_STATE_W2S_SYNC_CB:
    /*
    case STORE_MSGID_MULTI_PRICE_RES_TO_SCENE:
    */
    case CFGSVR_MSGID_RES_PULL_RSP:
    case MSGID_STATIC_ROUTE_TABLE_SYNC:
        // free guid, map 初始化数据协议放行
        scene_recv_server_package(id_from, xypkg);
        break;
        
    // 以下4条协议的处理与 datasyned 保持一致,而不是与 inservice 一致
    case CMD_PSP_WORLD_OFFLINE:
        infor_tlog("scene detect world offline in state=%d, just exit", entry->state);
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    case CMD_PSP_KPLV_PING:
        break;
        
    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay        
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", id, entry->state);
        on_scene_stopping_notify_world();
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        tapp_exit_mainloop();
        break;

    default:
        {
            //scene_recv_server_package(id_from, xypkg); // 还没开始全面处理协议
            return XYERR_PSP_OK;
        }
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_prepareservice(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}


// scene in normal state, providing game service
int scene_fsm_enter_inservice(struct scene_fsm_entry* entry)
{
	res_mng_stat_reset();
	
    //[cingo] 这部分移入 scene_fsm_handle_prepareservice
    //item_send_applying_guid_req(E_ITEM_IS_GOODS, THRESHOLD_MAX_IDS);
    //item_send_applying_guid_req(E_ITEM_IS_EQUIP, THRESHOLD_MAX_IDS);

    // 通知 world 本 scene 进入 inservice 状态
    scene_send_notify_world_inservice();

    scene_auction_send_inservice_notify();
    
    return XYERR_PSP_OK;
}

//[cingo] 这部分无效代码,去掉
//enum SCENE_RECV_METHOD
//{
//    RECV_METHOD_NONE = 0,       //接收方法还未初始化
//    TRANSMIT_TO_CLIENT,         //转发到client
//    TRANSMIT_TO_SCENE,          //转发到scene
//    TRANSMIT_TO_WORLD,          //转发到world
//};
//extern RECVPKGFUN g_recv_funs[MAX_MESSAGE_ID];
//extern int scene_recv_client_package(int id_from, struct scene_fsm_xypkg* xypkg);

int scene_fsm_handle_inservice(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_OFFLINE:
        {
        /* //[cingo] kplv 不能停, 因为 等world resume 之后 还可以重新注册 从 standlone 状态转回 inservice 状态
            struct scene_keepalive_entry* kplv_entry = NULL;

            infor_tlog("scene %x detect world offline in state=%d, try to trans to STANDALONE state",
                       id, entry->state);

            kplv_entry = scene_kplv_find(id_from, NULL);
            if (kplv_entry != NULL)
            {
                u64 ping_timer_mid = kplv_entry->timer_mid;

                if (ping_timer_mid != INVALID64)
                    scene_keepalive_close(ping_timer_mid);

                scene_kplv_del(id_from);
            }
        */
        
            infor_tlog("world recv scene %x offline when in %d state", id_from, entry->state);
            int reason = xypkg->stBody.stPsp_pkg.stPkg.stWorld_offline.dwReason;
            if( PSP_OFFLINE_KPLV_TIMEOUT == reason )
            {
                // 检测到world crash 了,进入 STANDALONE 状态
                scene_fsm_trans_req(SCENE_FSM_STANDALONE);               
            }
            else if( PSP_OFFLINE_OUT_OF_SERVICE == reason )
            {
                // world 通知要关服,进入 STOPPING 状态
                scene_fsm_trans_req(SCENE_FSM_STOPPING);
            }
            else
            {
                error_tlog("invalid stWorld_offline.dwReason: id_from=%x,reason=%d", id_from, reason);
            }
            
        }
        break;

    case CMD_PSP_KPLV_PING:
        break;
        
    case CMD_PSP_DELAY_STOP_SELF_REQ:
        scene_fsm_delay_stop_req();
        break;
        
    case CMD_PSP_STOP_SELF_REQ:
        // trans to stopping state
        infor_tlog("scene %x trans to stopping state when in state=%d according to SIG_USR1",
                   id, entry->state);
        scene_fsm_trans_req(SCENE_FSM_STOPPING);
        break;
     case CMD_PSP_WORLD_BRDCST_SCENE: // 这个时候来这个消息,说明world 死了又活过来了!
         {
            //// 先将 world 状态转入 SCENE_FSM_STANDALONE, 等到下一次world 的ping 过来再重跑注册流程!
            //scene_fsm_trans_req(SCENE_FSM_STANDALONE, NULL, NULL);
            //// 1 在进入 standalone 的时候,所有玩家已经被 kick out 了!        
            //// 2 玩家数据已经被保存了

            // 1
            infor_tlog("world reinit: kick out actors and export actors' data to disk"
                "because rece msg CMD_PSP_WORLD_BRDCST_SCENE"); 
            // create the "kick out all actors" timer
            // kickout all actors directly, just a temp solution
            scene_psp_kickout_actors_timeout(NULL, 0);
            // export actors' data to disk
            scene_psp_export_data();
            // TODO:
            // a. world 和 torm 已连通, 这里也可以将 actorlist 直接存db 了
            // b. 需要存 db 的还有 fort, capitol 等全局数据 (free id 不需要)
            
            // 2 保守的方案是 scene 先关掉, 等待 检测脚本把 scene 重新拉起来, 全新初始化
            // 因为上面 玩家都踢了,actorlist 也 本地存盘了, (fort, capitol 也存盘了)
            // 在 scene 重新初始化 注册的过程中会将 本地存盘的 actorlist 存回db
            infor_tlog("world reinit: scene %x trans to stopped state when in state=%d "
                    "but rece msg CMD_PSP_WORLD_BRDCST_SCENE", id, entry->state);
                                       
            // 标记本次 stopped 的原因, 以便检测脚本把 scene 重新拉起来做全新初始化 而不是 resume
            srp_set_down_reason(SCENE_DOWN_REASON_WORLD_REINIT);

            //
            scene_fsm_trans_req(SCENE_FSM_STOPPED);// 直接stop,不通知 world
            // 要不要延迟stop呢，不要,延迟就必然会通知到world,赶紧重启就好了

        /*  // 2 另一种方案是: 在 world reinit 的时候, scene 不重启, world 从scene同步重建数据
            // 可能吗? world 上的逻辑中的静态变量等,不会达到与scene一致的状态
        
            // 应答world 要求所有scene注册的请求
            on_scene_regreq();       

            // scene 配合world的流程,重新跑一遍注册流程
            infor_tlog("world reinit: scene %x enter registering state", id);
            scene_fsm_trans_req(SCENE_FSM_REGISTERING, NULL, NULL);
        */

         }
         break;

    default:
        {
            scene_recv_server_package(id_from, xypkg);
            return XYERR_PSP_OK;
        }
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_inservice(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}

// scene in standalone state
int scene_fsm_enter_standalone(struct scene_fsm_entry* entry)
{
    int bus_id = get_tbus_id();
  
    // 1
    infor_tlog("scene %x enter standalone state", bus_id);
    // create the "kick out all actors" timer
    // kickout all actors directly, just a temp solution
    scene_psp_kickout_actors_timeout(NULL, 0);
    // export actors' data to disk
    scene_psp_export_data();

    // 无论如何不自动进入stopped 状态
    //// world出问题了，直接切换到stopped状态，等keep-alive脚本来停掉自己
    //xy_add_timer(1 * xy_get_tick_per_second(), 1, TIMEOUT_PSP_SCENE_STANDALONE, NULL, 0);    

	// chuishi:   不管怎么样都应该有个超时，只是时间长短的问题
	xy_add_timer(SCENE_STANDALONE_TIMEOUT * xy_get_tick_per_second(), 1, TIMEOUT_PSP_SCENE_STANDALONE, NULL, 0);    

    return XYERR_PSP_OK;
}

int scene_fsm_handle_standalone(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int bus_id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_SCENE_STANDALONE_TIMEOUT:
        infor_tlog("scene %x STANDALONE state timeout, now trans to STOPPED", bus_id);
        scene_fsm_trans_req(SCENE_FSM_STOPPED); // 无论如何不自动进入stopped 状态
        break;
     case CMD_PSP_KPLV_PING: 
        break;
     case CMD_PSP_WORLD_BRDCST_SCENE:    // world 活过来了! reinit 了
         {
            // 1 在进入 standalone 的时候,所有玩家已经被 kick out 了! 已经被存盘了
            
            // TODO:
            // a. world 和 torm 已连通, 这里也可以将 本地存盘的actorlist 直接存db 了,还是等到重启之后同步的时候再存db?
            // b. 需要存 db 的还有 fort, capitol 等全局数据 (free id 不需要),此时存盘还是存db?

             
            // 2 保守的方案是 scene 先关掉, 等待 检测脚本把 scene 重新拉起来, 全新初始化
            // 因为上面 玩家都踢了,actorlist 也 本地存盘了, (fort, capitol 也存盘了)
            // 在 scene 重新初始化 注册的过程中会将 本地存盘的 actorlist 存回db
            infor_tlog("world reinit: scene %x trans to stopped state when in state=%d according to SIG_USR1",
                   bus_id, entry->state);
                   
            // 标记本次 stopped 的原因, 以便检测脚本把 scene 重新拉起来做全新初始化 而不是 resume
            srp_set_down_reason(SCENE_DOWN_REASON_WORLD_REINIT);

            //
            scene_fsm_trans_req(SCENE_FSM_STOPPED);// 直接stop,不通知 world 
            // 要不要延迟stop呢，不要,延迟就必然会通知到world,赶紧重启就好了

        /*  // 2 另一种方案是: 在 world reinit 的时候, scene 不重启, world 从scene同步重建数据
            // 可能吗? world 上的逻辑中的静态变量等,不会达到与scene一致的状态
        
            // 应答world 要求所有scene注册的请求
            on_scene_regreq();       

            // scene 配合world的流程,重新跑一遍注册流程
            infor_tlog("world reinit: scene %x enter registering state", id);
            scene_fsm_trans_req(SCENE_FSM_REGISTERING, NULL, NULL);
        */
         }
         break;
     
     case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
     case CMD_PSP_STOP_SELF_REQ:
         infor_tlog("scene %x exit when in state=%d according to SIG_USR1", bus_id, entry->state);
         on_scene_stopping_notify_world();
         tapp_exit_mainloop();
         break;
    }
    
    return XYERR_PSP_OK;
}

int scene_fsm_leave_standalone(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}


/*
//world/stop.sh 的时候
// a 将world_fsm 进入WORLD_FSM_STOPPING状态，
// b 将world_scene 进入 WORLD_SCENE_STOPSYNING 状态
//		b.1 (现在 world scene 进入STOPSYNING 状态时会主动发一个 CMD_PSP_WORLD_OFFLINE 通知所有scene 本world要stop)，
//		b.2 并设置一个timer(60s)监控此状态超时,
//	 	b.3 (若超时则直接将 world_scene 转入 WORLD_SCENE_STOPPED 状态(如果所有world scene stop, world 也进入状态 stopped)
//			   --之前的代码如果 world/stop.sh 后不及时 scene/stop.sh, 就很可能导致此超时;
//			   --现在有了 b.1之后这里不会超时了, 因为 scene 总是会及时存盘退出
// c scene 收到 world 的 CMD_PSP_WORLD_OFFLINE 通知后,转入 stopping 状态
//		c.1 发送 CMD_PSP_SCENE_STOP_REQ 协议(scene通知world自己将存盘并退出)，
//		c.2 增加 stopping 超时监控,若超时给自己发送 CMD_PSP_SCENE_STOPPING_TIMEOUT 消息,处理该消息将进入 SCENE_FSM_STANDALONE 状态(下面e.1)
// d world 收到 CMD_PSP_SCENE_STOP_REQ 协议后，使 world_scene 从 WORLD_SCENE_STOPSYNING状态 进入 WORLD_SCENE_STOPPING 状态，
// 		d.1 world_scene 进入 WORLD_SCENE_STOPPING 状态时，发送协议 CMD_PSP_WORLD_SYN_ACTORS 要求scene同步数据，
//			(并设置timer 监控 同步数据超时,若超时则 发送协议 CMD_PSP_SCENE_OFFLINE, 将相应的 world scene 转为 WORLD_SCENE_STOPPED 状态)
// e scene 收到协议 CMD_PSP_WORLD_SYN_ACTORS 时, 
//		e.1 向 world 回应 CMD_PSP_SCENE_ACK_ACTORS 协议,依次同步数据,
//				若此过程中 world 离线,或 stopping 超时(上面c.2),转入 STANDALONE 状态,(进入 STANDALONE 状态时踢掉所有玩家并本地存盘 )
//		e.2 如果剩余未保存的玩家不为0，继续d.2; 
//			如果剩余未保存的玩家为0,说明同步完成，scene自己转入stopped状态; 
// f world 收到CMD_PSP_SCENE_ACK_ACTORS 协议后, 
//		    如果剩余未保存的玩家为0,说明同步完成，world_scene 也进入stopped状态(见下面 2.2)
*/
// scene in stopping state

inline void scene_stopping_addmon(struct scene_fsm_entry* entry)
{
    if (entry->state_timer != INVALID64)
    {
        xy_del_timer(entry->state_timer);
        entry->state_timer = INVALID64;
    }

    u64 timer = xy_add_timer(SCENE_STATE_TIMEOUT * xy_get_tick_per_second(),
                TIMER_RUN_FOREVER, TIMEOUT_PSP_SCENE_STOPPING, NULL, 0);
    if (timer != INVALID64)
    {
        entry->state_timer = timer;
    }
}

int scene_fsm_enter_stopping(struct scene_fsm_entry* entry)
{
    int bus_id = get_tbus_id();
    
    infor_tlog("scene %x enter stopping state", bus_id);

    // 发送请求包 --通知world自己将存盘并退出
    {
        int ret = 0;
        int world_addr = get_tbus_id_by_type_index(TBUS_ID_TYPE_WORLD, 1);
        
        XYPKG_SS xypkg_ss;
        init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_STOP_REQ);
        scene_send_to_world(&xypkg_ss);
        infor_tlog("scene %x send stop-req to world %x, retval=%d", get_tbus_id(), world_addr, ret);
    }

    // 增加timer监控对端是否超时，超时则直接退出
    //[cingo]-- 其实是超时则给自己发送 CMD_PSP_SCENE_STOPPING_TIMEOUT 消息,处理该消息将进入 SCENE_FSM_STANDALONE 状态
    // 这里没有使用keep-alive监控world的可用状态，即这种边界情况目前不予处理
    scene_stopping_addmon(entry);
   
    return XYERR_PSP_OK;
}

int scene_fsm_handle_stopping(struct scene_fsm_entry* entry, int id_from,
                              struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int bus_id = get_tbus_id();

    switch (msg_id)
    {
    case CMD_PSP_WORLD_SYN_ACTORS:
        {
            // 进行数据同步-- 发送actor数据存盘协议
            int req_count = xypkg->stBody.stPsp_pkg.stPkg.stWorld_syn_actors.wCount;
            int syn_count = scene_stopping_syn_actors(id_from, req_count);
#warning bok, 暂时加在这里，后面分离
            int data_remain = scene_stopping_unsyn_actors_count();
    
            // 发送一个应答消息-- 每一条actor数据存盘协议之后跟一条这个协议
            XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_ACTORS);
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wCount = syn_count;
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wRemain = data_remain;
            scene_send_to_world(&xypkg_ss);
            
            infor_tlog("scene %x do %d actor(s) saved, %d actor(s) remain",
                       bus_id, syn_count, data_remain);
    
            if (data_remain == 0)// 如果剩余没同步的玩家数量为0
            {
                scene_stopping_sync_capitol();// 也停止同步城战 
                // 这里会不会有问题,在同步 actor 的过程中,城战一定会存了吗
                // 还有 fort 呢,fort是即用即存的
                                      
                // 标记本次 stopped 的原因, 以示是正常关闭 -- 经历过此阶段的都是正常关闭
                srp_set_down_reason(SERVER_DOWN_REASON_BE_STOPPED);
                
                // 如无数据剩余，则进入可以退出状态
                scene_fsm_trans_req(SCENE_FSM_STOPPED);
            }
            else
            {
                // 仍然需要加超时监控
                scene_stopping_addmon(entry);
            }
        }
        break;


    case CMD_PSP_WORLD_OFFLINE:           // world 离线
    	if(PSP_OFFLINE_KPLV_TIMEOUT == xypkg->stBody.stPsp_pkg.stPkg.stWorld_offline.dwReason)
    	{
        	infor_tlog("scene %x can't get world, now trans to STANDALONE", bus_id);
        	scene_fsm_trans_req(SCENE_FSM_STANDALONE);
    	}
        break;
    case CMD_PSP_SCENE_STOPPING_TIMEOUT:  // 进入stopping状态开始同步数据时 种的 timer 超时了
        // stopping 状态下 向 world同步数据出错,对 actor 等本地存盘, 让 scene 转入 stopped 状态
        infor_tlog("scene %x STOPPING state timeout, can't sync to world, "
            "now save actors to local dat file, then trans to STOPPED", bus_id);
        scene_fsm_trans_req(SCENE_FSM_STANDALONE);
        // 然后延迟发送 CMD_PSP_STOP_SELF_REQ -- 已经在同步数据了,一定是之前从inservice状态下转过来时发过stop 了
        // 既然不能向world存入,那么本地保存磁盘吧
        // create the "kick out all actors" timer
        // kickout all actors directly, just a temp solution
        scene_psp_kickout_actors_timeout(NULL, 0);
        // export actors' data to disk
        scene_psp_export_data();
        // 本地保存后退出游戏
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        // 这个stop 一定是从 inservice 状态下就发起,先延迟了30s(系统配置)以等待inservice 逻辑结束
        // 然后 又等了至少30s 向 world sync 数据失败,然后才本地存盘,转入 stopped 状态
        break;
    
    case CMD_PSP_DELAY_STOP_SELF_REQ: 
    case CMD_PSP_STOP_SELF_REQ:         // 在stopping 状态下不能直接 stop
        {
            // stopping 状态下遇到 stop 指令,延迟处理
            int data_remain = scene_stopping_unsyn_actors_count();    
            if (data_remain > 0)   // 如果还没同步完
            {
                scene_fsm_delay_stop_req(); // 延迟发送stop命令
                break;
            }
            // 理论上不应走到这里
            scene_fsm_trans_req(SCENE_FSM_STOPPED);
        }
        break;
    default:
        break;
    }

    return XYERR_PSP_OK;
}

int scene_fsm_leave_stopping(struct scene_fsm_entry* entry)
{
    if (entry->state_timer != INVALID64)
    {
        xy_del_timer(entry->state_timer);
        entry->state_timer = INVALID64;
    }
    
    return XYERR_PSP_OK;
}

// scene in stopped state
int scene_fsm_enter_stopped(struct scene_fsm_entry* entry)
{
    infor_tlog("scene %x enter stopped state", get_tbus_id());
   
    //tapp_exit_mainloop();   // 要不要这么做?

    return XYERR_PSP_OK;
}

int scene_fsm_handle_stopped(struct scene_fsm_entry* entry, int id_from,
                             struct scene_fsm_xypkg* xypkg)
{
    int msg_id = xypkg->stHeader.nMsgid;
    int bus_id = get_tbus_id();

    switch (msg_id)
    {     
    case CMD_PSP_DELAY_STOP_SELF_REQ:   //  don't delay
    case CMD_PSP_STOP_SELF_REQ:
        infor_tlog("scene %x exit when in state=%d according to SIG_USR1", bus_id, entry->state);
        on_scene_stopping_notify_world();
        tapp_exit_mainloop();
        break;

    default:
        break;
    }
    return XYERR_PSP_OK;
}

int scene_fsm_leave_stopped(struct scene_fsm_entry* entry)
{
    return XYERR_PSP_OK;
}

