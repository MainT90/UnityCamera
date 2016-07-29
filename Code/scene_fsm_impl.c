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
        // 1 world Ҫ������scene ע��
        // 2 �� world_fsm_on_scene_offline() Ϊ�Ժ���̲��� bus ��׼��
        infor_tlog("scene %x enter registering state", id);
        on_scene_regreq();
        scene_fsm_trans_req(SCENE_FSM_REGISTERING);
        break;

    case CMD_PSP_WORLD_REGACK_SCENE:    //??? �п���������׶��յ������Ϣô?
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
            // �������ȷ����resume ���� start,������ȷ���ǵ�һ��start���Ǻ����start
            {
                // ���ǵ�һ�γ�ʼ����ʱ��,��Ҫ����prepare, �ȴ�world ͳһ����ʩ��,scene������guid ��instance
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
            // ��������ͬ��
            int req_count = xypkg->stBody.stPsp_pkg.stPkg.stWorld_syn_actors.wCount;
            int syn_count = scene_psp_sync_actors(id_from, req_count);
            int data_remain = scene_psp_has_data_to_sync(); // ��ȡҪͬ��������

            // ����һ��Ӧ����Ϣ
            XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_ACTORS);
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wCount = syn_count;
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wRemain = data_remain;
            //send_package_to_tbus_ss(&id_from, &xypkg_ss);
            scene_send_to_world(&xypkg_ss);
            
            infor_tlog("scene %x do %d actors syned", id, syn_count);

            // ��������ʣ�࣬�����ͬ�����״̬
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
    case CMD_PSP_WORLD_NOTIFY_SCENE_PREPARE:// ���world �Ѿ����� prepare  // ���Э��ᷴ����
        {
            //  ��worldͬ����һ�ο���ʱ��
            scene_svr_cfg_init();
            
            struct scene_fsm_data* fsm_data = container_of_state_entry(entry);
            if(XY_MEMPOOL_INIT_RESUME == fsm_data->mem_mode)
            // ���������resume,freeid, fort, capitol �� shm����,ֱ�ӽ��� inservice ״̬
            {
                scene_fsm_trans_req(SCENE_FSM_INSERVICE);
                break;
            }

            //// �п��ܴ�ʱ�Ѿ����Խ���inservice��(��ǰ�� world �� reinit ʱ)
            //// ��������Ҫ scene �� world ���� freeguid, fort, capitol ������,
            //// ������ datasync��ʱ��,world Ӧ���Ѿ��� scene ͬ������Щ��Ϣ
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
    case SCENE_MSGID_PREP_INSTANCE_REQ: // ������ͼ
    case FORT_MSGID_SCENE_CREATE_FORT_REQ:  // ����fort
    case FORT_MSGID_SCENE_GET_DATA_RSP:    
    case SCENE_MSGID_BOSS_STATE_W2S_SYNC_CB:
    /*
    case STORE_MSGID_MULTI_PRICE_RES_TO_SCENE:
    */
    case CFGSVR_MSGID_RES_PULL_RSP:
    case MSGID_STATIC_ROUTE_TABLE_SYNC:
        // free guid, map ��ʼ������Э�����
        scene_recv_server_package(id_from, xypkg);
        break;
        
    // ����4��Э��Ĵ����� datasyned ����һ��,�������� inservice һ��
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
            //scene_recv_server_package(id_from, xypkg); // ��û��ʼȫ�洦��Э��
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
	
    //[cingo] �ⲿ������ scene_fsm_handle_prepareservice
    //item_send_applying_guid_req(E_ITEM_IS_GOODS, THRESHOLD_MAX_IDS);
    //item_send_applying_guid_req(E_ITEM_IS_EQUIP, THRESHOLD_MAX_IDS);

    // ֪ͨ world �� scene ���� inservice ״̬
    scene_send_notify_world_inservice();

    scene_auction_send_inservice_notify();
    
    return XYERR_PSP_OK;
}

//[cingo] �ⲿ����Ч����,ȥ��
//enum SCENE_RECV_METHOD
//{
//    RECV_METHOD_NONE = 0,       //���շ�����δ��ʼ��
//    TRANSMIT_TO_CLIENT,         //ת����client
//    TRANSMIT_TO_SCENE,          //ת����scene
//    TRANSMIT_TO_WORLD,          //ת����world
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
        /* //[cingo] kplv ����ͣ, ��Ϊ ��world resume ֮�� ����������ע�� �� standlone ״̬ת�� inservice ״̬
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
                // ��⵽world crash ��,���� STANDALONE ״̬
                scene_fsm_trans_req(SCENE_FSM_STANDALONE);               
            }
            else if( PSP_OFFLINE_OUT_OF_SERVICE == reason )
            {
                // world ֪ͨҪ�ط�,���� STOPPING ״̬
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
     case CMD_PSP_WORLD_BRDCST_SCENE: // ���ʱ���������Ϣ,˵��world �����ֻ������!
         {
            //// �Ƚ� world ״̬ת�� SCENE_FSM_STANDALONE, �ȵ���һ��world ��ping ����������ע������!
            //scene_fsm_trans_req(SCENE_FSM_STANDALONE, NULL, NULL);
            //// 1 �ڽ��� standalone ��ʱ��,��������Ѿ��� kick out ��!        
            //// 2 ��������Ѿ���������

            // 1
            infor_tlog("world reinit: kick out actors and export actors' data to disk"
                "because rece msg CMD_PSP_WORLD_BRDCST_SCENE"); 
            // create the "kick out all actors" timer
            // kickout all actors directly, just a temp solution
            scene_psp_kickout_actors_timeout(NULL, 0);
            // export actors' data to disk
            scene_psp_export_data();
            // TODO:
            // a. world �� torm ����ͨ, ����Ҳ���Խ� actorlist ֱ�Ӵ�db ��
            // b. ��Ҫ�� db �Ļ��� fort, capitol ��ȫ������ (free id ����Ҫ)
            
            // 2 ���صķ����� scene �ȹص�, �ȴ� ���ű��� scene ����������, ȫ�³�ʼ��
            // ��Ϊ���� ��Ҷ�����,actorlist Ҳ ���ش�����, (fort, capitol Ҳ������)
            // �� scene ���³�ʼ�� ע��Ĺ����лὫ ���ش��̵� actorlist ���db
            infor_tlog("world reinit: scene %x trans to stopped state when in state=%d "
                    "but rece msg CMD_PSP_WORLD_BRDCST_SCENE", id, entry->state);
                                       
            // ��Ǳ��� stopped ��ԭ��, �Ա���ű��� scene ������������ȫ�³�ʼ�� ������ resume
            srp_set_down_reason(SCENE_DOWN_REASON_WORLD_REINIT);

            //
            scene_fsm_trans_req(SCENE_FSM_STOPPED);// ֱ��stop,��֪ͨ world
            // Ҫ��Ҫ�ӳ�stop�أ���Ҫ,�ӳپͱ�Ȼ��֪ͨ��world,�Ͻ������ͺ���

        /*  // 2 ��һ�ַ�����: �� world reinit ��ʱ��, scene ������, world ��sceneͬ���ؽ�����
            // ������? world �ϵ��߼��еľ�̬������,����ﵽ��sceneһ�µ�״̬
        
            // Ӧ��world Ҫ������sceneע�������
            on_scene_regreq();       

            // scene ���world������,������һ��ע������
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

    // ������β��Զ�����stopped ״̬
    //// world�������ˣ�ֱ���л���stopped״̬����keep-alive�ű���ͣ���Լ�
    //xy_add_timer(1 * xy_get_tick_per_second(), 1, TIMEOUT_PSP_SCENE_STANDALONE, NULL, 0);    

	// chuishi:   ������ô����Ӧ���и���ʱ��ֻ��ʱ�䳤�̵�����
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
        scene_fsm_trans_req(SCENE_FSM_STOPPED); // ������β��Զ�����stopped ״̬
        break;
     case CMD_PSP_KPLV_PING: 
        break;
     case CMD_PSP_WORLD_BRDCST_SCENE:    // world �������! reinit ��
         {
            // 1 �ڽ��� standalone ��ʱ��,��������Ѿ��� kick out ��! �Ѿ���������
            
            // TODO:
            // a. world �� torm ����ͨ, ����Ҳ���Խ� ���ش��̵�actorlist ֱ�Ӵ�db ��,���ǵȵ�����֮��ͬ����ʱ���ٴ�db?
            // b. ��Ҫ�� db �Ļ��� fort, capitol ��ȫ������ (free id ����Ҫ),��ʱ���̻��Ǵ�db?

             
            // 2 ���صķ����� scene �ȹص�, �ȴ� ���ű��� scene ����������, ȫ�³�ʼ��
            // ��Ϊ���� ��Ҷ�����,actorlist Ҳ ���ش�����, (fort, capitol Ҳ������)
            // �� scene ���³�ʼ�� ע��Ĺ����лὫ ���ش��̵� actorlist ���db
            infor_tlog("world reinit: scene %x trans to stopped state when in state=%d according to SIG_USR1",
                   bus_id, entry->state);
                   
            // ��Ǳ��� stopped ��ԭ��, �Ա���ű��� scene ������������ȫ�³�ʼ�� ������ resume
            srp_set_down_reason(SCENE_DOWN_REASON_WORLD_REINIT);

            //
            scene_fsm_trans_req(SCENE_FSM_STOPPED);// ֱ��stop,��֪ͨ world 
            // Ҫ��Ҫ�ӳ�stop�أ���Ҫ,�ӳپͱ�Ȼ��֪ͨ��world,�Ͻ������ͺ���

        /*  // 2 ��һ�ַ�����: �� world reinit ��ʱ��, scene ������, world ��sceneͬ���ؽ�����
            // ������? world �ϵ��߼��еľ�̬������,����ﵽ��sceneһ�µ�״̬
        
            // Ӧ��world Ҫ������sceneע�������
            on_scene_regreq();       

            // scene ���world������,������һ��ע������
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
//world/stop.sh ��ʱ��
// a ��world_fsm ����WORLD_FSM_STOPPING״̬��
// b ��world_scene ���� WORLD_SCENE_STOPSYNING ״̬
//		b.1 (���� world scene ����STOPSYNING ״̬ʱ��������һ�� CMD_PSP_WORLD_OFFLINE ֪ͨ����scene ��worldҪstop)��
//		b.2 ������һ��timer(60s)��ش�״̬��ʱ,
//	 	b.3 (����ʱ��ֱ�ӽ� world_scene ת�� WORLD_SCENE_STOPPED ״̬(�������world scene stop, world Ҳ����״̬ stopped)
//			   --֮ǰ�Ĵ������ world/stop.sh �󲻼�ʱ scene/stop.sh, �ͺܿ��ܵ��´˳�ʱ;
//			   --�������� b.1֮�����ﲻ�ᳬʱ��, ��Ϊ scene ���ǻἰʱ�����˳�
// c scene �յ� world �� CMD_PSP_WORLD_OFFLINE ֪ͨ��,ת�� stopping ״̬
//		c.1 ���� CMD_PSP_SCENE_STOP_REQ Э��(scene֪ͨworld�Լ������̲��˳�)��
//		c.2 ���� stopping ��ʱ���,����ʱ���Լ����� CMD_PSP_SCENE_STOPPING_TIMEOUT ��Ϣ,�������Ϣ������ SCENE_FSM_STANDALONE ״̬(����e.1)
// d world �յ� CMD_PSP_SCENE_STOP_REQ Э���ʹ world_scene �� WORLD_SCENE_STOPSYNING״̬ ���� WORLD_SCENE_STOPPING ״̬��
// 		d.1 world_scene ���� WORLD_SCENE_STOPPING ״̬ʱ������Э�� CMD_PSP_WORLD_SYN_ACTORS Ҫ��sceneͬ�����ݣ�
//			(������timer ��� ͬ�����ݳ�ʱ,����ʱ�� ����Э�� CMD_PSP_SCENE_OFFLINE, ����Ӧ�� world scene תΪ WORLD_SCENE_STOPPED ״̬)
// e scene �յ�Э�� CMD_PSP_WORLD_SYN_ACTORS ʱ, 
//		e.1 �� world ��Ӧ CMD_PSP_SCENE_ACK_ACTORS Э��,����ͬ������,
//				���˹����� world ����,�� stopping ��ʱ(����c.2),ת�� STANDALONE ״̬,(���� STANDALONE ״̬ʱ�ߵ�������Ҳ����ش��� )
//		e.2 ���ʣ��δ�������Ҳ�Ϊ0������d.2; 
//			���ʣ��δ��������Ϊ0,˵��ͬ����ɣ�scene�Լ�ת��stopped״̬; 
// f world �յ�CMD_PSP_SCENE_ACK_ACTORS Э���, 
//		    ���ʣ��δ��������Ϊ0,˵��ͬ����ɣ�world_scene Ҳ����stopped״̬(������ 2.2)
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

    // ��������� --֪ͨworld�Լ������̲��˳�
    {
        int ret = 0;
        int world_addr = get_tbus_id_by_type_index(TBUS_ID_TYPE_WORLD, 1);
        
        XYPKG_SS xypkg_ss;
        init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_STOP_REQ);
        scene_send_to_world(&xypkg_ss);
        infor_tlog("scene %x send stop-req to world %x, retval=%d", get_tbus_id(), world_addr, ret);
    }

    // ����timer��ضԶ��Ƿ�ʱ����ʱ��ֱ���˳�
    //[cingo]-- ��ʵ�ǳ�ʱ����Լ����� CMD_PSP_SCENE_STOPPING_TIMEOUT ��Ϣ,�������Ϣ������ SCENE_FSM_STANDALONE ״̬
    // ����û��ʹ��keep-alive���world�Ŀ���״̬�������ֱ߽����Ŀǰ���账��
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
            // ��������ͬ��-- ����actor���ݴ���Э��
            int req_count = xypkg->stBody.stPsp_pkg.stPkg.stWorld_syn_actors.wCount;
            int syn_count = scene_stopping_syn_actors(id_from, req_count);
#warning bok, ��ʱ��������������
            int data_remain = scene_stopping_unsyn_actors_count();
    
            // ����һ��Ӧ����Ϣ-- ÿһ��actor���ݴ���Э��֮���һ�����Э��
            XYPKG_SS xypkg_ss;
            init_s2w_xypkg(&xypkg_ss, CMD_PSP_SCENE_ACK_ACTORS);
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wCount = syn_count;
            xypkg_ss.stBody.stPsp_pkg.stPkg.stScene_ack_actors.wRemain = data_remain;
            scene_send_to_world(&xypkg_ss);
            
            infor_tlog("scene %x do %d actor(s) saved, %d actor(s) remain",
                       bus_id, syn_count, data_remain);
    
            if (data_remain == 0)// ���ʣ��ûͬ�����������Ϊ0
            {
                scene_stopping_sync_capitol();// Ҳֹͣͬ����ս 
                // ����᲻��������,��ͬ�� actor �Ĺ�����,��սһ���������
                // ���� fort ��,fort�Ǽ��ü����
                                      
                // ��Ǳ��� stopped ��ԭ��, ��ʾ�������ر� -- �������˽׶εĶ��������ر�
                srp_set_down_reason(SERVER_DOWN_REASON_BE_STOPPED);
                
                // ��������ʣ�࣬���������˳�״̬
                scene_fsm_trans_req(SCENE_FSM_STOPPED);
            }
            else
            {
                // ��Ȼ��Ҫ�ӳ�ʱ���
                scene_stopping_addmon(entry);
            }
        }
        break;


    case CMD_PSP_WORLD_OFFLINE:           // world ����
    	if(PSP_OFFLINE_KPLV_TIMEOUT == xypkg->stBody.stPsp_pkg.stPkg.stWorld_offline.dwReason)
    	{
        	infor_tlog("scene %x can't get world, now trans to STANDALONE", bus_id);
        	scene_fsm_trans_req(SCENE_FSM_STANDALONE);
    	}
        break;
    case CMD_PSP_SCENE_STOPPING_TIMEOUT:  // ����stopping״̬��ʼͬ������ʱ �ֵ� timer ��ʱ��
        // stopping ״̬�� �� worldͬ�����ݳ���,�� actor �ȱ��ش���, �� scene ת�� stopped ״̬
        infor_tlog("scene %x STOPPING state timeout, can't sync to world, "
            "now save actors to local dat file, then trans to STOPPED", bus_id);
        scene_fsm_trans_req(SCENE_FSM_STANDALONE);
        // Ȼ���ӳٷ��� CMD_PSP_STOP_SELF_REQ -- �Ѿ���ͬ��������,һ����֮ǰ��inservice״̬��ת����ʱ����stop ��
        // ��Ȼ������world����,��ô���ر�����̰�
        // create the "kick out all actors" timer
        // kickout all actors directly, just a temp solution
        scene_psp_kickout_actors_timeout(NULL, 0);
        // export actors' data to disk
        scene_psp_export_data();
        // ���ر�����˳���Ϸ
        scene_fsm_trans_req(SCENE_FSM_STOPPED);
        // ���stop һ���Ǵ� inservice ״̬�¾ͷ���,���ӳ���30s(ϵͳ����)�Եȴ�inservice �߼�����
        // Ȼ�� �ֵ�������30s �� world sync ����ʧ��,Ȼ��ű��ش���,ת�� stopped ״̬
        break;
    
    case CMD_PSP_DELAY_STOP_SELF_REQ: 
    case CMD_PSP_STOP_SELF_REQ:         // ��stopping ״̬�²���ֱ�� stop
        {
            // stopping ״̬������ stop ָ��,�ӳٴ���
            int data_remain = scene_stopping_unsyn_actors_count();    
            if (data_remain > 0)   // �����ûͬ����
            {
                scene_fsm_delay_stop_req(); // �ӳٷ���stop����
                break;
            }
            // �����ϲ�Ӧ�ߵ�����
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
   
    //tapp_exit_mainloop();   // Ҫ��Ҫ��ô��?

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

