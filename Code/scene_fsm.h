
//
// ����������ʽ����״̬��ʵ��
//
//   �ŵ�: ���Ķ���⡢����չ��ǿ��һ��ʽ�����߼����ӶȽϸ�ʱ���١�����ǿ������ʽ
//   ȱ��: ������Ч��������һ��ʽ����ѧϰ�ɱ����Ը�������ʽ
//
// Ϊ��һ��ƽ�⣬��ѡ����ʹ������ʽ��ʵ��
//

#ifndef SCENE_FSM_H_
#define SCENE_FSM_H_

enum SCENE_FSM_STATE
{
    SCENE_FSM_UNREGISTER,           	// ��δע��ɹ����ܾ��߼�����
    SCENE_FSM_REGISTERING,          	// ����ִ��ע�ᣬ�ܾ��߼�����
    SCENE_FSM_REGISTERED,           	// ע��ɹ����ܾ��߼�����
    SCENE_FSM_DATASYNING,           	// ע��ɹ���ͬ�����ݣ��ܾ��߼�����
    SCENE_FSM_DATASYNED,            	// ͬ��������ɣ��ܾ��߼�����
    SCENE_FSM_PREPARESERVICE,          	// ׼���ṩ��������,scene��worldͬ���ݵ����Ϣ
    SCENE_FSM_INSERVICE,            	// �ṩ��������
    SCENE_FSM_STANDALONE,           	// world�����ã��ܾ��߼������ߵ������ˣ�ת��������
    SCENE_FSM_STOPPING,                 // �����رշ������У������������ݵ�world���ߵ�������
    SCENE_FSM_STOPPED,                  // ���Źرճɹ��������˳���
    SCENE_FSM_MAXSTATE,
    SCENE_FSM_NOSTATE,              	// ��״̬
};

enum SCENE_FSM_TRANS_STATE
{
    SCENE_FSM_TRANS_PENDING_0,			// trans in free state
    SCENE_FSM_TRANS_PENDING_1,			// leave old state, but current state not update yet
    SCENE_FSM_TRANS_PENDING_2,			// current state updated, but not enter new state yet
    SCENE_FSM_TRANS_PENDING_MAX,
};

#define scene_fsm_xypkg     			tagXYPKG_SS

struct scene_fsm_entry;
struct scene_fsm_xypkg;

typedef int (*scene_fsm_on_enter)(struct scene_fsm_entry* entry);
typedef int (*scene_fsm_on_leave)(struct scene_fsm_entry* entry);
typedef int (*scene_fsm_on_handle)(struct scene_fsm_entry* entry, int id_from,
                                   struct scene_fsm_xypkg* xypkg);

#define SCENE_STATE_TIMEOUT             (30)
#define SCENE_STANDALONE_TIMEOUT        (30)


struct scene_fsm_entry
{
    int state;
    u64 state_timer;     // timer for state timeout

    scene_fsm_on_enter on_enter;        // do something when enter this state
    scene_fsm_on_leave on_leave;        // do something when leave this state
    scene_fsm_on_handle on_handle;      // do something when in this state
};

#define SCENE_KEEPALIVE_ENTRY_MAX   10

struct scene_keepalive_entry
{
    int id_from;                        // remote tbus id
    u64 timer_mid;       // ping timer mid
    int timeout_max;                    // max timeout from remote
    int signal_timeout;                 // current timeout
};

struct scene_fsm_data
{
    struct scene_fsm_entry state_array[SCENE_FSM_MAXSTATE];

    struct scene_fsm_entry* current_entry;
    int current_state;
    int new_state;
    int trans_pending;                  // prevent re-entering & resume

    // keep-alive entries, such as world
    struct scene_keepalive_entry kplv_array[SCENE_KEEPALIVE_ENTRY_MAX];
    int kplv_count;

    // configuration download from world
    int psp_enabled;
    int kplv_enabled;
    int shutdown_gracefully;
    
    XY_MEMPOOL_INIT_MODE mem_mode; // resume or not,
};

// scene fsm functions
int scene_fsm_init(XY_MEMPOOL_INIT_MODE mode);
int scene_fsm_action(int id_from, struct scene_fsm_xypkg* xypkg);
int scene_fsm_trans_req(int new_state);
int scene_fsm_stop();
int scene_fsm_current_state();
int scene_fsm_delay_stop_req();
void scene_delay_stop_self_timeout(void * data, size_t data_len);

inline int scene_fsm_was_stopped()
{
    return (scene_fsm_current_state() == SCENE_FSM_STOPPED);
}

// keep-alive management & access
int scene_kplv_add(int id_from, int timeout, u64 timer_mid);
struct scene_keepalive_entry* scene_kplv_find(int id_from, int* index);
int scene_kplv_del(int id_from);
int scene_kplv_update(int time_delta);
int scene_kplv_reset_timeout(int id_from);

#endif // SCENE_FSM_H_

