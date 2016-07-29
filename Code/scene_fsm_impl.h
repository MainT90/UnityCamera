
#ifndef SCENE_FSM_IMPL_H_
#define SCENE_FSM_IMPL_H_

// scene�ĸ���״̬����ʵ��

#include "scene_fsm.h"

// scene��δע��״̬
int scene_fsm_enter_unregister(struct scene_fsm_entry* entry);
int scene_fsm_handle_unregister(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_unregister(struct scene_fsm_entry* entry);

// scene����ִ��ע��״̬
int scene_fsm_enter_registering(struct scene_fsm_entry* entry);
int scene_fsm_handle_registering(struct scene_fsm_entry* entry, int id_from,
                                 struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_registering(struct scene_fsm_entry* entry);

// scene���ע��״̬
int scene_fsm_enter_registered(struct scene_fsm_entry* entry);
int scene_fsm_handle_registered(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_registered(struct scene_fsm_entry* entry);

// sceneͬ��������
int scene_fsm_enter_datasyning(struct scene_fsm_entry* entry);
int scene_fsm_handle_datasyning(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_datasyning(struct scene_fsm_entry* entry);

// sceneͬ���������
int scene_fsm_enter_datasyned(struct scene_fsm_entry* entry);
int scene_fsm_handle_datasyned(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_datasyned(struct scene_fsm_entry* entry);

// scene׼������״̬
int scene_fsm_enter_prepareservice(struct scene_fsm_entry* entry);
int scene_fsm_handle_prepareservice(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_prepareservice(struct scene_fsm_entry* entry);
// scene��������״̬
int scene_fsm_enter_inservice(struct scene_fsm_entry* entry);
int scene_fsm_handle_inservice(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_inservice(struct scene_fsm_entry* entry);

// scene�޷��õ�world֧��״̬��׼�������˳�
int scene_fsm_enter_standalone(struct scene_fsm_entry* entry);
int scene_fsm_handle_standalone(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_standalone(struct scene_fsm_entry* entry);

// scene�ط�״̬�����ˡ����̡��˳�
int scene_fsm_enter_stopping(struct scene_fsm_entry* entry);
int scene_fsm_handle_stopping(struct scene_fsm_entry* entry, int id_from,
                              struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_stopping(struct scene_fsm_entry* entry);

// scene���Źرճɹ�
int scene_fsm_enter_stopped(struct scene_fsm_entry* entry);
int scene_fsm_handle_stopped(struct scene_fsm_entry* entry, int id_from,
                             struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_stopped(struct scene_fsm_entry* entry);  

#endif // SCENE_FSM_IMPL_H_

