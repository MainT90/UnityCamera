
#ifndef SCENE_FSM_IMPL_H_
#define SCENE_FSM_IMPL_H_

// scene的各种状态处理实现

#include "scene_fsm.h"

// scene尚未注册状态
int scene_fsm_enter_unregister(struct scene_fsm_entry* entry);
int scene_fsm_handle_unregister(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_unregister(struct scene_fsm_entry* entry);

// scene正在执行注册状态
int scene_fsm_enter_registering(struct scene_fsm_entry* entry);
int scene_fsm_handle_registering(struct scene_fsm_entry* entry, int id_from,
                                 struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_registering(struct scene_fsm_entry* entry);

// scene完成注册状态
int scene_fsm_enter_registered(struct scene_fsm_entry* entry);
int scene_fsm_handle_registered(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_registered(struct scene_fsm_entry* entry);

// scene同步数据中
int scene_fsm_enter_datasyning(struct scene_fsm_entry* entry);
int scene_fsm_handle_datasyning(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_datasyning(struct scene_fsm_entry* entry);

// scene同步数据完成
int scene_fsm_enter_datasyned(struct scene_fsm_entry* entry);
int scene_fsm_handle_datasyned(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_datasyned(struct scene_fsm_entry* entry);

// scene准备服务状态
int scene_fsm_enter_prepareservice(struct scene_fsm_entry* entry);
int scene_fsm_handle_prepareservice(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_prepareservice(struct scene_fsm_entry* entry);
// scene正常服务状态
int scene_fsm_enter_inservice(struct scene_fsm_entry* entry);
int scene_fsm_handle_inservice(struct scene_fsm_entry* entry, int id_from,
                               struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_inservice(struct scene_fsm_entry* entry);

// scene无法得到world支持状态，准备存盘退出
int scene_fsm_enter_standalone(struct scene_fsm_entry* entry);
int scene_fsm_handle_standalone(struct scene_fsm_entry* entry, int id_from,
                                struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_standalone(struct scene_fsm_entry* entry);

// scene关服状态，踢人、存盘、退出
int scene_fsm_enter_stopping(struct scene_fsm_entry* entry);
int scene_fsm_handle_stopping(struct scene_fsm_entry* entry, int id_from,
                              struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_stopping(struct scene_fsm_entry* entry);

// scene优雅关闭成功
int scene_fsm_enter_stopped(struct scene_fsm_entry* entry);
int scene_fsm_handle_stopped(struct scene_fsm_entry* entry, int id_from,
                             struct scene_fsm_xypkg* xypkg);
int scene_fsm_leave_stopped(struct scene_fsm_entry* entry);  

#endif // SCENE_FSM_IMPL_H_

