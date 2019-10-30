
/* POSIX Header files */
#include <pthread.h>

#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#ifdef __cplusplus
extern "C" {
#endif

extern pthread_t heartBeatThread_handler;
extern void *heartBeatThread(void *arg0);


#endif /* HEARTBEAT_H_ */
