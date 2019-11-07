
/* POSIX Header files */
#include <pthread.h>

#ifndef LIGHTSENSORTHREAD_H_
#define LIGHTSENSORTHREAD_H_


#ifdef __cplusplus
extern "C" {
#endif


extern pthread_t lightSensorThread_handler;
extern void *lightSensorThread(void *arg0);




#endif /* LIGHTSENSORTHREAD_H_ */
