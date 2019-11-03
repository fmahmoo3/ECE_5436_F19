/* POSIX Header files */
#include <pthread.h>

#ifndef UARTTHREAD_H_
#define UARTTHREAD_H_


#ifdef __cplusplus
extern "C" {
#endif


extern pthread_t uartThread_handler;
extern void *uartThread(void *arg0);


#endif /* UARTTHREAD_H_ */
