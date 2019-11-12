
/* POSIX Header files */
#include <pthread.h>

#ifndef PRINTTHREAD_H_
#define PRINTTHREAD_H_


#ifdef __cplusplus
extern "C" {
#endif


extern pthread_t printThread_handler;
extern void *printThread(void *arg0);


#endif /* PRINTTHREAD_H_ */
