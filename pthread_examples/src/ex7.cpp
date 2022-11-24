#include <stdio.h>
#include <pthread.h>
 
// pp ;: pingpong
pthread_mutex_t ppmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ppcond = PTHREAD_COND_INITIALIZER;
int flag = 0;
 
void *ping(void *argumentPointer)
{
    int i = 0;
 
    printf("ping 먼저 \n");
    while(i <= 100)
    {
            pthread_mutex_lock(&ppmutex);
 
            printf("ping :: %d\n",i);
            i++;
            flag = 1;
 
            pthread_cond_signal(&ppcond);
    
            pthread_cond_wait(&ppcond,&ppmutex);
 
            pthread_mutex_unlock(&ppmutex);
        
    }
    return NULL;
}
 
 
void *pong(void *argumentPointer)
{
    int i = 0;
 
    printf("pong 먼저 \n");
    while(i <= 100)
    {
        pthread_mutex_lock(&ppmutex);
        
        while(flag == 0)
            pthread_cond_wait(&ppcond, &ppmutex);
 
        flag = 0;
        printf("pong :: %d\n",i);
        i++;
 
        pthread_cond_signal(&ppcond);
 
        pthread_mutex_unlock(&ppmutex);
 
    }
    return NULL;
}
 
 
int main()
{
    pthread_t threadID1, threadID2;
 
    // Create < 뮤텍스 이용 >
    pthread_create(&threadID1, NULL, ping, NULL);
    pthread_create(&threadID2, NULL, pong, NULL);
 
    // Join < 뮤텍스 이용 >
    pthread_join(threadID1, NULL);
    pthread_join(threadID2, NULL);
 
    printf("Ping Pong finish !! \n"); 
    return 0;
}