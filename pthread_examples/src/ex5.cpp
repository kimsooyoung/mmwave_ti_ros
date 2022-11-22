// mutex 

#include <pthread.h>
#include <stdio.h>
 
pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;
 
int sum = 0;
int sum1 = 0;
 
void *threadRoutine(void *argumentPointer)
{
    int i;

    printf("flag");
 
    // 뮤텍스 락을 거는 과정
    pthread_mutex_lock(&counter_mutex);
 
    // 이 부분이 Critical Section에 해당한다.
    for(i = 0; i < 1000000; i ++)
        sum++;
 
    // 뮤텍스 락을 푸는 과정
    pthread_mutex_unlock(&counter_mutex);
    
    return NULL;
}
 
void *threadRoutine1(void *argumentPointer)
{
    int i;
 
    // 이 부분이 Critical Section에 해당한다.
    for(i = 0; i < 1000000; i ++)
        sum1++;
 
    return NULL;
}
 
int main()
{
    pthread_t threadID1, threadID2;
    pthread_t threadID3, threadID4;
 
    // Create < 뮤텍스 이용 >
    pthread_create(&threadID1, NULL, threadRoutine, NULL);
    pthread_create(&threadID2, NULL, threadRoutine, NULL);
 
    // Create < 뮤텍스 이용 x >
    pthread_create(&threadID3, NULL, threadRoutine1, NULL);
    pthread_create(&threadID4, NULL, threadRoutine1, NULL);

    // while(1){
    //     continue;
    // }
 
    // Join < 뮤텍스 이용 >
    pthread_join(threadID1, NULL);
    pthread_join(threadID2, NULL);
 
    // Join < 뮤텍스 이용 x >
    pthread_join(threadID3, NULL);
    pthread_join(threadID4, NULL);
 
    printf("뮤텍스를 이용한 결과 합 :: %d\n",sum);
    printf("뮤텍스를 이용하지 않은 결과 합 :: %d\n",sum1);
 
    return 0;
}