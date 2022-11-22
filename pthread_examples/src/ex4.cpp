// Pthread create and detach
 
#include <pthread.h>
#include <stdio.h>
 
void *threadRoutine(void *argumentPointer)
{
    pthread_t id = pthread_self();
 
    // TID를 반환하고 출력
    printf("thread ID (TID) :: %lu\n", id);
    
    while(1)
    {
        printf("a\n");
    }
 
    // 부모 스레드 부분에서 리턴값을 받기때문에 항상 리턴을 해준다.
    return NULL;
}
 
int main()
{
    pthread_t threadID;
    char tmp[10];
 
    // threadID로 TID를 받아오고, threadRoutine라는 함수 포인터로 스레드를 실행한다.
    printf(" == If you want Create Thread, write everything and enter == \n");
    scanf("%s",tmp);
 
    printf(" == Create Thread! ==\n");
    pthread_create(&threadID, NULL, threadRoutine, NULL);
 
    // threadID를 가진 thread를 detach한다.
    printf(" == If you want Detach, write everything and enter == \n");
    scanf("%s",tmp);
 
    pthread_detach(threadID);
 
    printf(" == Complete == \n");
 
    while(1){printf("b\n");}
    return 0;
}