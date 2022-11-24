// thread에 변수를 전달하는 법

#include <pthread.h>
#include <stdio.h>
 
void *threadRoutine(void *argumentPointer)
{
    char *argument = (char *)argumentPointer;
 
    printf("%s\n", argument );
    // 부모 스레드 부분에서 리턴값을 받기때문에 항상 리턴을 해준다.
    return NULL;
}
 
int main()
{
    pthread_t threadID;
    
    // threadID로 TID를 받아오고, threadRoutine라는 함수 포인터로 스레드를 실행한다.
    printf("Create Thread!\n");
 
    // 알규먼트를 생성하여 스레드에 보내준다.
    char argument[10] = "hello";
 
    pthread_create(&threadID, NULL, threadRoutine, (void*)argument);
 
    // threadID를 가진 thread가 실행되는 동안 기다린다.
    printf("Main Thread is now waiting for Child Thread!\n");
 
    pthread_join(threadID, NULL);
 
    printf("Main Thread finish waitng Child Thread!\n");
 
    return 0;
}