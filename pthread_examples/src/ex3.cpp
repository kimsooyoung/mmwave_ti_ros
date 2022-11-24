// 스레드 함수에서의 return

#include <pthread.h>
#include <stdio.h>
 
void *threadRoutine(void *argumentPointer)
{
    long argument;
    argument = *((long *)argumentPointer);
 
    // 부모 스레드 부분에서 리턴값을 받기때문에 항상 리턴을 해준다.
    return (void*)(argument + 10);
}
 
int main()
{
    pthread_t threadID;
    
    // threadID로 TID를 받아오고, threadRoutine라는 함수 포인터로 스레드를 실행한다.
    printf("Create Thread!\n");
 
    // 알규먼트를 생성하여 스레드에 보내준다.
    long argument = 1;
    int ret;
    void *value;
 
    ret = pthread_create(&threadID, NULL, threadRoutine, (void*)&argument);
 
    // threadID를 가진 thread가 실행되는 동안 기다린다.
    printf("Main Thread is now waiting for Child Thread!\n");
 
    pthread_join(threadID, &value);
 
    printf("Main Thread finish waitng Child Thread!\n");
 
    printf(" == Return Value (0 : success , others : fail) :: %d\n",ret);
    printf(" == Receive Value :: %ld\n", (long)value); 
 
    return 0;
}