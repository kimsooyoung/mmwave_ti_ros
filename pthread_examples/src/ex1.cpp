// 기본 사용 법, pthread_create/ pthread_join/ pthread_self

#include <pthread.h>
#include <iostream>
 
void *threadRoutine(void *argumentPointer)
{
    pthread_t id = pthread_self();
 
    // TID를 반환하고 출력
    std::cout << "thread ID (TID) : " << id << std::endl; 
    
    // 부모 스레드 부분에서 리턴값을 받기때문에 항상 리턴을 해준다.
    return NULL;
}
 
int main()
{
    pthread_t threadID;
    
    // threadID로 TID를 받아오고, threadRoutine라는 함수 포인터로 스레드를 실행한다.
    std::cout << "Create Thread!" << std::endl;

    pthread_create(&threadID, NULL, threadRoutine, NULL);
 
    // threadID를 가진 thread가 실행되는 동안 기다린다.
    std::cout << "Main Thread is now waiting for Child Thread!" << std::endl;
 
    pthread_join(threadID, NULL);
 
    std::cout << "Main Thread finish waitng Child Thread!" << std::endl;

    return 0;
}