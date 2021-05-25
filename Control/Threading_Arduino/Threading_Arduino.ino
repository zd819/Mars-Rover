#include <pthread.h>
 
void *printThreadId(void *threadid) {
   Serial.println("Thread ");
   Serial.print((int)threadid); 
   Serial.print(" created");
   delay(10000);
}
 
void setup() {
 
   Serial.begin(115200);
 
   pthread_t threads[8];
   int returnValue;
   int returnValue1;
 
   for( int i = 0; i< 4; i++ ) {
 
      returnValue = pthread_create(&threads[i], NULL, printThreadId, (void *)i);
      returnValue1 = pthread_create(&threads[i+1], NULL, printThreadId, (void *)(i+1));
 
      if (returnValue|returnValue1) {
        
         Serial.println("An error has occurred");
      }
   }
 
}
 
void loop() {}                                                                                                                                          
