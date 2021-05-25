#include <pthread.h>
#include <WiFi.h>
#include <tuple>
#include<iostream>
 
const char* ssid = "BT-KNA8K2";
const char* password = "ATcXAcvn7iayT3";
 
const uint16_t port = 8090;
const char * host = "192.168.1.78";

//Connect to the WiFi network using a serial connection
//Must have client and server on same network rn, since no port forwarding


struct ConnectionData{
  const char* ssid = "BT-KNA8K2";
  const char* password = "ATcXAcvn7iayT3";
  const uint16_t port = 8090;
  const char * host = "192.168.1.78";
  };

struct Trial{
  int 1st = 1;
  String *value[3] = {"2nd","3rd","4th"};
  };
void *printThreadId(void *threadid) {
   Serial.println("Test1 :");
   String &ptr = (String *)threadid;
   Serial.print( *( ( (String *)threadid)+1) );
   Serial.print( (&ptr+1));
   Serial.println("Test2 : ");
   String passed = *(String *)threadid;
   Serial.println(passed[1]);
   Serial.println((passed+1)[1]);
   Serial.print(" created");
   delay(10000);
}
 
void setup() {
 
   Serial.begin(115200);
   pthread_t threads[4];
   int returnValue;
   String value1[][10] = {"1sedf","2sedf","ssdf"};
   String value2[3] = {"1st","2nd","3rd"};
   String *ptr =  value2;
   Serial.println("-----Setup-----");
   Serial.println(*(ptr+1));
 
   //for( int i = 0; i< 1; i++ ) {
 
      returnValue = pthread_create(&threads[1], NULL, printThreadId, (void *)ptr);
 
      if (returnValue) {
        
         Serial.println("An error has occurred");
      }
   //}
 
}
 
void loop() {}                                                                                                                                          
