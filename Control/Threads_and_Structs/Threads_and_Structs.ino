#include <pthread.h>
#include <WiFi.h>
#include <tuple>
#include <stdlib.h>
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

typedef struct{
  String value[3] = {"2nd","3rd","4th"};
  } Data_struct;
  
void *printThreadId(void *threadid) {
   //Serial.println(*actual_data);
   Data_struct *True_data = (Data_struct *)threadid;
   //Data_struct True_data1 = (Data_struct)threadid;
   Serial.println("-----Attempting new-----");
   Serial.println( *(&True_data->value[1]) );
   String *trial = (String *)threadid->value;
   //String *ptr = trial->value;
   Serial.println(*trial);
   Serial.println("-----Working-----");
   //Serial.println(Data->value[1]);
   //Serial.println(*ptr);
   //Serial.println(*(ptr+1));
   delay(10000);
   return 0;
}
 
void setup() {
 
   Serial.begin(115200);
   pthread_t threads;
   int returnValue;
   Data_struct Data;// = malloc(sizeof *Data);
   Serial.println("-----Setup-----");
   Serial.println(Data.value[1]);
 
   //for( int i = 0; i< 1; i++ ) {
 
      returnValue = pthread_create(&threads, NULL, printThreadId, &Data);
 
      if (returnValue) {
         //free(Data);//Only needed if we assign a malloc (manually allocate so must manually de-allocate memory)
         Serial.println("An error has occurred");
      }
   //}
 
}
 
void loop() {}                                                                                                                                          
