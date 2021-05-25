#include <WiFi.h>
//#include <WebSocketsClient.h> // WebSocket Client Library for WebSocket
 
const char* ssid = "BT-KNA8K2";
const char* password = "ATcXAcvn7iayT3";
 
const uint16_t port = 8000;
const char * host = "192.168.1.78";
WiFiClient client; //Socket(TCP) connection over WiFi, this is not a WebSocket connection

//Connect to the WiFi network using a serial connection
//Must have client and server on same network rn, since no port forwarding
void setup()
{
 
  Serial.begin(115200);
 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }

 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  
  //On start up connect
  client.connect(host, port);
}

void SendCommand(const String & EnergyData, const String & VisionData = "Null", const String & DriveData = "Null"){
  client.print("Data : "+ EnergyData + " , " + VisionData + " , " + DriveData);
}

void loop()
{
    //Declare an object of wifi client, and connect to it, using the IP of the server and the port
    if(Serial.available() != 0) {// serial.available() returns the number of bytes waiting in the UART buffer
      String rx_byte = Serial.readStringUntil('\n');
      //char rx_byte = Serial.read();
      if(rx_byte == "s"){//Disconnects on s input
          Serial.println("Disconnecting...");
          //Free resources by stopping the serial connection
          client.stop();
      }else{
          Serial.println("Connected to server and sending message!");       
          //client.print(rx_byte);
          SendCommand(rx_byte);
      }
    
    }
    

  
}
