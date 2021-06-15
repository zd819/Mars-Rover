#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

HardwareSerial Vision(1); // Setup Serial1 for Uart to Vision
HardwareSerial Drive(2); // Setup Serial2 for Uart to Drive

WebSocketsClient webSocket;

//Wifi variables
unsigned long myTime;
const char *ssid = "VIVACOM_FiberNet";
const char *password = "3d55faf0";
unsigned long messageInterval = 50;//Speed imbetween UART transmissions (kindof a clock)
bool connected = false;

//Char data from Submodules
String FromCommand = "";
String FromVision = "";
String FromDrive = "";

//Formatting data variables to drive
String toDrive= "";
String Speed = "";
String Offset = "";
String CoordXY = "";
int x = 0;

////Formatting data variables from vision
String distance1 = "";
String angle1 = "";
String color1 = "";
String distance2 = "";
String angle2 = "";
String color2 = "";
String distance3 = "";
String angle3 = "";
String color3 = "";
int y = 0;

//Formatting data variables to Command
String battery= "";
String range= "";
String r_x = "";
String r_y= "";
String angle= "";
String obs1_x= "";
String obs1_y= "";
String obs2_x = "";
String obs2_y = "";
String obs3_x = "";
String obs3_y = "";
String toCommand = "";
int z = 0;

//Cache mechanism variables
bool newData= false;



#define DEBUG_SERIAL Serial

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
const uint8_t* src = (const uint8_t*) mem;
DEBUG_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
for(uint32_t i = 0; i < len; i++) {
if(i % cols == 0) {
DEBUG_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src,
i);
}
DEBUG_SERIAL.printf("%02X ", *src);
src++;
}
DEBUG_SERIAL.printf("\n");
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
switch(type) {
case WStype_DISCONNECTED://Disconnect event from webserver triggered by websocket
DEBUG_SERIAL.printf("[WSc] Disconnected!\n");
connected = false;
break;
case WStype_CONNECTED: {//Connection event to Websever triggered by websocket
DEBUG_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
connected = true;
// send message to server when Connected
DEBUG_SERIAL.println("[WSc] SENT: Connected");
webSocket.sendTXT("Connected");
}
break;
case WStype_TEXT:
DEBUG_SERIAL.printf("[WSc] RESPONSE: %s\n", payload);
if( (char *)payload != "Connected"){
newData = true;
FromCommand = (char *) payload;
}
break;
case WStype_BIN:
DEBUG_SERIAL.printf("[WSc] get binary length: %u\n", length);
hexdump(payload, length);
break;
case WStype_PING:
// pong will be send automatically (on connection)
DEBUG_SERIAL.printf("[WSc] get ping\n");
break;
case WStype_PONG:
// answer to a ping we send
DEBUG_SERIAL.printf("[WSc] get pong\n");
break;
case WStype_ERROR:
case WStype_FRAGMENT_TEXT_START:
case WStype_FRAGMENT_BIN_START:
case WStype_FRAGMENT:
case WStype_FRAGMENT_FIN:
break;
}
}


void setup() {
DEBUG_SERIAL.begin(115200);
//Speed.reserve(25); Offset.reserve(25); CoordXY.reserve(25);
//FromCommand.reserve(25); FromVision.reserve(25); FromDrive.reserve(25); toDrive.reserve(25); toCommand.reserve(25);
DEBUG_SERIAL.setDebugOutput(true);
Drive.begin(115200,SERIAL_8N1,16,17);//Setup serial uart with (Baud rate, protocol, Rx, Tx)
Vision.begin(115200,SERIAL_8N1,19,21);//Setup serial uart with (Baud rate, protocol, Rx, Tx)

DEBUG_SERIAL.print ("Serial Connection Initiated between Drive and Vision");
DEBUG_SERIAL.println();
DEBUG_SERIAL.println();
DEBUG_SERIAL.println();

for(uint8_t t = 4; t > 0; t--) {
DEBUG_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
DEBUG_SERIAL.flush();
delay(1000);
}
DEBUG_SERIAL.println("For loop done");

WiFi.begin(ssid, password);
DEBUG_SERIAL.println("Wifi connected");
while ( WiFi.status() != WL_CONNECTED ) {
delay ( 500 );
DEBUG_SERIAL.print ( "." );
}
DEBUG_SERIAL.print("Local IP: "); DEBUG_SERIAL.println(WiFi.localIP());
// server address, port and URL
webSocket.begin("192.168.1.4", 8080, "/"); //Establish WebSocket connection to server using Handshake
// event handler
webSocket.onEvent(webSocketEvent);
DEBUG_SERIAL.println("WebSocket initiated");
}

unsigned long lastUpdate = millis();

void loop() { //Can receieve at each loop, but only send at each cycle designated above
webSocket.loop();
webSocket.onEvent(webSocketEvent);

//Read from drive, read from Vision (constantly) but only assign variables and send at acute times
// then recieve from command, then send data to Drive (priority) and after send to command
while(Drive.available()){
uint8_t byteDrive = Drive.read();
FromDrive += (char)byteDrive;
//Variable processing
}

while(Vision.available()){
uint8_t byteVision = Vision.read();
FromVision += (char)byteVision;
//Variable processing
}

//Process data from ENERGY


//Formatting data from Drive and Vision
if (connected && lastUpdate+messageInterval<millis()){
  //Vision
  if(FromVision!=""){
    DEBUG_SERIAL.print("Receiving from vision:");
    DEBUG_SERIAL.println(FromVision.c_str());
    FromVision = FromVision.substring(2);
    y = FromVision.indexOf(']');
    distance1 = FromVision.substring(0,y);
    FromVision = FromVision.substring(y+2);
    y = FromVision.indexOf(']');
    angle1 = FromVision.substring(0,y);
    FromVision = FromVision.substring(y+2);
    y = FromVision.indexOf(']');
    color1= FromVision.substring(0,y); 
  }
  //Drive [0,0][0][0,0]/
  DEBUG_SERIAL.print("Data from drive: ");
  DEBUG_SERIAL.println(FromDrive);
  FromDrive = FromDrive.substring(2);
  x = FromDrive.indexOf(',');
  r_x= FromDrive.substring(0,x);
  FromDrive = FromDrive.substring(x+1);
  x = FromDrive.indexOf(']');
  r_y = FromDrive.substring(0,x);
  FromDrive = FromDrive.substring(x+2);
  x = FromDrive.indexOf(']');
  angle = FromDrive.substring(0,x);
  FromDrive = FromDrive.substring(x+2);
  x = FromDrive.indexOf(']');
  color1 = FromDrive.substring(0,x);
  FromDrive = FromDrive.substring(x+2);
  x = FromDrive.indexOf(',');
  obs1_x = FromDrive.substring(0,x);
  FromDrive = FromDrive.substring(x+1);
  x = FromDrive.indexOf(']');
  obs1_y = FromDrive.substring(0,x);
  FromDrive = FromDrive.substring(x+1);
  x= FromDrive.indexOf(']');

if( (FromCommand.c_str() != "" )&& (newData == true) ){
// If new command data, retireve data components, else keep last ones (no new assignments) if needed to send data to drive
FromCommand = FromCommand.substring(2);
z = FromCommand.indexOf(',');
Speed = FromCommand.substring(0,z);
FromCommand = FromCommand.substring(z+1);
z = FromCommand.indexOf(',');
Offset = FromCommand.substring(0,z);
FromCommand = FromCommand.substring(z+1);
z = FromCommand.indexOf('/');
CoordXY = FromCommand.substring(0,z-1);
}
toDrive = "/[" + Speed + "][" + Offset + "][" + CoordXY + "][" + color1 + "][" + distance1 + "][" + angle1+ "]/";
Drive.write(toDrive.c_str()); //Need to send a null terminating string for UART transmission
newData = false;
DEBUG_SERIAL.print("Sending to Drive");
DEBUG_SERIAL.println(toDrive.c_str());
FromDrive = ""; //Not sure if this is needed, but it works
 
FromVision = ""; //Not sure if this is needed, but it works
toCommand = "/,80,100," + r_x + "," + r_y + "," + angle + "," + obs1_x + "," + obs1_y + "," + color1 +",/";
DEBUG_SERIAL.print("Data to command: ");
DEBUG_SERIAL.println(toCommand);
webSocket.sendTXT(toCommand);

lastUpdate = millis();
}
}
