/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/

//Define pins for the Drive arduino nano every
#define RXD1 Insert
#define TXD1 Insert


void setup() 
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);//Serial0(programming serial) is automatically setup
  
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);//Setup the serial1 port for communication to control
  
}

void loop() { //Where main Drive code is
  
  while(Serial1.available()) {//Then can use UART serial2 data from serial2.read
    //if(some function is triggered){
    //  Serial1.write(Data);
    char data = Serial1.read() //Read from UART serial port
    Serial1.write() //Send through too UART serial port
    //}
    //if(Serial1.read() == 's'){//'s' to indicate rover to stop, from vision
    ///Then stop
    //}
             
  }
}
