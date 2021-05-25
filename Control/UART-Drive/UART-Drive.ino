/*
 * There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
 * 
 * U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
 * U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
 * U2UXD is unused and can be used for your projects.
 * 
*/
#define RXD1 40//Not sure about second set of uart serial pins
#define TXD1 41

#define RXD2 16
#define TXD2 17

void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);//Serial0(programming serial) is automatically setup
  //Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);//Setup the serial2 port for vision communication
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);//Setup the serial1 port for drive communication
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
}

void loop() { //Choose Serial1 or Serial2 as required
  while (Serial2.available()) {//Then can use UART serial2 data from serial2.read
    char rx_byte = Serial2.read();
    Serial.print(rx_byte);//Data from drive, which will be piped through
    //Serial1.print(char(Serial2.read()));//Pipe from vision to command
    //client.print(rx_byte); //Continously pipe drive output to command, if nothing to be transmitted
    //Esp will forward through a null character to command, to show no change

    //Format for the data to send to vision needs to be sorted
       
  }
}
