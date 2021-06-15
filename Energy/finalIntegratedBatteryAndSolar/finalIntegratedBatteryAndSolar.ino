#include <Wire.h>
#include <INA219_WE.h>
#include <SPI.h>
#include <SD.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 10;
unsigned int rest_timer;
unsigned int loop_trigger;
unsigned int int_count = 0; // a variables to count the interrupts. Used for program debugging.
float u0i, u1i, delta_ui, e0i, e1i, e2i; // Internal values for the current controller
float ui_max = 1, ui_min = 0; //anti-windup limitation
float kpi = 0.02512, kii = 39.4, kdi = 0; // current pid.
float Ts = 0.001; //1 kHz control frequency.
float current_measure, current_ref = 0, error_amps, currentCount = 0, currentAvg = 0; // Current Control
float pwm_out;
//voltage measure
float V_Bat;
float cellOne;
float cellTwoThree;
float cellTwo;
float cellThree;
//SoC and SoH
float healthCounter = 0;
float SoC = 0;
float SoCOne = 0;
float SoCTwo = 0;
float SoCThree = 0;
float tempOne;
float mAh = 0;
float currentIntegration = 0;
float currentTemp = 0;
float dischargeTimer = 0;
//use for MPPT
float powerOut;
float powerPrev = 0;
float voltageCount;
float voltageAvg;
boolean input_switch;
int state_num=0,next_state;
String dataString;

void setup() {
  //Some General Setup Stuff

  Wire.begin(); // We need this for the i2c comms for the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  ina219.init(); // this initiates the current sensor
  Serial.begin(9600); // USB Communications


  //Check for the SD Card
  Serial.println("\nInitializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("* is a card inserted?");
    while (true) {} //It will stick here FOREVER if no SD is in on boot
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  if (SD.exists("BatCycle.csv")) { // Wipe the datalog when starting
    SD.remove("BatCycle.csv");
  }

  
  noInterrupts(); //disable all interrupts
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  //SMPS Pins
  pinMode(13, OUTPUT); // Using the LED on Pin D13 to indicate status
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  pinMode(6, OUTPUT); // This is the PWM Pin

  //LEDs on pin 7 and 8
  pinMode(7, OUTPUT); //red
  pinMode(8, OUTPUT); //green

  //Discharge pins
  pinMode(4, OUTPUT); //cellOne
  pinMode(9, OUTPUT); //cellTwo
  pinMode(3, OUTPUT); //cellThree


  //Analogue input, the respective cell voltages
  pinMode(A0, INPUT); //V_Bat
  pinMode(A1, INPUT); //cellTwoThree
  pinMode(A2, INPUT); //cellThree


  // TimerA0 initialization for 1kHz control-loop interrupt.
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm;

  // TimerB0 initialization for PWM output
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz

  interrupts();  //enable interrupts.
  analogWrite(6, 120); //just a default state to start with

}

void loop() {
  if (loop_trigger == 1){ // FAST LOOP (1kHZ)
      state_num = next_state; //state transition

      //determine Cell Voltages
      V_Bat = analogRead(A0)*3*4.096/1.03; //check the battery voltage (1.03 is a correction for measurement error, you need to check this works for you) (total battery voltage. Multiplied by 3 to account for potential divider)
      cellTwoThree = analogRead(A1)*2*4.096/1.03; //checks total voltage  of cells two and three. Multipled by two to account for potential divider
      cellOne = V_Bat - cellTwoThree; // Calculates difference to determine the first cell voltage
      cellThree = analogRead(A2)*4.096/1.03; // Voltage across cell three
      cellTwo = cellTwoThree - cellThree; // Calculates difference to determine the second cell voltage
      
      if ((cellOne > 3700 || cellTwo > 3700 || cellThree > 3700)) { //Checking for Error states (just the individual cell voltages)
          state_num = 6; //go directly to jail
          next_state = 6; // stay in jail
          digitalWrite(7,true); //turn on the red LED
          current_ref = 0; // no current
      }
      current_measure = (ina219.getCurrent_mA()); // sample the inductor current (via the sensor chip)

      currentCount = currentCount + current_measure; // add up the current every millisecond for the purpose of avergaging the current for coulomb counting      
      voltageCount = voltageCount + V_Bat; // use to average the voltage for later
      error_amps = (current_ref - current_measure) / 1000; //PID error calculation
      if (state_num != (1 && 2)) { // when not using the MPPT algorithim then duty cycle is then used to control the current based on current reference
        pwm_out = pidi(error_amps); //Perform the PID controller calculation
      }
      pwm_out = saturation(pwm_out, 0.99, 0.01); //duty_cycle saturation
      analogWrite(6, (int)(255 - pwm_out * 255)); // write it out (inverting for the Buck here)
      int_count++; //count how many interrupts since this was last reset to zero
      loop_trigger = 0; //reset the trigger and move on with life
  }
  
  if (int_count == 1000) { // SLOW LOOP (1Hz)
    input_switch = digitalRead(2); //get the OL/CL switch status
    currentAvg = currentCount/1000; //average current for smoother SoC
    voltageAvg = voltageCount/1000; //smooth out the voltage reading
    powerOut = voltageAvg * currentAvg; //determine the power output at any given time
    
    switch (state_num) { // STATE MACHINE 
      case 0:{ // Start state (no current, no LEDs)
        current_ref = 0;
        if (input_switch == 1) { // if switch, move to charge
          next_state = 1;
          pwm_out = 0.5; //set inital duty cycle for MPPT
          digitalWrite(8,true);
        } else { // otherwise stay put
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      case 1:{ // Charge state using MPPT algoritim (increase duty)
        
        if ((cellOne < 3600 && cellTwo < 3600 && cellThree < 3600)) { // if not charged, stay put. All cells must not be charged above 3600 in order to prevent overcharge
          
          digitalWrite(8,true); 
          
          //the following is basic code for passive balancing (check balancing before changing duty cycle)
          //cell one
          if (cellOne - 20 > (cellTwo || cellThree)) {
            digitalWrite(4, true); // if cell one is 20mV greater than cell two or cell three then turn on the discharge circuit to slow down its rate of charge
          } else {
            digitalWrite(4, false);
          }
         
          //cell two
           if (cellTwo - 20 > (cellThree || cellOne)) {
            digitalWrite(9, true); // if cell two is 20mV greater than cell three or cell one then turn on the discharge circuit to slow down its rate of charge
          } else {
            digitalWrite(9, false);
          }
          
           //cell three
           if (cellThree - 20 > (cellTwo || cellOne)) {
            digitalWrite(3, true); // if cell three is 20mV greater than cell two or cell one then turn on the discharge circuit to slow down its rate of charge
          } else {
            digitalWrite(3, false);
          }

          //now begin first half of the MPPT algorithim

          if (powerOut >= powerPrev)  { //if the current power output is greater than previous value then keep increasing the duty cycle
          pwm_out = pwm_out + 0.01;
          powerPrev = powerOut; //set new previous power value
          next_state = 1; //stay at current state
         } else {
          pwm_out = pwm_out - 0.01;
          powerPrev = powerOut; //set new previous power value
          next_state = 2;
         }
         
                  
        } else { // otherwise go to charge rest
          next_state = 3;
          digitalWrite(8,false);
          healthCounter++;
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }

      case 2 : { // charge state using MPPT algorithim (decrease the duty cycle
      if ((cellOne < 3600 && cellTwo < 3600 && cellThree < 3600)) { // if not charged, stay put. All cells must not be charged in order to prevent overcharge
      
          digitalWrite(8,true); 
          
          //the following is basic code for passive balancing (check balancing before changing duty cycle)
          //cell one
          if (cellOne - 20 > (cellTwo || cellThree)) {
            digitalWrite(4, true); // if cell one is 20mV greater than cell two or cell three then turn on the discharge circuit to slow down its rate of charge
          } else {
            digitalWrite(4, false);
          }
         
          //cell two
           if (cellTwo - 20 > (cellThree || cellOne)) {
            digitalWrite(9, true); // if cell two is 20mV greater than cell three or cell one then turn on the discharge circuit to slow down its rate of charge
          } else {
            digitalWrite(9, false);
          }
          
           //cell three
           if (cellThree - 20 > (cellTwo || cellOne)) {
            digitalWrite(3, true); // if cell three is 20mV greater than cell two or cell one then turn on the discharge circuit to slow down its rate of charge
          } else {
            digitalWrite(3, false);
          }

          //now begin second half of the MPPT algorithim

          if (powerOut >= powerPrev)  { //if the current power output is greater than previous value then keep decreasing the duty cycle
          pwm_out = pwm_out - 0.01;
          powerPrev = powerOut; //set new previous power value
          next_state = 2; //stay at current state
         } else {
          pwm_out = pwm_out + 0.01;
          powerPrev = powerOut; //set new previous power value
          next_state = 1;
         }
         
                  
        } else { // otherwise go to charge rest
          next_state = 3;
          digitalWrite(8,false);
          healthCounter++; //increase the cycle count
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }  
      
      case 3:{ // Charge Rest, green LED is off and no current
        current_ref = 0;
        if (rest_timer < 1800) { // Stay here if timer < 30mins to let OCV settle, meaning we now know the cells are at 100% SoC
          next_state = 3;
          digitalWrite(8,false);
          rest_timer++;
        } else { // Or move to discharge (and reset the timer)
          next_state = 4;
          digitalWrite(8,false);
          rest_timer = 0;
        }
        if(input_switch == 0){ // UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;        
      }
      
      case 4:{ //Discharge state (-250mA and no LEDs) (would take place on the drive arduino/SMPS)
         current_ref = -250;
         if ((cellOne > 2400 && cellTwo < 2400 && cellThree > 2400)) { // While not at minimum voltage, stay here
           next_state = 4;
           digitalWrite(8,false);

           //the following is basic code for passive balancing
          //cell one
          if (cellOne - 20 > (cellTwo || cellThree)) {
            digitalWrite(4, true); // if cell one is 20mV greater than cell two or cell three then turn on the discharge circuit to speed up its rate of discharge
          } else {
            digitalWrite(4, false);
          }
         
          //cell two
           if (cellTwo - 20 > (cellThree || cellOne)) {
            digitalWrite(9, true); // if cell two is 20mV greater than cell three or cell one then turn on the discharge circuit to speed up its rate of discharge
          } else {
            digitalWrite(9, false);
          }
          
           //cell three
           if (cellThree - 20 > (cellTwo || cellOne)) {
            digitalWrite(3, true); // if cell three is 20mV greater than cell two or cell one then turn on the discharge circuit to speed up its rate of discharge
          } else {
            digitalWrite(3, false);
          }

              // Compute the discharge state of charge using coulomb counting
        dischargeTimer++; //keep track of the time the discharge has been taking
        currentIntegration = (currentIntegration + currentAvg); //integrate the current to achieve charge in coulombs using reimann sum. Interval is only 1sec
        mAh = currentIntegration/3600; //convert charge in coulombs to mAh instead

        // the 100 is the SoC at full charge, in future versions this would be recalibrated at regular intervals using each cells respective SoC-OCV discharge curves. 
        //This would improve accuracy by a subastanial amount
        //upon further thought, although recalibrating using SoC-OCV curves seems preferrable it would actually require 
        //the rover to turn off for 30 minutes at regular intervals whilst the cell settles to its open circuit value which is less than ideal in practice
        SoCOne = 100 - (mAh/480); // 480mAh is the calculated capacity of cell one
        SoCTwo = 100 - (mAh/491); // 491mAh is the calculated capacity of cell two
        SoCThree = 100 - (mAh/486); // 486mAh is the calculated capcaity of cell three
        
        //states the battery's state of charge is equal to whichever cell has the lowest state of charge
        tempOne = min(SoCOne, SoCTwo); 
        SoC = min(tempOne, SoCThree);

          //the following piece of code is pseudo-code to work out the range remaining in the rover. Due to energy not being real, this can't be implemented
        //timeLeft = mAh/currentAvg; // divide the current capacity of the cells by the current being drawn by the rover's motors to determine how much time left until cells empty
        //timeLeft = timeLeft*60*60; //convert to seconds
        //range = timeLeft * motorSpeed; //simple s=vt calculation. speed determined on drive side by averaging out the number of rotations over a set timeframe 
          //and extrapolating that to rpm and getting speed by mulitiplying by wheel circumference
        
          
         } else { // If we reach full discharged, move to rest
           next_state = 5;
           digitalWrite(8,false);
           dischargeTimer = 0; //reset the timer for future integration cycles
           currentIntegration = 0; //reset the integrated charge value for future cycles
         }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        
        break;
      }
      
      case 5:{ // Discharge rest, no LEDs no current
        current_ref = 0;
        if (rest_timer < 1800) { // Rest here for 30mins to let OCV settle to 0% SoC
          next_state = 5;
          digitalWrite(8,false);
          rest_timer++;
        } else { // When thats done, move back to charging (and light the green LED)
          next_state = 1;
          digitalWrite(8,true);
          rest_timer = 0;
        }
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(8,false);
        }
        break;
      }
      
      case 6: { // ERROR state RED led and no current
        current_ref = 0;
        next_state = 6; // Always stay here
        digitalWrite(7,true);
        digitalWrite(8,false);
        if(input_switch == 0){ //UNLESS the switch = 0, then go back to start
          next_state = 0;
          digitalWrite(7,false);
        }
        break;
      }

      default :{ // Should not end up here ....
        Serial.println("Boop");
        current_ref = 0;
        next_state = 6; // So if we are here, we go to error
        digitalWrite(7,true);
      }
      
    
    
    dataString = String(state_num) + "," + String(V_Bat) + "," + String(current_ref) + "," + String(current_measure) + "," + String(powerOut); //build a datastring for the CSV file
    Serial.println(dataString); // send it to serial as well in case a computer is connected
    File dataFile = SD.open("BatCycle.csv", FILE_WRITE); // open our CSV file
    if (dataFile){ //If we succeeded (usually this fails if the SD card is out)
      dataFile.println(dataString); // print the data
    } else {
      Serial.println("File not open"); //otherwise print an error
    }
    dataFile.close(); // close the file
    int_count = 0; // reset the interrupt count so we dont come back here for 1000ms
    currentCount = 0; //reset count for next average calculation
    voltageCount = 0; //reset for next cycle
  }
}
}

// Timer A CMP1 interrupt. Every 1000us the program enters this interrupt. This is the fast 1kHz loop
ISR(TCA0_CMP1_vect) {
  loop_trigger = 1; //trigger the loop when we are back in normal flow
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
}

float saturation( float sat_input, float uplim, float lowlim) { // Saturation function
  if (sat_input > uplim) sat_input = uplim;
  else if (sat_input < lowlim ) sat_input = lowlim;
  else;
  return sat_input;
}

float pidi(float pid_input) { // discrete PID function
  float e_integration;
  e0i = pid_input;
  e_integration = e0i;

  //anti-windup
  if (u1i >= ui_max) {
    e_integration = 0;
  } else if (u1i <= ui_min) {
    e_integration = 0;
  }

  delta_ui = kpi * (e0i - e1i) + kii * Ts * e_integration + kdi / Ts * (e0i - 2 * e1i + e2i); //incremental PID programming avoids integrations.
  u0i = u1i + delta_ui;  //this time's control output

  //output limitation
  saturation(u0i, ui_max, ui_min);

  u1i = u0i; //update last time's control output
  e2i = e1i; //update last last time's error
  e1i = e0i; // update last time's error
  return u0i;
}
