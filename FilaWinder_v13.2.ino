//Filament Winder Original by Ian Johnson
//Version_13.2  by Erich Reich 14.7.2024 
//www.erichreich.ch
//with Arduino IDE 2.3.2 , for Arduino-Nano (and Arduino-Nano-Every)



//Digital Pins
int toggle = 2;         //Auto / Manual Spool Switch
int guide_minPin = 3;   //Button for setting the Min Guide limits
int setPin = 4;         //Button for setting (Sensor Cal) + ServoPosition
int MOTOR_PIN = 5;      //PWM pin for spooler motor
int guidePin = 6;       //ServoGuide 
int hall_Pin = 7;       //Hall sensor A
int guide_maxPin = 8;   //Button for setting the Max Guide limit

//Analog Pins
unsigned int sensor_1 = A0;  //Bottom filament sensor
unsigned int sensor_2 = A1;  //Middle under filament sensor
unsigned int sensor_3 = A2;  //Middle upper filament sensor
unsigned int sensor_4 = A3;  //Top filament sensor
unsigned int poti_Pin = A6;  //Potentiometer


// Photoresistors
unsigned int Line_Pos = 0;                           // variable for filamentposition


//Variables for moving the guide-servo:
//Servo type MG-995, pulse: -90°/544us - +90°/2400us, 50Hz=20ms(20000us), angle-range 180° 
//Servo type BMS-660, pulse: -60°/900us - +60°/2100us, 125Hz=8ms(8000us), angle-range 120° (is not better type)
//Servo type REELY RE-6802575 STANDARD-SERVO S-5252-0, pulse: -60°/900us - +60°/2100us, 50Hz=20ms(20000us), angle-range 120°

//write values here in 0.00 for calculating with other floating variables!
//you can ad other servos, then changing the pulsevalues in microseconds here from servodatasheet:

const int MIN_PULSE_WIDTH = 900.00;        // the shortest pulse sent to a servo (pulse at min.degrees, by -90 or -60 degrees) (-90 degrees = 544us, -60 degrees = 900us)
const int MAX_PULSE_WIDTH = 2100.00;       // the longest pulse sent to a servo (pulse at max.degrees, by +90 or +60 degrees) (+90 degrees = 2400us, +60 degrees = 2100us)
const int DEFAULT_PULSE_WIDTH = 1500.00;   // default pulse width when servo is attached (pulse at 90 (in servodatasheet 0) degrees)
const int REFRESH_INTERVAL = 20000.00;     // servoimpulse in microseconds ((1 second) 1'000'000 microseconds / Hz servofrequency = REFRESH_INTERVAL)
const int SERVO_ANGLE_RANGE = 120.00;      // servo angle range 180 (-90 to +90) or 120 (-60 to +60) degrees


float puls = 1500.00;                   //servo position in pulses
float guide_min = 72.00;                //Left limit for filament guide (0 - 120 /180)
float guide_max = 94.00;                //Right limit for Filamnet guide (0 - 120 /180)
int guide_direction = 0;                //Direction the guide is moving
float guide_angle = 72.00;              //servo position in degrees (0 - 120 /180)




//Variables for smoothing the potentiometer reading:
const int numReadings = 3;
int readings[numReadings];  //readings from the analog Poti input
int index = 0;              //index of the current reading
int total = 0;              //running total
int average = 0;            //readingsaverage
int PotiValue = 0;          //Potentiometer value


//Variables for motor speed control
int motorSpeed = 0;
int rotation_status = 0;



//*********************************************************************************************************


//********
//*Setup:*
//********


void setup() 
{
  Serial.begin(115200);
  pinMode(toggle, INPUT);         // Pin 2 Auto- / Manual- Spool  Switch
  pinMode(3, INPUT);              // Pin 3 Button Guide Min.
  pinMode(setPin, INPUT);         // Pin 4 Button for setting (Sensor Cal) + ServoPosition
  pinMode(MOTOR_PIN, OUTPUT);     // Pin 5 Motorpin
  pinMode(guidePin, OUTPUT);      // Pin 6 ServoGuide
  pinMode(hall_Pin, INPUT);       // Pin 7 Hallsensor
  pinMode(8, INPUT);              // Pin 8 Button Guide Max.
  pinMode(9, INPUT);              // with Jumper to GND: Tetsmode 
  pinMode(10, INPUT);             // Expansion Port
  pinMode(11, INPUT);             // Troubleshoot-Mode: with Jumper to GND = Serialoutput ON
  pinMode(12, INPUT);             // Filamentdiameter 1.75mm (without Jumper to GND) or 3mm (with Jumper to GND)
  pinMode(13, OUTPUT);            // Serial-Output ( PIN 13 and USB )
  pinMode(sensor_1, INPUT);       // Pin 14 / A0
  pinMode(sensor_2, INPUT);       // Pin 15 / A1
  pinMode(sensor_3, INPUT);       // Pin 16 / A2
  pinMode(sensor_4, INPUT);       // Pin 17 / A3


  digitalWrite(toggle, HIGH);         // Pullup resistor for toggle switch
  digitalWrite(3, HIGH);              // Pullup resistor for guide setup switch
  digitalWrite(setPin, HIGH);         // Pullup resistor for pid setup switch
  digitalWrite(hall_Pin, HIGH);       // Pullup resistor for Hallsensor
  digitalWrite(8, HIGH);              // Pullup resistor for guide setup switch
  digitalWrite(9, HIGH);              // Jumper/Switch-Pin for TestMode
  digitalWrite(10, HIGH);             // Expansion Port
  digitalWrite(11, HIGH);             // Jumper/Switch-Pin for TroubleshootMode SerialPrint
  digitalWrite(12, HIGH);             // Jumper/Switch-Pin for Filamentdiameter 1.75mm (whitout Jumper) 3mm (whit Jumper) to GND

}


//*********************************************************************************************************

//*******
//*Loop:*
//*******

void loop() 
{
 // Servo calibrate_min:
  if (digitalRead(toggle) == 0 && digitalRead(3) == 0 && digitalRead(8) == 1)  //If guide min button is pressed set guide min
 {                                                                             //calibrate_min will loop while the button is held down.
   smoothing();                                                                //potivalue average 0 to 1023
   guide_angle = average / (1023 / SERVO_ANGLE_RANGE);                         //0 to servo angle range 180 or 120
   servo_set();
   guide_min = guide_angle;
 }



 // Servo calibrate_max:
  if (digitalRead(toggle) == 0 && digitalRead(3) == 1 && digitalRead(8) == 0)   //If guide max button is pressed go to calibrate_max
 {                                                                              //calibrate_max will loop while the button is held down.
   smoothing();                                                                 //potivalue average 0 to 1023
   guide_angle = average / (1023 / SERVO_ANGLE_RANGE);                          //0 to servo angle range 180 or 120
   servo_set();
   guide_max = guide_angle;
 }



 // guide-servo control:
 if (digitalRead(toggle) == 0 && digitalRead(setPin) == 0)   //If in automatic motor control mode (Pin2) + middle button (Pin4) is pressed, set the current guide position
 {
   smoothing();                                              // potivalue average 0 to 1023
   guide_angle = average / (1023 / SERVO_ANGLE_RANGE);       // 0 to servo angle range 180 or 120

   servo_set();
 }

 else 
 {
  auto_guide_control();
  }
 


 // motor control function:
 if (digitalRead(toggle) == 1)    //(Pin2) Switch OFF (0) (= Resistor Pullup = 1)
 {
   manual_motor_control();        //Go to the manual motor pull/speed control function
 }
 else 
 {
  auto_motor_control();           //Go to the automatic motor pull/speed control function
  }
 


 //Troubleshoot mode:
 if (digitalRead(11) == 0) 
 {
    serial_output();  
 }


 //Test mode:
 if (digitalRead(9) == 0) 
 {
    test();
 }
 
}
// ********Loop-End***********************************************************************************




// *****************
// **  Funktions  **
// *****************


//set servo:
void servo_set()
{
 puls = map(guide_angle,0.00,SERVO_ANGLE_RANGE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);   //set servo: value-mapping , 0 degree = servo min.  to  180 or 120 (servo-angle-range) degree = servo max.
 //give 4 pulses to give time to positioning
 digitalWrite(guidePin,HIGH);
 delayMicroseconds(puls);
 digitalWrite(guidePin,LOW);
 delayMicroseconds(REFRESH_INTERVAL - puls + 1);

 digitalWrite(guidePin,HIGH);
 delayMicroseconds(puls);
 digitalWrite(guidePin,LOW);
 delayMicroseconds(REFRESH_INTERVAL - puls + 1);

 digitalWrite(guidePin,HIGH);
 delayMicroseconds(puls);
 digitalWrite(guidePin,LOW);
 delayMicroseconds(REFRESH_INTERVAL - puls + 1);
         
 digitalWrite(guidePin,HIGH);
 delayMicroseconds(puls);
 digitalWrite(guidePin,LOW);
 delayMicroseconds(REFRESH_INTERVAL - puls + 1);

}



 
//****************************************


void auto_guide_control() 
{
  //The hall sensor might get checked several times while the magnet is in range, but we don't want a rotation logged with every check
  //while the magnet passes by.  When a rotation is logged, rotation status gets set to 1 so it doesn't get logged again until after the hall has
  //switched off.

  if (digitalRead(hall_Pin) == 0)   //Keep rotation status at 0 as long as hall isn't triggered
  {
    rotation_status = 0;
  }

  // If Hall A has been triggered and rotation status is 0, log a rotation
  if (digitalRead(hall_Pin) == 1 && rotation_status == 0) 
  {

    if (guide_angle < guide_min)   //If the guide angle passes minimum set direction to forward
    {
      guide_angle = guide_min;
      guide_direction = 0;
    }

    if (guide_angle > guide_max)   //If the guide angle has reached maximum change direction to back
    {
      guide_angle = guide_max;
      guide_direction = 1;
    }

    if (guide_direction == 0)     //If the current direction of the guide is forward
    {
      if (digitalRead(12) == 1)   // If there is no jumper on Pin 12 to GND
      {
        guide_angle = (guide_angle + 1.00);   //Move the guide +1 degree for 1.75mm filament, (Servoarm length = 100mm)
      }                  
      if (digitalRead(12) == 0)   //If there is a jumper on Pin 12 to GND
      {
        guide_angle = (guide_angle + 1.90);  //Move the guide 1.6 degrees for 3mm filament, (Servoarm length = 100mm)
      }


      puls = map(guide_angle,0.00,SERVO_ANGLE_RANGE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);   //set servo: value-mapping , 0 degree = servo min.  to  180 or 120 (servo-angle-range) degree = servo max.
     //give 4 pulses to give time to positioning
     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);

     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);
     
     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);
     
     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);

    }

    if (guide_direction == 1)     //If the current direction of the guide is back
    {
      if (digitalRead(12) == 1)   //If there is no jumper on Pin 12 to GND
      {
        guide_angle = (guide_angle - 1.00); //Move the guide -1 degree for 1.75mm filament, (Servoarm length = 100mm)
      } 
      if (digitalRead(12) == 0)  //If there is a jumper on Pin 12 to GND
      { 
        guide_angle = (guide_angle - 1.90);  //Move the guide -1.6 degrees for 3mm filament, (Servoarm length = 100mm)
      }
    
 
      puls = map(guide_angle,0.00,SERVO_ANGLE_RANGE,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);   //set servo: value-mapping , 0 degree = servo min.  to  180 or 120 (servo-angle-range) degree = servo max.
      //give 4 pulses to give time to positioning
     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);

     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);

     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);
          
     digitalWrite(guidePin,HIGH);
     delayMicroseconds(puls);
     digitalWrite(guidePin,LOW);
     delayMicroseconds(REFRESH_INTERVAL - puls + 1);

    }
  
    rotation_status = 1;             //Remember that a rotation was counted

  }
 // If Hall is still being triggered after the rotation has been counted (rotation_status = 0) nothing will happen.
}


//****************************************



//Smoothing the Potentiometer reading:
void smoothing() 
{   
  total = total - readings[index];        // subtract the last reading
  readings[index] = analogRead(poti_Pin); // read from the Potentiometer-Knob
  total = total + readings[index];        // add the reading to the total
  index = index + 1;                      // advance to the next position in the array
  if (index >= numReadings) 
  {                                       // if we're at the end of the array...
   index = 0;                             // ...wrap around to the beginning
   average = total / numReadings;         // calculate the average value 0 to 1023
  }
}


//****************************************


void manual_motor_control()                   //Motor = manual, (Guide = auto)
{
  smoothing();                                 // 0 to 1023
  motorSpeed = average / 4;                    //convert reading from pot to 0-255
  if (motorSpeed <= 3) {motorSpeed = 0;}       // reducing jumping-Values to min+max PotiValues
  if (motorSpeed >= 250) {motorSpeed = 255;}   // reducing jumping-Values to min+max PotiValues
  analogWrite(MOTOR_PIN, motorSpeed);          //Set motor to the speed
}



//****************************************


void auto_motor_control() 
{
 sensor_1 = analogRead(A0);   //read sensorvalues
 sensor_2 = analogRead(A1);
 sensor_3 = analogRead(A2);
 sensor_4 = analogRead(A3);
     
 if (((sensor_1 + sensor_2 + sensor_3 + sensor_4)/4)-20> sensor_1) {Line_Pos = 255;}  //smoothing + set the sensor with filamentshadow
 if (((sensor_1 + sensor_2 + sensor_3 + sensor_4)/4)-20> sensor_2) {Line_Pos = 128;}
 if (((sensor_1 + sensor_2 + sensor_3 + sensor_4)/4)-20> sensor_3) {Line_Pos = 64;}
 if (((sensor_1 + sensor_2 + sensor_3 + sensor_4)/4)-20> sensor_4) {Line_Pos = 0;}

 
 // PWM-maping motorspeed
 motorSpeed = Line_Pos;         //set motorspeed
 //if (motorSpeed <= 5) {motorSpeed = 0;}                             //calculate Values to min+max Values
 //if (motorSpeed >= 245) {motorSpeed = 255;}                         //calculate Values to min+max Values
 analogWrite(MOTOR_PIN, motorSpeed);                                  //set Motorspeed

}

//****************************************


// Serial Output to Pin 13 , when PIN 11 is a Jumper to Ground:
void serial_output() 
{
  Serial.print(" Hall-Status ");
  Serial.print(digitalRead(hall_Pin));

  Serial.print(" Max-Switch ");
  Serial.print(digitalRead(guide_maxPin));

  Serial.print(" Min-Switch ");
  Serial.print(digitalRead(guide_minPin));

  Serial.print(" Center-Switch ");
  Serial.print(digitalRead(setPin));

  Serial.print(" Auto-Switch ");
  Serial.print(digitalRead(toggle));

  Serial.print(" Potentiometer ");
  Serial.print(average);

  Serial.print(" Guide Angle ");
  Serial.print(guide_angle);

  Serial.print(" Motor Speed ");
  Serial.println(motorSpeed);

  Serial.println( );

  Serial.print(" Sensor 1 "); 
  Serial.print(sensor_1);

  Serial.print(" Sensor 2 "); 
  Serial.print(sensor_2);

  Serial.print(" Sensor 3 "); 
  Serial.print(sensor_3);

  Serial.print(" Sensor 4 "); 
  Serial.print(sensor_4);

  Serial.print(" Line Pos ");
  Serial.println(Line_Pos);


  delay (1);
}


//****************************************



// test, when PIN 9 is a Jumper to Ground:
void test() 
{
  while (digitalRead(9) == 0) 
  {
    analogWrite(MOTOR_PIN, 255);
    analogWrite(guidePin, 130);
    Serial.println("Testing");
    if 
    (digitalRead(hall_Pin) == 0 &&
     digitalRead(setPin) == 0 && 
     digitalRead(guide_minPin) == 0 && 
     digitalRead(guide_maxPin) == 0 && 
     digitalRead(toggle) == 0 && 
     digitalRead(9) == 0 && 
     digitalRead(10) == 0 && 
     digitalRead(11) == 0 && 
     digitalRead(12) == 0 && 
     analogRead(poti_Pin) > 900 && 
     analogRead(sensor_1) > 900 && 
     analogRead(sensor_2) > 900 && 
     analogRead(sensor_3) > 900 && 
     analogRead(sensor_4) > 900 && 
     analogRead(setPin) > 900 && 
     analogRead(5) > 900 && 
     analogRead(7) > 900  
    ) 
   {
      digitalWrite(13, HIGH);
      Serial.println("Verified");
    } 
    else 
    {
     digitalWrite(13, LOW);
    }
  }
}


