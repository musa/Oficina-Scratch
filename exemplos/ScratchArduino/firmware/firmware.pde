//ARDUINO FIRMWARE FOR S4A SOFTWARE - http://seaside.citilab.eu/scratch/arduino

//INPUT/OUTPUT CONFIGURATION
  //digital outputs (digital pins 10,11 and 13)
  //analog outputs (digital pins 5, 6 and 9)
  //analog inputs (analog pins)
  //digital inputs (digital pins 2 and 3)
  //servomotors RC (digital pins 4, 7, 8 and 12)
  
char outputs[10];
int states[10];
int pulseWidth;
unsigned long initialPulseTime;
unsigned long lastDataReceivedTime;
//servomotor ISR variables
volatile int updateServoMotors = false; 
volatile boolean newInterruption;

void setup()
{
  Serial.begin(38400);
  Serial.flush();
  configurePins();
  configureServomotors();
  lastDataReceivedTime = millis();
}
  
void loop()
{ 
  sendSensorValues();
  readSerialPort();
}

void configurePins()
{
  for (int index = 0; index < 10; index++) 
  { 
     states[index] = 0;
     pinMode(index+4, OUTPUT);
     digitalWrite(index+4, LOW); //reset pins
  }
  
  outputs[1] = 'a';//pin 5
  outputs[2] = 'a';//pin 6
  outputs[5] = 'a';//pin 9
  outputs[6] = 'd';//pin 10 
  outputs[7] = 'd';//pin 11
  outputs[9] = 'd';//pin 13
    
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  outputs[0] = 'c'; //pin 4
  outputs[3] = 'c'; //pin 7  
  outputs[4] = 's'; //pin 8
  outputs[8] = 's'; //pin 12
}

void configureServomotors()
{//servomotors interruption configuration (interruption each 10 ms on timer2)
  newInterruption = false;
  updateServoMotors = false; 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;//preescaler = 1024 
  TIMSK2 = (1<<TOIE2); //Timer2 Overflow Interrupt Enable  
  TCNT2 = 100;   //start timer 
}

void sendSensorValues()
{
     int sensorValues[6], readings[5], sensorIndex;
      
      for (sensorIndex = 0; sensorIndex < 6; sensorIndex++) //For analog sensors, calculate the median of 5 sensor readings in order to avoid variability and power surges
      {
        for (int p = 0; p < 5; p++)    
          readings[p] = analogRead(sensorIndex);    
          
        InsertionSort(readings, 5); //sort readings
        sensorValues[sensorIndex] = readings[2]; //select median reading
      }
     
     //send analog sensor values
     for (sensorIndex = 0; sensorIndex < 6; sensorIndex++)   ScratchBoardSensorReport(sensorIndex, sensorValues[sensorIndex]);
     
      //send digital sensor values
     ScratchBoardSensorReport(6, digitalRead(2)?1023:0);
     ScratchBoardSensorReport(7, digitalRead(3)?1023:0);
}

void InsertionSort(int* array, int n)
{
  for (int i = 1; i < n; i++)
    for (int j = i; (j > 0) && ( array[j] < array[j-1] ); j--) 
      swap( array, j, j-1 );
}

void swap (int* array, int a, int b)
{
   int temp = array[a];
   array[a] = array[b];
   array[b] = temp;
}

void ScratchBoardSensorReport(int sensor, int value)
{ //PicoBoard protocol, 2 bytes per sensor
  Serial.print( B10000000
                 | ((sensor & B1111)<<3)
                 | ((value>>7) & B111),
                BYTE);
  Serial.print( value & B1111111, BYTE);
}

void readSerialPort()
{//read serial port until actuators plot has arrived 

  int pin, inByte, sensorHighByte;
  boolean newSensorsPlot = false, complete = false;
  char parameter = '1';
  
   while (!complete) 
   {
      if (updateServoMotors) updateServomotors();
      
      if (Serial.available()) 
      {   
        inByte = Serial.read();
        lastDataReceivedTime = millis();
        
          if (parameter == '1') 
          {
            if (inByte >= 128) //high byte (most significant bit equal to 1)
            {
              newSensorsPlot = true;
              sensorHighByte = inByte;
              pin = ((inByte >> 3) & 15);
            }
          }
          else if (parameter == '2') 
          {
            states[pin - 4] = ((sensorHighByte & 7) << 7) + (inByte & 127);     
            updateActuator(pin - 4);
            complete = (pin == 13);
          }
          
          if (newSensorsPlot)
          {
            if (parameter == '2') parameter = '1'; 
            else parameter++;
          }
      }
     else checkScratchDisconnection();
   }
}

void reset()
{//With xbee module, we need to simulate the setup execution that occurs when a usb connection is opened or closed without this module
 
  for (int pos = 0; pos < 10; pos++)  //stop all actuators
  {
     states[pos] = 0;
     digitalWrite(pos + 2, LOW);
  }
  
  //reset servomotors
  newInterruption = false;
  updateServoMotors = false; 
  TCNT2 = 100;   
  
  //protocol handshaking
  sendSensorValues();
  lastDataReceivedTime = millis();
}

void updateActuator(int pinNumber)
{    
        if (outputs[pinNumber] == 'd')  digitalWrite(pinNumber + 4,states[pinNumber]);
        else if (outputs[pinNumber] == 'a')  analogWrite(pinNumber + 4, states[pinNumber]);
} 

void updateServomotors()
{        
   updateServoMotors = false;
   
    for (int p = 0; p < 10; p++)
    {     
      if (outputs[p] == 'c') servomotorC(p + 4, states[p]);
      if (outputs[p] == 's') servomotorS(p + 4, states[p]);
    }
} 

void servomotorC (int pinNumber, int dir)
{
    if (dir == 1) pulseWidth = 1300; //clockwise rotation
    else if (dir == 2) pulseWidth = 1700;////anticlockwise rotation
    else return;
    pulse(pinNumber);
}

void servomotorS (int pinNumber, int angle)
{
    if (angle < 0) pulseWidth = 0;
    else if (angle > 180) pulseWidth = 2400;
    else pulseWidth = (angle * 10) + 600;
    pulse(pinNumber);
    
}

void pulse (int pinNumber) {
    digitalWrite(pinNumber, HIGH);
    initialPulseTime = micros();
    while (micros() < pulseWidth + initialPulseTime){}     
    digitalWrite(pinNumber, LOW);
}

void checkScratchDisconnection()
{//the reset is necessary when using an wireless arduino board (because we need to ensure that arduino isn't waiting the actuators state from Scratch) or when scratch isn't sending information (because is how serial port close is detected)
  if (millis() - lastDataReceivedTime > 1000)   reset(); //reset state if actuators reception timeout = one second
}

ISR(TIMER2_OVF_vect) //Timer1 overflow interrupt vector handler
{//Timer 2 => 8 bits counter => 256 clock ticks
 //preeescaler = 1024 => this routine is called 61 (16.000.000/256/1024) times per second approximately => interruption period =  1 / 16.000.000/256/1024 = 16,384 ms
 
 //as we need a 20 ms interruption period but timer2 doesn't have a suitable preescaler for this, we program the timer with a 10 ms interruption period and we consider an interruption every 2 times this routine is called.
 //to have a 10 ms interruption period, timer2 counter must overflow after 156 clock ticks => interruption period = 1 / 16.000.000/156/1024 = 9,984 ms => counter initial value (TCNT) = 100

  if (newInterruption) 
  {
     TCNT2 = 100;  //reset timer
     updateServoMotors = true;
  } 
  
  newInterruption = !newInterruption;
}



