// Libraries

// DEFINITIONS

// Motor and H-bridge Data 
#define interruptPinA  3
#define interruptPinB  2
#define PWMPin  6
#define DirPin1  8
#define DirPin2  7
#define pi 3.14
#define GearBox 70
//#define RedLed 13
//#define GreenLed 12
//#define YellowLed 11

char mystring_set_Left[15];     //String data which is to be recieved

// PID Parameters
float kp = 0.25;         //0.25
float ki = 0.00025;      // 0.00025
float kd = 0;            // 0

// INITILAIZAIONS
unsigned long t;
unsigned long t_prev = 0;

const int filterFactor = 1000;  // Filter factor (larger numbers result in more smoothing)
float RPM_Filter;  // Actual motor rpm 

volatile long EncoderCount = 0;

volatile unsigned long count = 0;
unsigned long count_prev = 0;

float Theta, RPM, RPM_d; 
float Theta_prev = 0;
float RPM_max = 80;      // this is our motor max rpm
float V_prev = 0;

float Vmax = 12;
float Vmin = -12;
float V = 0.1;

float e, e_prev = 0, inte, inte_prev = 0;
int dt;


//*********FUNCTIONS*****************
//     Void ISR_EncoderA
//     Void ISR_EncoderB
//     Void Motor Driver Write
//     Timer Interrupt
//*************************************
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    } else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    } else {
      EncoderCount++;
    }
  }
}

void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    } else {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    } else {
      EncoderCount--;
    }
  }
}

//**Motor Driver Functions****

void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  } // double safety 
  if (V > 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
  } 
  else if (V < 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
  } 
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
  }
  analogWrite(PWMPin, PWMval);
}

void setup() {

  Serial.begin(9600);
  delay(250); //due to latency in this motor 
  
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  /*are macros that map the digital pin numbers to their corresponding interrupt numbers. 
  This is necessary because not all digital pins on the Arduino board can be used as interrupts, 
  and the mapping is specific to each board.*/
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  

  //Leds to indicate range 
  //pinMode(GreenLed, OUTPUT);
  //pinMode(RedLed, OUTPUT);
  //pinMode(YellowLed, OUTPUT);

 // Timer Setup for Intrupets "Critical Section"
  cli();    // clear interupet 
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 25498;  //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei(); // setinterupet 
}

// Void Loop
void loop() {
  
  //Serial.print("Left Set Point RECEIVER: ");
  //Serial.println(RPM_d);

  if(Serial.available() > 0){
    Serial.readBytes(mystring_set_Left,15);  //Read the serial data
    RPM_d = float(atoi(mystring_set_Left));
  }

  /*if (RPM_d == 0){
    digitalWrite(RedLed,HIGH);
    digitalWrite(GreenLed,LOW);
    digitalWrite(YellowLed,LOW);
  }
  else if (RPM_d == 50){
    digitalWrite(GreenLed,HIGH);
    digitalWrite(RedLed,LOW);
    digitalWrite(YellowLed,LOW);
  } 
  else if (RPM_d == 70){
    digitalWrite(YellowLed,HIGH);
    digitalWrite(GreenLed,LOW);
    digitalWrite(RedLed,LOW);
  } 
  else if (RPM_d == 45){
    digitalWrite(GreenLed,HIGH);
    digitalWrite(RedLed,HIGH);
    digitalWrite(YellowLed,LOW);
  }*/ 
  if (count > count_prev) {
    t = millis();
    Theta = (EncoderCount * 2 * pi) / (44 * GearBox);
    dt = (t - t_prev);

    if (t / 1000.0 > 10000) {
      RPM_d = 0;
    }

    RPM = ((Theta - Theta_prev) / (2 * pi)) / ((dt / 1000.0) / 60);

    RPM_Filter = 0.854 * RPM_Filter + 0.0728 * RPM + 0.0728 * V_prev;
    
    e = RPM_d - RPM_Filter;
    
    inte = inte_prev + (dt * (e + e_prev) / 2);
    
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt);
    
    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
    }
  
    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
    V_prev = RPM;

    WriteDriverVoltage(V, Vmax);
    
    //Plotting 
    /*Serial.println(0);  // To freeze the lower limit
    Serial.print(" ");
    Serial.println(80);  // To freeze the upper limit
    Serial.print(" ");
    Serial.print("RPM_Filter:");
    Serial.println(RPM_Filter);
    Serial.print(",");
    Serial.print("RPM_d:");
    Serial.println(RPM_d);
    Serial.print(",");
    Serial.print("RPM:");
    Serial.println(RPM);*/
  }
}

ISR(TIMER1_COMPA_vect) {
  count++;
}
