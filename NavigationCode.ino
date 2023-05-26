#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

#define NO_OF_PATIENTS 3
#define ROW 5
#define pi 3.14159264
#define v_max 0.2125811
#define y_i 1
#define x_i 2

// Ultrasonic Data 
#define TRIGGER_PIN_FL  24
#define ECHO_PIN_FL     26

#define TRIGGER_PIN_FR  30
#define ECHO_PIN_FR     32

#define TRIGGER_PIN_SL  36
#define ECHO_PIN_SL     38

#define TRIGGER_PIN_SR  42
#define ECHO_PIN_SR     44

#define MAX_DISTANCE 25 // Maximum distance we want to measure (in centimeters).

LiquidCrystal_I2C lcd(0x3F, 16, 2);

NewPing sonar_FL(TRIGGER_PIN_FL, ECHO_PIN_FL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar_FR(TRIGGER_PIN_FR, ECHO_PIN_FR, MAX_DISTANCE);
NewPing sonar_SR(TRIGGER_PIN_SR, ECHO_PIN_SR, MAX_DISTANCE);
NewPing sonar_SL(TRIGGER_PIN_SL, ECHO_PIN_SL, MAX_DISTANCE);

int DAY = 24 * 60 * 60 * 1000;
int T_START = 20 * 500;
int T_QR = 15 * 1000;
const int COL = NO_OF_PATIENTS * 2;
double theta_dot_90 =  v_max / 0.41;
double theta_dot_270 =  v_max / 0.392;

int days = 0;
int patient_no = 1;
char* v_rpm = "50";
char* v_zero = "0";

double move_matrix [ROW][COL] = 
{
  //Patient 1         //Patient 2         //Patient 3
  //x         //y       //x       //y      //x       //y
  {0,           0,          0,            0,          0,            0},
  {0,           1,          0,            1,          0,            2},
  {1.25,        0,       1.25,            0,       1.25,            0},
  {0.785,       0,      0.785,            0,      0.785,            0},
  {0,       1.375,          0,        1.375,          0,        1.375}
};

void wait_for_next_day(unsigned long start){
  while((millis() - start) < DAY){     
  }
}

void turn_CCW(double angle){
  Serial1.write(v_rpm, 15);
  unsigned long t_ultra = 0;
  unsigned long t_ccw = millis();
  while((millis() - t_ccw - t_ultra) < (angle * 1000)){
    int distance_FL = sonar_FL.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    int distance_FR = sonar_FR.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    
    if(((distance_FL <= MAX_DISTANCE) and (distance_FL > 0)) or ((distance_FR > 0) and (distance_FR <= MAX_DISTANCE))){
      lcd.clear(); 
      lcd.print("STOP");

      Serial1.write(v_zero,15);
      Serial2.write(v_zero,15);
      t_ultra = millis() - t_ccw;
      

      while(((distance_FL <= MAX_DISTANCE) and (distance_FL > 0)) or ((distance_FR > 0) and (distance_FR <= MAX_DISTANCE))){
        distance_FL = sonar_FL.ping_cm();
        distance_FR = sonar_FR.ping_cm();
        t_ultra = millis() - t_ccw;
      }
      Serial1.write(v_rpm, 15); 
    } 
 }
}

void turn_CW(double angle){
  Serial2.write(v_rpm, 15);
  unsigned long t_cw = millis();
  unsigned long t_ultra = 0;
  while((millis() - t_cw - t_ultra) < (angle * 1000)){  
    int distance_FL = sonar_FL.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    int distance_FR = sonar_FR.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    
    if(((distance_FL <= MAX_DISTANCE) and (distance_FL > 0)) or ((distance_FR > 0) and (distance_FR <= MAX_DISTANCE))){\
      lcd.clear(); 
      lcd.print("STOP");

      Serial1.write(v_zero,15);
      Serial2.write(v_zero,15);
      t_ultra = millis() - t_cw;
      

      while(((distance_FL <= MAX_DISTANCE) and (distance_FL > 0)) or ((distance_FR > 0) and (distance_FR <= MAX_DISTANCE))){
        distance_FL = sonar_FL.ping_cm();
        distance_FR = sonar_FR.ping_cm();
        t_ultra = millis() - t_cw;
      }
      Serial2.write(v_rpm, 15); 
    }     
  }
}

void stop(void){
  Serial1.write(v_zero, 15);
  Serial2.write(v_zero, 15);
  unsigned long t_s = millis();
  while((millis() - t_s) < (0.5 * 1000)){     
  }
}

void move_fwd2(double D_fwd){
  Serial1.write(v_rpm, 15);
  unsigned long t_z = millis();
  unsigned long t_ultra = 0;
  while((millis() - t_z) < 30){     
  }
  Serial2.write(v_rpm, 15);
  unsigned long t_f = millis();
  while((millis() - t_f - t_ultra) < (D_fwd * 1000)){
    int distance_FL = sonar_FL.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    int distance_FR = sonar_FR.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
    
    if(((distance_FL <= MAX_DISTANCE) and (distance_FL > 0)) or ((distance_FR > 0) and (distance_FR <= MAX_DISTANCE))){
      lcd.clear(); 
      lcd.print("STOP");

      Serial1.write(v_zero,15);
      Serial2.write(v_zero,15);
      t_ultra = millis() - t_f;
      

      while(((distance_FL <= MAX_DISTANCE) and (distance_FL > 0)) or ((distance_FR > 0) and (distance_FR <= MAX_DISTANCE))){
        distance_FL = sonar_FL.ping_cm();
        distance_FR = sonar_FR.ping_cm();
        t_ultra = millis() - t_f;
      }
      Serial1.write(v_rpm, 15);
      unsigned long t_z = millis();
      while((millis() - t_z) < 30){     
      }
      Serial2.write(v_rpm, 15); 
    }      
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  lcd.init();
  lcd.backlight();
}

void loop() {
  /*  PID Only Test  */
  int distance_SL = sonar_SL.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)
  int distance_SR = sonar_SR.ping_cm(); // Send ping, get distance in cm and print result (0 = outside set distance range)

  if ((((distance_SL <= MAX_DISTANCE) and (distance_SL >0))and (((distance_SR <= MAX_DISTANCE) and (distance_SR > 0))))){
    unsigned long t_z = millis();
    while((millis() - t_z) < 50){     
    }
    if ((((distance_SL <= MAX_DISTANCE) and (distance_SL >0))and (((distance_SR <= MAX_DISTANCE) and (distance_SR > 0))))){
      Serial1.write(v_rpm, 15);
      Serial2.write(v_rpm, 15);
      while(1){}
    }
  }

  /* Main Operation */
  unsigned long daystart = millis();
  unsigned long currentm = millis();
  while((millis() - currentm) < T_START){
  }
  if(patient_no % 2 == 0){
    move_fwd2(move_matrix [1][(2 * patient_no) - y_i] / v_max);
    stop();
    turn_CW(double(pi / 2) / theta_dot_90);
    stop();
    move_fwd2(move_matrix [2][(2 * patient_no) - x_i] / v_max);
    stop();
    unsigned long QRm = millis();
    while((millis() - QRm) < T_QR){
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command == "R2"){
          lcd.print("     Room 2     ");
          lcd.setCursor(0, 1);
          lcd.print("Medicine Slot 2");
        }
      }
    }
    turn_CCW(double(3.0 * pi / 2.0) / theta_dot_270);
    stop();
    turn_CW(double(pi / 2.0) / theta_dot_90);
    stop();
    move_fwd2(move_matrix [3][(2 * patient_no) - x_i] / v_max);
    stop();
    turn_CCW(double(pi / 2.0) / theta_dot_90);
    stop();
    move_fwd2(move_matrix [4][(2 * patient_no) - y_i] / v_max);
    stop();
    turn_CW(double(3.0 * pi / 2.0) / theta_dot_270);
    stop();
    turn_CCW(double(pi / 2.0) / theta_dot_90);
    stop();
  }
  else{
    move_fwd2(move_matrix [1][(2 * patient_no) - y_i] / v_max);
    stop();
    turn_CCW(double(pi / 2.0) / theta_dot_90);
    stop();
    move_fwd2(move_matrix [2][(2 * patient_no) - x_i] / v_max);
    stop();
    unsigned long QRm = millis();
    while((millis() - QRm) < T_QR){
      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command == "R1"){
          lcd.print("     Room 1     ");
          lcd.setCursor(0, 1);
          lcd.print("Medicine Slot 1");
        }
        else if (command == "R3"){
          lcd.print("     Room 3     ");
          lcd.setCursor(0, 1);
          lcd.print("Medicine Slot 3");
        }
      }
    }
    turn_CW(double(3.0 * pi / 2.0) / theta_dot_270);
    stop();
    turn_CCW(double(pi / 2.0) / theta_dot_90);
    stop();
    move_fwd2(move_matrix [3][(2 * patient_no) - x_i] / v_max);
    stop();
    turn_CW(double(pi / 2.0) / theta_dot_90);
    stop();
    move_fwd2(move_matrix [4][(2 * patient_no) - y_i] / v_max);
    stop();
    turn_CW(double(3.0 * pi / 2.0) / theta_dot_270);
    stop();
    turn_CCW(double(pi / 2.0) / theta_dot_90);
    stop();
  }

  patient_no++;
  if(patient_no>NO_OF_PATIENTS){
    wait_for_next_day(daystart);
    days++;
  }

}
