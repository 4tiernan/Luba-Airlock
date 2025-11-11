#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>


#define MIN_PWM 60
#define MAX_PWM 150
#define TARGET_RPM 60
#define CLICKS_PER_ROTATION 464.64
#define PID_FREQUENCY 50 //Hz


#define HOMING_RPM 60
#define HOMING_TIME 5 //Seconds to enguage motor to perform homing

#define CAPTURE_DIST 100 // Clicks to consider a target pos captured.
#define DOOR_ACTUATION_TIMEOUT 10 // Max time (secodns) for motor to stay on while opening or closing (not accounting for holding door open)
#define MAX_DOOR_OPEN_TIME 30 //Max time (seconds) for door to remain open
class DoorOpener{
  public:
    long currentPos;
    double Kp=0.01, Ki=0, Kd=0.00;
    // Constructor
    DoorOpener() : motorPID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, DIRECT) {
    }

    void SetupMotor(int _INA, int _INB, int _PWM){
      INA_PIN = _INA;
      INB_PIN = _INB;
      //ENA_PIN = _ENA;
      PWM_PIN = _PWM;
      //CS_PIN = _CS;
      pinMode(INA_PIN, OUTPUT);
      pinMode(INB_PIN, OUTPUT);
      pinMode(PWM_PIN, OUTPUT);
      motorPID.SetMode(AUTOMATIC);
      motorPID.SetOutputLimits(-255, 255);
    }

    void SetErrorPin(int pin){ //Set pin to drive high if fault occours
      ErrorPin = pin;
      pinMode(ErrorPin, OUTPUT);
    }

    void SetupEncoder(int PIN_A, int PIN_B){
      Encoder_A_Pin = PIN_A;
      Encoder_B_Pin = PIN_B;
      encoder.attachHalfQuad(Encoder_A_Pin, Encoder_B_Pin);
      encoder.setCount(0);
    }

    void SetPositions(long open, long closed){
      openPos = open;
      closePos = closed;
    }

    void Run(){
      if(doorState == true){
        if(millis() - door_open_timestamp < MAX_DOOR_OPEN_TIME*1000){
          //PositionTracking(openPos, false);
        }else{
          Close();
        }
      }
            
      currentPos = encoder.getCount();
    }

    void Home(){
      encoder.setCount(closePos);
      unsigned long homing_timestamp = millis();
      Serial.println("Homing");
      while(millis() - homing_timestamp < HOMING_TIME*1000){
        //PositionTracking(openPos, true);
      }
      Serial.println("Homing Complete, reset encoder count");
      encoder.setCount(openPos);
      Close();
    }

    void Open(){
      doorState = true;
      door_open_timestamp = millis();
      last_motor_pwm = -100;

      Serial.println("Open");
      while(currentPos > -2400 && millis()-door_open_timestamp <20000){
        DrivePID(-60, false);
      }
      last_motor_pwm = 0;
      Break(255);
    }
    void Close(){
      doorState = false;
      Serial.println("Close");
      unsigned long start = millis();
      while(currentPos < 0 && millis()-start < 20000){
        DrivePID(60, true);
      }
      last_motor_pwm = 0;
      Break(255);
    }
    /*
    int PositionTracking(long targetPos, bool homing){//Non-Blocking run to pos
      currentPos = encoder.getCount();
      
      bool dir = 0;
      long positionError = currentPos-targetPos;
      if(positionError > 0)dir = 1;
      if(digitalRead(ErrorPin))return positionError;

      int trackingPWM = abs(positionError)*0.2;
      trackingPWM = constrain(trackingPWM, 0, MAX_PWM);
      if(trackingPWM < MIN_PWM)trackingPWM = 0;
      if(homing)trackingPWM = HOMING_PWM;
      Drive(dir, trackingPWM);
      Serial.println("Current Pos: "+String(currentPos)+" Target Pos: "+String(targetPos)+"  PWM: "+String(trackingPWM)+"  Error: "+String(positionError));
      return positionError;
    }*/
    void DrivePID(int target_rpm, bool closing){
      if(millis()-last_pid_timestamp > (1/double(PID_FREQUENCY))*1000){
        currentPos = encoder.getCount();
        double minutes = ((millis()-last_pid_timestamp)/1000.0)/60.0;
        double rotations = (currentPos-last_speed_pos)/CLICKS_PER_ROTATION;
        double rpm = rotations/minutes;

        pid_input = rpm;
        pid_setpoint = target_rpm;
        //Serial.println("  Clicks: "+String(currentPos-last_speed_pos));
        //Serial.println("  Current Pos: "+String(currentPos));

        last_pid_timestamp = millis();
        last_speed_pos = currentPos;

        motorPID.Compute();
        last_motor_pwm += pid_output;
        last_motor_pwm = constrain(last_motor_pwm, -255, 255);
        
        if(closing && abs(rpm) > abs(target_rpm)*0){
          Break(abs(last_motor_pwm)*0.1);
          Serial.println("BREAKINGASDFG");
        }else{
          Drive(last_motor_pwm);
        }
        Serial.println("closing: "+String(closing)+" PWM: "+String(last_motor_pwm)+" pid: "+String(pid_output)+"  RPM: "+String(rpm)+"  Target: "+String(target_rpm)+"  Current Pos: "+String(currentPos));
      }      
    }

    int Sign(int val){
      if(val > 0)return 1;
      if(val < 0)return -1;
      return 0;
    }

     
  //PID??????? limit open time

 private:
    int INA_PIN, INB_PIN, PWM_PIN, ENA_PIN, CS_PIN;
    int Encoder_A_Pin, Encoder_B_Pin;
    int ErrorPin;
    double last_motor_pwm; 
    long openPos, closePos, last_speed_pos;
    bool doorState; //Open or closed
    unsigned long door_open_timestamp, last_pid_timestamp;
    ESP32Encoder encoder;

    double pid_input, pid_output, pid_setpoint;
    PID motorPID;
    
    
    void Drive(int pwm){ //Takes -255 to 255
      pwm = constrain(pwm, -255, 255);
      if(pwm == 0){
        digitalWrite(INA_PIN, LOW);
        analogWrite(INB_PIN, 0);
      }else if(pwm > 0){
        digitalWrite(INA_PIN, LOW);
        analogWrite(INB_PIN, 255);
      }else if(pwm < 0){
        digitalWrite(INA_PIN, HIGH);
        digitalWrite(INB_PIN, LOW);
      } 
      analogWrite(PWM_PIN, abs(pwm));
    }

    void Break(int pwm){
      pwm = constrain(pwm, 0, 255);
      digitalWrite(INA_PIN, HIGH);
      analogWrite(INB_PIN, abs(pwm));
      analogWrite(PWM_PIN, abs(0));
    }

    

    void RunToPosition(long targetPos){ //Blocking run to pos
      unsigned long start_timestamp = millis();
      if(digitalRead(ErrorPin))return;
      
      //while(PositionTracking(targetPos, false) < CAPTURE_DIST && millis()-start_timestamp <= DOOR_ACTUATION_TIMEOUT*1000);

      if(millis()-start_timestamp >= DOOR_ACTUATION_TIMEOUT*1000){
        digitalWrite(ErrorPin, HIGH);
        Serial.println("Door Actuation Timeout");
        //Drive(0,0);
      }   
    }

};

DoorOpener frontDoor;
DoorOpener backDoor;

void setup() {
  Serial.begin(115200);
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
  // put your setup code here, to run once:
  frontDoor.SetupEncoder(34, 35);
  frontDoor.SetupMotor(17, 16, 18);
  frontDoor.SetPositions(0, 2500);
  frontDoor.SetErrorPin(23);

  //backDoor.SetupEncoder();
  backDoor.SetupMotor(19, 21, 22);
  backDoor.SetPositions(0, 1000);
  backDoor.SetErrorPin(23);

  //frontDoor.Home();
}
void loop() {
  //frontDoor.PositionTracking(1000, false);
  frontDoor.Open();
  delay(1000);
  frontDoor.Close();
  delay(1000);

  
  
  
  /*
  Serial.println("Opening");
  frontDoor.Open();
  unsigned long start = millis();
  while(millis()-start < 10000)frontDoor.Run();

  Serial.println("Closeing");
  frontDoor.Close();
  delay(10000);
  */

}

