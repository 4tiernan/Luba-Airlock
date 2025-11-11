#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>


#define MIN_PWM 60
#define MAX_PWM 150
#define OPENING_RPM 100
#define CLOSING_RPM 50
#define CLICKS_PER_ROTATION 464.64
#define PID_FREQUENCY 50 //Hz

#define HOMING_PWM 100
#define HOMING_TIME 7 //Seconds to enguage motor to perform homing

#define CAPTURE_DIST 10 // Clicks to consider a target pos captured.
#define DOOR_ACTUATION_TIMEOUT 20 // Max time (secodns) for motor to stay on while opening or closing (not accounting for holding door open)
#define MAX_DOOR_OPEN_TIME 30 //Max time (seconds) for door to remain open
int Sign(int val){
  if(val > 0)return 1;
  if(val < 0)return -1;
  return 0;
}

class DoorOpener{
  public:
    long currentPos;
    double rpm;
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

    void SetPositions(long open, long closed, long offset){
      openPos = open;
      closePos = closed;
      homePosOffset = offset;
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
      unsigned long homing_timestamp = millis();
      Serial.println("Homing");
      while(rpm == 0 && millis() - homing_timestamp < 1000){
        Drive(-200);
        Serial.println("RPM: 0!!");
      }
      unsigned long last_movement_timestamp = millis();
      while(millis()-last_movement_timestamp < 1000 && millis() - homing_timestamp < HOMING_TIME*1000){
        Drive(-HOMING_PWM);
        if(abs(rpm) > 5)last_movement_timestamp = millis();
        delay(10);
        //Serial.println(millis());
        //Serial.println("RPM: "+String(rpm));
      }
      Break(255);
      if(millis() - homing_timestamp >= HOMING_TIME*1000 || currentPos == 0){
        Serial.println("Homing failed!!!");
        digitalWrite(ErrorPin, HIGH);
        while(true);
      }else{
        //Serial.println("POS: "+String(currentPos));
        Serial.println("Homing Complete, reset encoder count to: "+String(openPos+homePosOffset));
        encoder.setCount(openPos+homePosOffset);
        EncoderCompute();
        Close();
      }
    }

    void Open(){
      doorState = true;
      door_open_timestamp = millis();
      last_motor_pwm = -100;

      Serial.println("Open");
      while(abs(currentPos-openPos) > CAPTURE_DIST && millis()-door_open_timestamp < DOOR_ACTUATION_TIMEOUT*1000){
        DrivePID(Sign(openPos-currentPos)*OPENING_RPM, false);
      }
      last_motor_pwm = 0;
      Break(255);
      if(millis()-door_open_timestamp > DOOR_ACTUATION_TIMEOUT*1000){
        Serial.println("Failed to Close door");
        digitalWrite(ErrorPin, HIGH);
        while(true);
      }
    }
    void Close(){
      doorState = false;
      last_motor_pwm = 50;
      Serial.println("Close");
      unsigned long start = millis();
      while(abs(closePos-currentPos) > CAPTURE_DIST && millis()-start < DOOR_ACTUATION_TIMEOUT*1000){
        DrivePID(Sign(closePos-currentPos)*CLOSING_RPM, true);
      }
      last_motor_pwm = 0;
      Break(255);
      if(millis()-start > DOOR_ACTUATION_TIMEOUT*1000){
        Serial.println("Failed to Close door");
        digitalWrite(ErrorPin, HIGH);
        while(true);
      }
    }

    void DrivePID(int target_rpm, bool closing){
      if(millis()-last_pid_timestamp > (1/double(PID_FREQUENCY))*1000){
        EncoderCompute();
        pid_input = rpm;
        pid_setpoint = target_rpm;
        //Serial.println("  Clicks: "+String(currentPos-last_speed_pos));
        //Serial.println("  Current Pos: "+String(currentPos));

        last_pid_timestamp = millis();
        last_speed_pos = currentPos;

        motorPID.Compute();
        last_motor_pwm += pid_output;
        last_motor_pwm = constrain(last_motor_pwm, -255, 255);
        if(abs(rpm) > abs(target_rpm)+50){
          breaking_active = true;
        }else if(abs(rpm) == 0){
          breaking_active = false;
        }
        
        if(closing && breaking_active){
          Break(abs(rpm-target_rpm)*5);
        }else{
          Drive(last_motor_pwm);
        }
        //Serial.println("PWM: "+String(last_motor_pwm)+" abs(rpm-target_rpm)*10: "+String(abs(rpm-target_rpm)*10)+"  RPM: "+String(rpm)+"  Target: "+String(target_rpm)+"  Current Pos: "+String(currentPos));
      }      
    }


     
  //PID??????? limit open time

 private:
    int INA_PIN, INB_PIN, PWM_PIN, ENA_PIN, CS_PIN;
    int Encoder_A_Pin, Encoder_B_Pin;
    int ErrorPin;
    double last_motor_pwm; 
    long openPos, closePos, homePosOffset, last_speed_pos;
    bool doorState; //Open or closed
    bool breaking_active;
    unsigned long door_open_timestamp, last_pid_timestamp, last_rpm_timestamp;
    ESP32Encoder encoder;

    double pid_input, pid_output, pid_setpoint;
    PID motorPID;
    
    
    void Drive(int pwm){ //Takes -255 to 255
      pwm = constrain(pwm, -255, 255);
      if(pwm == 0){
        analogWrite(INA_PIN, 0);//Using PWM as breaking function requires it
        analogWrite(INB_PIN, 0);
      }else if(pwm > 0){
        analogWrite(INA_PIN, 0);
        analogWrite(INB_PIN, 255);
      }else if(pwm < 0){
        analogWrite(INA_PIN, 255);
        analogWrite(INB_PIN, 0);
      } 
      analogWrite(PWM_PIN, abs(pwm));
      EncoderCompute();
    }

    void Break(int pwm){
      pwm = constrain(pwm, 0, 255);
      analogWrite(INA_PIN, abs(pwm));
      analogWrite(INB_PIN, abs(pwm));
      analogWrite(PWM_PIN, abs(0));
      EncoderCompute();
    }

    void EncoderCompute(){
      if(millis()-last_rpm_timestamp > 20){
        currentPos = encoder.getCount();
        double minutes = ((millis()-last_rpm_timestamp)/1000.0)/60.0;
        double rotations = (currentPos-last_speed_pos)/CLICKS_PER_ROTATION;
        rpm = rotations/minutes;
        last_speed_pos = currentPos;
        last_rpm_timestamp = millis();
        Serial.println(currentPos); 
      }
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
  frontDoor.SetPositions(-2550, 0, -250);
  frontDoor.SetErrorPin(23);

  //backDoor.SetupEncoder();
  //backDoor.SetupMotor(19, 21, 22);
  //backDoor.SetPositions(0, -2450, 100);
  //backDoor.SetErrorPin(23);

  frontDoor.Home();
}
void loop() {
  //frontDoor.PositionTracking(1000, false);
  delay(3000);
  frontDoor.Open();
  delay(3000);
  frontDoor.Close();
  

  
  
  
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

