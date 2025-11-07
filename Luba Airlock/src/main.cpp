#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>


#define MIN_PWM 40
#define MAX_PWM 150
#define TARGET_RPM 60
#define CLICKS_PER_ROTATION 464.64

#define HOMING_PWM 50
#define HOMING_TIME 5 //Seconds to enguage motor to perform homing

#define CAPTURE_DIST 40 // Clicks to consider a target pos captured.
#define DOOR_ACTUATION_TIMEOUT 10 // Max time (secodns) for motor to stay on while opening or closing (not accounting for holding door open)
class DoorOpener{
  public:
    double Kp=0.1, Ki=0, Kd=0;
    
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
      if(doorState == 1)PositionTracking(openPos, false);
      currentPos = encoder.getCount();
    }

    void Home(){
      encoder.setCount(closePos);
      unsigned long homing_timestamp = millis();
      while(millis() - homing_timestamp < HOMING_TIME){
        PositionTracking(openPos, true);
      }
      encoder.setCount(openPos);
      Close();
    }

    void Open(){
      RunToPosition(openPos);
    }
    void Close(){
      RunToPosition(closePos);
    }
    

     
  //PID??????? limit open time

 private:
    int INA_PIN, INB_PIN, PWM_PIN, ENA_PIN, CS_PIN;
    int Encoder_A_Pin, Encoder_B_Pin;
    int ErrorPin;
    long openPos, closePos, currentPos;
    bool doorState; //Open or closed
    ESP32Encoder encoder;

    double pid_input, pid_output, pid_setpoint;
    PID motorPID;
    
    
    void Drive(bool dir, int pwm){
      if(pwm == 0){
        digitalWrite(INA_PIN, LOW);
        digitalWrite(INB_PIN, LOW);
      }else if(dir){
        digitalWrite(INA_PIN, LOW);
        digitalWrite(INB_PIN, HIGH);
      }else if(!dir){
        digitalWrite(INA_PIN, HIGH);
        digitalWrite(INB_PIN, LOW);
      } 
      analogWrite(PWM_PIN, pwm);
    }

    int PositionTracking(long targetPos, bool homing){//Non-Blocking run to pos
      currentPos = encoder.getCount();
      bool dir = 0;
      long positionError = currentPos-targetPos;
      if(positionError > 0)dir = 1;

      int trackingPWM = abs(positionError)/10;
      if(trackingPWM < MIN_PWM)trackingPWM = 0;
      if(homing)trackingPWM = HOMING_PWM;
      trackingPWM = (trackingPWM, MIN_PWM, MAX_PWM);
      Drive(dir, trackingPWM);
      Serial.println("Current Pos: "+String(currentPos)+"  PWM: "+String(trackingPWM));

      return positionError;
    }

    void RunToPosition(long targetPos){ //Blocking run to pos
      unsigned long start_timestamp = millis();
      while(PositionTracking(targetPos, false) < CAPTURE_DIST && millis()-start_timestamp <= DOOR_ACTUATION_TIMEOUT)
      if(millis()-start_timestamp >= DOOR_ACTUATION_TIMEOUT){
        digitalWrite(ErrorPin, HIGH);
        Drive(0,0);
      }   
    }

};

DoorOpener frontDoor;
DoorOpener backDoor;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  frontDoor.SetupEncoder(34, 35);
  frontDoor.SetupMotor(16, 17, 18);
  frontDoor.SetPositions(0, 1000);
  frontDoor.SetErrorPin(23);

  //backDoor.SetupEncoder();
  backDoor.SetupMotor(19, 21, 22);
  backDoor.SetPositions(0, 1000);
  backDoor.SetErrorPin(23);

  frontDoor.Home();
}

void loop() {
  Serial.println("Opening");
  frontDoor.Open();
  unsigned long start = millis();
  while(millis()-start < 10000)frontDoor.Run();

  Serial.println("Closeing");
  frontDoor.Close();
  delay(10000);


}

