// 15/03/2022 - Chariot robot with PID
#include <HUSKYLENS.h>
#include <Servo.h>
void(* resetFunc) (void) = 0;       // soft reset function
Servo sR,sL;                        // right servo and left servo
int R, L;                           // right servo value, left servo value
HUSKYLENS huskylens; int ID1=1;     // HuskyLens objet
HUSKYLENSResult result;             // HuskyLens values
float Kp = 0.5625, Ki = 0.001, Kd = 0.055;  // PID constants
int pid, p=0, i=0, d=0, err=0, lastErr=0;   // PID variables

void setup() {
  delay(400);                       // for soft reset consideration
  Serial.begin(9600);               // Serial initialization
  pinMode(6,INPUT_PULLUP);          // start/stop/reset button attachment
  sL.attach(2);sL.write(90);        // left   servo, 0->FW, 90->stop, 180->BW
  sR.attach(3);sR.write(90);        // right  servo, 0->FW, 90->stop, 180->BW
  Wire.begin();                     // I2C initialization
  while (!huskylens.begin(Wire)) {Serial.print("\n Check I2C"); delay(100);}
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch to line tracking.
  Serial.print("\n\t To start, click on the Start button"); while( digitalRead(6) ); delay(400);
}

void loop() {
  if (! digitalRead(6)) resetFunc();
  huskylens.request(ID1);  result = huskylens.read();
  err = map(result.xTarget,0,320,-160,160); //Serial.print(String()+("\n err=")+err);  // getting and maping err
  p = Kp*err;  i = Ki*(i+err);  d = Kd*(err-lastErr);  pid = p+i+d;  lastErr = err;
  if ( pid < 0) { L=-pid; R=0; }  else  { L=0; R=pid; } // error makes turning left or right
  sL.write(L);  sR.write(R);                            // left and right servos order
}
