#include <DeltaKinematics.h>

DeltaKinematics DK(256, 400, 45, 270);


int PUL_1 = 7; //define Pulse pin
int DIR_1 = 6; //define Direction pin
int ENA_1 = 5; //define Enable Pin

int PUL_2 = 10; //define Pulse pin
int DIR_2 = 9; //define Direction pin
int ENA_2 = 8; //define Enable Pin

int PUL_3 = 13; //define Pulse pin
int DIR_3 = 12; //define Direction pin
int ENA_3 = 11; //define Enable Pin

long motor1Speed = 20;
long motor2Speed = 10;

unsigned long pMotor1Time = millis();
unsigned long pMotor2Time = millis();

int NC = 32;
float angle = 1.8;
float ratio = 3969 / 289; //TI SO TRUYEN 3969/289


void setup() {
  pinMode (PUL_1, OUTPUT);
  pinMode (DIR_1, OUTPUT);
  pinMode (ENA_1, OUTPUT);

  pinMode (PUL_2, OUTPUT);
  pinMode (DIR_2, OUTPUT);
  pinMode (ENA_2, OUTPUT);

  pinMode (PUL_3, OUTPUT);
  pinMode (DIR_3, OUTPUT);
  pinMode (ENA_3, OUTPUT);

  Serial.begin(115200);

  digitalWrite(DIR_1, LOW);
  digitalWrite(ENA_1, HIGH);

  digitalWrite(DIR_2, LOW);
  digitalWrite(ENA_2, HIGH);

  digitalWrite(DIR_3, LOW);
  digitalWrite(ENA_3, HIGH);
}

void RunTheta(int motor, float theta, int spd)
{
  int stp;
  switch (motor) {
    case 1:
      if (theta < 0) {
        digitalWrite(DIR_1, LOW);
        theta = -theta;
      }
      else digitalWrite(DIR_1, HIGH);
      stp = (theta / (angle / NC)) * ratio;
      for (int i = 0; i < stp; i++)
      {
        digitalWrite(PUL_1, HIGH);
        delayMicroseconds(100 - spd);
        digitalWrite(PUL_1, LOW);
        delayMicroseconds(100 - spd);
      }
      break;

    case 2:
      if (theta < 0) {
        digitalWrite(DIR_2, LOW);
        theta = -theta;
      }
      else digitalWrite(DIR_2, HIGH);
      stp = (theta / (angle / NC)) * ratio;
      for (int i = 0; i < stp; i++)
      {
        digitalWrite(PUL_2, HIGH);
        delayMicroseconds(100 - spd);
        digitalWrite(PUL_2, LOW);
        delayMicroseconds(100 - spd);
      }
      break;

    case 3:
      if (theta < 0) {
        digitalWrite(DIR_3, LOW);
        theta = -theta;
      }
      else digitalWrite(DIR_3, HIGH);
      stp = (theta / (angle / NC)) * ratio;
      for (int i = 0; i < stp; i++)
      {
        digitalWrite(PUL_3, HIGH);
        delayMicroseconds(100 - spd);
        digitalWrite(PUL_3, LOW);
        delayMicroseconds(100 - spd);
      }
      break;
  }
}

void loop() {
  //RunTheta(PUL_1, 45, 90);
  //delay(1000);

  DK.inverse(0, 0, -200); // position
  //DK.forward(20, 20, 20); // angle
  RunTheta(1, DK.a, 95);
  RunTheta(2, DK.b, 95);
  RunTheta(3, DK.c, 95);
  Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
  Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
  Serial.println();
  delay(3000);

  RunTheta(1, -DK.a, 95);
  RunTheta(2, -DK.b, 95);
  RunTheta(3, -DK.c, 95);
  delay(3000);

  /*unsigned long cMotor1Time = millis();
    unsigned long cMotor2Time = millis();
    digitalWrite(PUL_1, LOW);
    digitalWrite(PUL_2, LOW);

    if (cMotor1Time - pMotor1Time < motor1Speed)
    {
     digitalWrite(PUL_1, HIGH);
     pMotor1Time = cMotor1Time;
    }

    if (cMotor2Time - pMotor2Time < motor2Speed)
    {
     digitalWrite(PUL_2, HIGH);
     pMotor2Time = cMotor2Time;
    }*/
}
