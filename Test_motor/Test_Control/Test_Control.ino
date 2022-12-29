#include <DeltaKinematics.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 7, 6); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 10, 9);
AccelStepper stepper3(1, 13, 12);

DeltaKinematics DK(256, 430, 45, 115);


int PUL_1 = 7; //define Pulse pin
int DIR_1 = 6; //define Direction pin
int ENA_1 = 5; //define Enable Pin

int PUL_2 = 10; //define Pulse pin
int DIR_2 = 9; //define Direction pin
int ENA_2 = 8; //define Enable Pin

int PUL_3 = 13; //define Pulse pin
int DIR_3 = 12; //define Direction pin
int ENA_3 = 11; //define Enable Pin

MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3];

unsigned long pMotor1Time = millis();
unsigned long pMotor2Time = millis();
const unsigned int MAX_MESSAGE_LENGTH = 12;

int NC = 32;
float angle = 1.8;
float ratio = 3969 / 289; //TI SO TRUYEN 3969/289

float homeTheta1 = 0;
float homeTheta2 = 0;
float homeTheta3 = 0;

float currentTheta1 = 0;
float currentTheta2 = 0;
float currentTheta3 = 0;

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

  stepper1.setMaxSpeed(100000.0);
  stepper1.setAcceleration(5000.0);

  stepper2.setMaxSpeed(100000.0);
  stepper2.setAcceleration(5000.0);

  stepper3.setMaxSpeed(100000.0);
  stepper3.setAcceleration(5000.0);

  // Adding the 3 steppers to the steppersControl instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);
}

int ThetaToStep(float theta)
{
  int stp;
  if (theta < 0) {
    digitalWrite(DIR_1, LOW);
    digitalWrite(DIR_2, LOW);
    digitalWrite(DIR_3, LOW);
    theta = -theta;
  }
  else {
    digitalWrite(DIR_1, HIGH);
    digitalWrite(DIR_2, HIGH);
    digitalWrite(DIR_3, HIGH);
  }
  stp = (theta / (angle / NC)) * ratio;
  return stp;
}

void OneStep (int motor)
{
  switch (motor) {
    case 1:
      digitalWrite(PUL_1, HIGH);
      delayMicroseconds(20);
      digitalWrite(PUL_1, LOW);
      delayMicroseconds(20);
      break;
    case 2:
      digitalWrite(PUL_2, HIGH);
      delayMicroseconds(20);
      digitalWrite(PUL_2, LOW);
      delayMicroseconds(20);
      break;
    case 3:
      digitalWrite(PUL_3, HIGH);
      delayMicroseconds(20);
      digitalWrite(PUL_3, LOW);
      delayMicroseconds(20);
      break;
  }
}

void RunMotor(float theta1, float theta2, float theta3)
{
  //long runmotor[3];
  int stp1 = ThetaToStep(theta1);
  int stp2 = ThetaToStep(theta2);
  int stp3 = ThetaToStep(theta3);

  int cstp1 = 0;
  int cstp2 = 0;
  int cstp3 = 0;
  while (cstp1 != stp1 || cstp2 != stp2 || cstp3 != stp3)
  {
    if (cstp1 != stp1) {
      OneStep(1);
      cstp1++;
    }
    else digitalWrite(ENA_1, LOW);

    if (cstp2 != stp2) {
      OneStep(2);
      cstp2++;
    }
    else digitalWrite(ENA_2, LOW);

    if (cstp3 != stp3) {
      OneStep(3);
      cstp3++;
    }
    else digitalWrite(ENA_3, LOW);
  }
  digitalWrite(ENA_1, HIGH);
  digitalWrite(ENA_2, HIGH);
  digitalWrite(ENA_3, HIGH);
}

void Goto (float X, float Y, float Z)
{
  DK.inverse(X, Y, Z); // position
  //DK.forward(20, 20, 20); // angle
  //Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
  Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
  Serial.println();
  currentTheta1 = currentTheta1 + DK.a;
  currentTheta2 = currentTheta1 + DK.b;
  currentTheta3 = currentTheta1 + DK.c;
}
void loop() {
  if (Serial.available() > 0) {
    //Create a place to hold the incoming message
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();

    //Message coming in (check not terminating character) and guard for over message size
    if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
    {
      //Add the incoming byte to our message
      message[message_pos] = inByte;
      message_pos++;
    }
    else
    {
      //Add null character to string
      message[message_pos] = '\0';

      //Print the message (or do other things)
      Serial.println(message);

      //Or convert to integer and print
      int number = atoi(message);
      Serial.println(number);

      if (number == 1)
      {
        DK.inverse(0, 0, -200); // position
        Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
        Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
        Serial.println();
        RunMotor(DK.a, DK.b, DK.c);
      }
      if (number == 2)
      {
        RunMotor(-DK.a, -DK.b, -DK.c);
      }

      if (number == 3)
      {
        gotoposition[0] = 0;
        gotoposition[1] = 0;
        gotoposition[2] = 0;

        steppersControl.moveTo(gotoposition);
        steppersControl.runSpeedToPosition();
      }

      if (number == 4)
      {
        DK.inverse(70, 0, 359.6); // position
        Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
        Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
        Serial.println();
        gotoposition[0] = ThetaToStep(DK.a);  // 800 steps - full rotation with quater-step resolution
        gotoposition[1] = ThetaToStep(DK.b);
        gotoposition[2] = ThetaToStep(DK.c);

        steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
        steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position

        delay(4000);

      }

      //Reset for the next message
      message_pos = 0;
    }
  }

  //  DK.inverse(0, 0, 200); // position
  //  //DK.forward(20, 20, 20); // angle
  //  Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
  //  Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
  //  Serial.println();
  //  RunMotor(DK.a, DK.b, DK.c);
  //  delay(3000);
  //
  //  RunMotor(-DK.a, -DK.b, -DK.c);
  //  delay(3000);

  //  DK.inverse(0, 0, 100); // position
  //  //DK.forward(20, 20, 20); // angle
  //  Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
  //  Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
  //  Serial.println();
  //  delay(3000);
}
