#include <DeltaKinematics.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 7, 6); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 10, 9);
AccelStepper stepper3(1, 13, 12);

DeltaKinematics DK(265, 425, 40, 115);


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

static char message[MAX_MESSAGE_LENGTH];
int number = atoi(message);
int pnumber = 0;

float Z_W = 368.5;

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

  stepper1.setMaxSpeed(5000.0);
  stepper1.setAcceleration(100.0);

  stepper2.setMaxSpeed(5000.0);
  stepper2.setAcceleration(100.0);

  stepper3.setMaxSpeed(5000.0);
  stepper3.setAcceleration(100.0);

  // Adding the 3 steppers to the steppersControl instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);


  Serial.println("Restarted Program");
}
void SetSpeed(int Speed, int Acc)
{
  stepper1.setMaxSpeed(Speed * 100);
  stepper1.setAcceleration(Acc);

  stepper2.setMaxSpeed(Speed * 100);
  stepper2.setAcceleration(Acc);

  stepper3.setMaxSpeed(Speed * 100);
  stepper3.setAcceleration(Acc);
}

int ThetaToStep(float theta)
{
  int stp;
  stp = (theta / (angle / NC)) * ratio;
  return stp;
}

void RunTheta(float theta1, float theta2, float theta3)
{
  gotoposition[0] = ThetaToStep(theta1);
  gotoposition[1] = ThetaToStep(theta2);
  gotoposition[2] = ThetaToStep(theta3);
  steppersControl.moveTo(gotoposition);
  steppersControl.runSpeedToPosition();
}

void PickPP (float X1, float Y1, float Z1, float X2, float Y2, float Z2)
{
  SetSpeed(40, 50);

  DK.inverse(X1, Y1, Z1 - 20); // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X1, Y1, Z1); // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X1, Y1, Z1 - 20); // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z2 - 20); // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z2); // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z2 - 20); // position
  RunTheta(DK.a, DK.b, DK.c);
  RunTheta(0, 0, 0);

}

void DrawLine (float X1, float Y1, float Z, float X2, float Y2)
{
  SetSpeed(40, 20);

  DK.inverse(X1, Y1, Z); // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z); // position
  RunTheta(DK.a, DK.b, DK.c);
}

void DrawCircle(float X, float Y, float Z, float R, float S_angle, float T_angle, bool dir)
{
  SetSpeed(40, 20);
  float px;
  float py;
  if (dir == true)                 // CCW
  {
    for (float angle = S_angle; angle <= S_angle + T_angle; angle++)
    {
      px = X + R * cos(angle / 360 * 2 * PI);
      py = Y + R * sin(angle / 360 * 2 * PI);
      if (angle == S_angle)
      {
        DK.inverse(px, py, Z - 20);
        RunTheta(DK.a, DK.b, DK.c);
      }
      DK.inverse(px, py, Z);
      RunTheta(DK.a, DK.b, DK.c);
    }
    DK.inverse(px, py, Z - 20);
    RunTheta(DK.a, DK.b, DK.c);
  }
  else                              //CW
  {
    for (float angle =  S_angle; angle <= S_angle - T_angle; angle--)
    {
      px = X + R * cos(angle / 360 * 2 * PI);
      py = Y + R * sin(angle / 360 * 2 * PI);
      if (angle == S_angle)
      {
        DK.inverse(px, py, Z - 20);
        RunTheta(DK.a, DK.b, DK.c);
      }
      DK.inverse(px, py, Z);
      RunTheta(DK.a, DK.b, DK.c);
    }
    DK.inverse(px, py, Z - 20);
    RunTheta(DK.a, DK.b, DK.c);
  }
}
void Command(int Com)
{
  switch (Com) {
    case 1:
      DK.inverse(0, 0, 348.5); // position
      Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
      Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
      Serial.println();
      DK.inverse(50, 0, 348.5); // position
      Serial.println(String(DK.x) + "," + String(DK.y) + "," + String(DK.z));
      Serial.println(String(DK.a) + "," + String(DK.b) + "," + String(DK.c));
      Serial.println();
      Serial.println(String(ThetaToStep(DK.a)) + "," + String(ThetaToStep(DK.b)) + "," + String(ThetaToStep(DK.c)));
      break;
    case 2:
      Serial.println("Go to Home");
      RunTheta(0, 0, 0);
      break;
    case 3:
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      Serial.println("Set Home");
      break;
    case 4:
      Serial.println("Go Down");
      RunTheta(-2, -2, -2);
      break;
    case 5:
      Serial.println("Go Up");
      RunTheta(2, 2, 2);
      break;
    case 6:
      Serial.println("Go X");
      DrawLine(0, 0, 348.5, 1, 0);
      break;
    case 7:
      Serial.println("Go Y");
      DrawLine(0, 0, 348.5, 0, 1);
      break;
    ////////////////////////////////////////////////////////////////
    case 8:
      /// 348.5
      PickPP(0, 0, 368.5, 50, 0, 368.5);
      break;
    case 9:
      Serial.println("DrawingThings....");
      RunTheta(0, 0, 0);
      PickPP(0, 0, Z_W, 0, 50, Z_W);
      PickPP(0, 0, Z_W, -50, 0, Z_W);
      PickPP(0, 0, Z_W, 0, -50, Z_W);
      PickPP(0, 0, Z_W, 50, 0, Z_W);
      Serial.println("...End");
      break;
    case 10:
      DrawCircle(0, 0, Z_W, 50, 0 , 360, true);
      RunTheta(0, 0, 0);
      break;
    case 11:
      DrawLine(50, 0, Z_W, 0, 50);
      DrawLine(0, 50, Z_W, -50, 0);
      DrawLine(-50, 0, Z_W, 0, -50);
      DrawLine(0, -50, Z_W, 50, 0);
      RunTheta(0, 0, 0);
  }
}
void GetMes()
{
  if (Serial.available() > 0) {
    //Create a place to hold the incoming message
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
      number = atoi(message);
      Serial.println(number);
      Command(number);
      //Reset for the next message
      message_pos = 0;

    }
  }
}
void loop() {
  GetMes();
  /////////////////////////////////////////////////////////
}
