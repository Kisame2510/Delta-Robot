#include <DeltaKinematics.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define the stepper motor and the pins that is connected to
// (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper1(1, 7, 6);  
AccelStepper stepper2(1, 10, 9);
AccelStepper stepper3(1, 13, 12);

DeltaKinematics DK(265, 425, 40, 115);


int PUL_1 = 7;  //define Pulse pin
int DIR_1 = 6;  //define Direction pin
int ENA_1 = 5;  //define Enable Pin


int PUL_3 = 10;  //define Pulse pin
int DIR_3 = 9;   //define Direction pin
int ENA_3 = 8;   //define Enable Pin

int PUL_2 = 13;  //define Pulse pin
int DIR_2 = 12;  //define Direction pin
int ENA_2 = 11;  //define Enable Pin

int LS1 = 53;  // LIMITS SWITCH
int LS2 = 51;
int LS3 = 49;

int Vacum = 45;
int Conv = 43;

MultiStepper steppersControl;  // Create instance of MultiStepper

long gotoposition[3];

int NC = 32;
float angle = 1.8;
float ratio = 3969 / 289;  //TI SO TRUYEN 3969/289
double dTheta1 = 40;
double dTheta2 = 40;
double dTheta3 = 40;
///////////////////////////////////////////
float redX = 0;
float redY = 160;
float redZ = -590;

float blueX = 110;
float blueY = 160;
float blueZ = -590;

float yellowX = -110;
float yellowY = 160;
float yellowZ = -590;
///////////////////////////////////////////
double TargetX = 0;
double TargetY = 0;
double TargetZ = 0;
///////////////////////////////////////////
int SOneLimitState = 0;
int STwoLimitState = 0;
int SThreeLimitState = 0;

int SOneLimitReached = false;
int STwoLimitReached = false;
int SThreeLimitReached = false;

int RobotHomed = false;
int S1Homed = false;
int S2Homed = false;
int S3Homed = false;

double S1Position = 0;
double S2Position = 0;
double S3Position = 0;

unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long time3 = 0;
unsigned long time4 = 0;

int n = 0;
int n1 = 0;
int n2 = 0;
int n3 = 0;

int send = 0;
int runcv = 0;
int ConvSpeed;

void setup() {
  Serial.begin(38400);
  Serial1.begin(38400);

  pinMode(PUL_1, OUTPUT);
  pinMode(DIR_1, OUTPUT);
  pinMode(ENA_1, OUTPUT);

  pinMode(PUL_2, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(ENA_2, OUTPUT);

  pinMode(PUL_3, OUTPUT);
  pinMode(DIR_3, OUTPUT);
  pinMode(ENA_3, OUTPUT);

  pinMode(Vacum, OUTPUT);
  pinMode(Conv, OUTPUT);

  pinMode(LS1, INPUT);
  pinMode(LS2, INPUT);
  pinMode(LS3, INPUT);

  digitalWrite(DIR_1, HIGH);
  digitalWrite(ENA_1, HIGH);

  digitalWrite(DIR_2, HIGH);
  digitalWrite(ENA_2, HIGH);

  digitalWrite(DIR_3, HIGH);
  digitalWrite(ENA_3, HIGH);

  stepper1.setMaxSpeed(10000.0);
  stepper1.setAcceleration(100.0);

  stepper2.setMaxSpeed(10000.0);
  stepper2.setAcceleration(100.0);

  stepper3.setMaxSpeed(10000.0);
  stepper3.setAcceleration(100.0);

  //Instance for multi stepper control
  steppersControl.addStepper(stepper1);
  steppersControl.addStepper(stepper2);
  steppersControl.addStepper(stepper3);
  digitalWrite(Conv, LOW);
}
void SetMaxSpeed(int Speed, int Acc) {
  stepper1.setMaxSpeed(Speed * 100);
  stepper1.setAcceleration(Acc);

  stepper2.setMaxSpeed(Speed * 100);
  stepper2.setAcceleration(Acc);

  stepper3.setMaxSpeed(Speed * 100);
  stepper3.setAcceleration(Acc);
}

int ThetaToStep(float theta) {
  int stp;
  stp = -(theta / (angle / NC)) * ratio;
  return stp;
}

void RunTheta(float theta1, float theta2, float theta3) {
  dTheta1 = theta1;
  dTheta2 = theta2;
  dTheta3 = theta3;
  SendMes();
  gotoposition[0] = ThetaToStep(theta1);
  gotoposition[1] = ThetaToStep(theta2);
  gotoposition[2] = ThetaToStep(theta3);
  steppersControl.moveTo(gotoposition);
  steppersControl.runSpeedToPosition();
}
void Safety() {
  if (SOneLimitState == LOW && stepper1.distanceToGo() >= 0) {
    stepper1.stop();
  }
  if (STwoLimitState == LOW && stepper2.distanceToGo() >= 0) {
    stepper2.stop();
  }
  if (SThreeLimitState == LOW && stepper3.distanceToGo() >= 0) {
    stepper3.stop();
  }
}
void PickPP(float X1, float Y1, float Z1, float X2, float Y2, float Z2)
{
  if (Y1 >= 120) Z1 = Z1 + 30;
  SetMaxSpeed(100, 60);

  DK.inverse(X1, Y1, Z1 + 50); 
  RunTheta(DK.a, DK.b, DK.c);
  digitalWrite(Vacum, HIGH);

  DK.inverse(X1, Y1, Z1);  
  RunTheta(DK.a, DK.b, DK.c);
  DK.inverse(X1, Y1, Z1 + 50);  
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z2 + 30); 
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z2);  
  RunTheta(DK.a, DK.b, DK.c);
  digitalWrite(Vacum, LOW);
  RunTheta(40, 40, 40);
}

void DrawLine(float X1, float Y1, float Z, float X2, float Y2) {
  SetMaxSpeed(20, 20);

  DK.inverse(X1, Y1, Z);  // position
  RunTheta(DK.a, DK.b, DK.c);

  DK.inverse(X2, Y2, Z);  // position
  RunTheta(DK.a, DK.b, DK.c);
}

void DrawCircle(float X, float Y, float Z, float R, float S_angle, float T_angle, bool dir) {
  SetMaxSpeed(20, 20);
  float px;
  float py;
  if (dir == true)  // CCW
  {
    for (float angle = S_angle; angle <= S_angle + T_angle; angle++) {
      px = X + R * cos(angle / 360 * 2 * PI);
      py = Y + R * sin(angle / 360 * 2 * PI);
      if (angle == S_angle) {
        DK.inverse(px, py, Z - 20);
        RunTheta(DK.a, DK.b, DK.c);
      }
      DK.inverse(px, py, Z);
      RunTheta(DK.a, DK.b, DK.c);
    }
    DK.inverse(px, py, Z - 20);
    RunTheta(DK.a, DK.b, DK.c);
  } else  //CW
  {
    for (float angle = S_angle; angle <= S_angle - T_angle; angle--) {
      px = X + R * cos(angle / 360 * 2 * PI);
      py = Y + R * sin(angle / 360 * 2 * PI);
      if (angle == S_angle) {
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

void TestSS() {
  int S2State = digitalRead(LS2);
  int S3State = digitalRead(LS3);
  int S1State = digitalRead(LS1);
  if (S1State == LOW) {
    RunTheta(10, 0, 0);
    delay(300);
    RunTheta(0, 0, 0);
  }
  if (S2State == LOW) {
    RunTheta(0, 10, 0);
    delay(300);
    RunTheta(0, 0, 0);
  }
  if (S3State == LOW) {
    RunTheta(0, 0, 10);
    delay(300);
    RunTheta(0, 0, 0);
  }
}

String splitString(String str, String delim, uint16_t pos) {
  String tmp = str;
  for (int i = 0; i < pos; i++) {
    tmp = tmp.substring(tmp.indexOf(delim) + 1);
    if (tmp.indexOf(delim) == -1
        && i != pos - 1)
      return "";
  }
  return tmp.substring(0, tmp.indexOf(delim));
}

void Command(String Com) {
  if (Com == "GOHOME") {
    RunTheta(40, 40, 40);
  }
  if (Com == "SETHOME") {
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
  }
  if (Com == "DOWN") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x, DK.y, DK.z - 20);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "UP") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x, DK.y, DK.z + 20);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "NORTH") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x + 20, DK.y, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "SOUTH") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x - 20, DK.y, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "EAST") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x, DK.y + 20, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "WEST") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x, DK.y - 20, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "NORTHEAST") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x + 20, DK.y + 20, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "NORTHWEST") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x + 20, DK.y - 20, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "SOUTHEAST") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x - 20, DK.y + 20, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (Com == "SOUTHWEST") {
    SetMaxSpeed(30, 50);
    DK.forward(dTheta1, dTheta2, dTheta3);
    DK.inverse(DK.x - 20, DK.y - 20, DK.z);
    RunTheta(DK.a, DK.b, DK.c);
  }
  //////////////// SIGN TARGET COLOR //////////////////
  if (Com == "SIGNRED") {
    DK.forward(dTheta1, dTheta2, dTheta3);
    redX = DK.x;
    redY = DK.y;
    redZ = DK.z;
  }
  if (Com == "SIGNBLUE") {
    DK.forward(dTheta1, dTheta2, dTheta3);
    blueX = DK.x;
    blueY = DK.y;
    blueZ = DK.z;
  }
  if (Com == "SIGNYELLOW") {
    DK.forward(dTheta1, dTheta2, dTheta3);
    yellowX = DK.x;
    yellowY = DK.y;
    yellowZ = DK.z;
  }
  /////////////////////// RUN TARGET //////////////
  if (splitString(Com, " ", 0) == "RUNT") {
    TargetX = splitString(Com, " ", 1).toDouble();
    TargetY = splitString(Com, " ", 2).toDouble();
    TargetZ = splitString(Com, " ", 3).toDouble();
    DK.inverse(TargetX, TargetY, TargetZ);
    RunTheta(DK.a, DK.b, DK.c);
  }
  if (splitString(Com, " ", 0) == "RED") {
    double redT_X = splitString(Com, " ", 2).toDouble();
    double redT_Y = splitString(Com, " ", 1).toDouble();
    double redT_Z = splitString(Com, " ", 3).toDouble();
    PickPP(redT_X, redT_Y, redT_Z, redX, redY, redZ);
  }
  if (splitString(Com, " ", 0) == "YELLOW") {
    double yellowT_X = splitString(Com, " ", 2).toDouble();
    double yellowT_Y = splitString(Com, " ", 1).toDouble();
    double yellowT_Z = splitString(Com, " ", 3).toDouble();
    PickPP(yellowT_X, yellowT_Y, yellowT_Z, yellowX, yellowY, yellowZ);
  }
  if (splitString(Com, " ", 0) == "BLUE") {
    double blueT_X = splitString(Com, " ", 2).toDouble();
    double blueT_Y = splitString(Com, " ", 1).toDouble();
    double blueT_Z = splitString(Com, " ", 3).toDouble();
    PickPP(blueT_X, blueT_Y, blueT_Z, blueX, blueY, blueZ);
  }
  //////////////////////////////////////
  if (Com == "VC") {
    digitalWrite(Vacum, HIGH);
  }
  if (Com == "VCC") {
    digitalWrite(Vacum, LOW);
  }
  if (Com == "CV") {
    digitalWrite(Conv, HIGH);
  }
  if (Com == "CCV") {
    digitalWrite(Conv, LOW);
  }
  if (Com == "STOP") {
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    digitalWrite(Conv, LOW);
  }
    if (Com == "SETUP") {
    unsigned long starttime = millis();
    unsigned long endtime = starttime;
    while ((unsigned long)(endtime - starttime) <= 6000) {
      Home();
      endtime = millis();
    }
    RobotHomed = false;
  }
  if (Com == "SENDMES") {
    send = 1;
  }
  if (splitString(Com, " ", 0) == "RUNCV") {
    runcv = 1;
    String sConvSpeed = splitString(Com, " ", 1);
    ConvSpeed = splitString(Com, " ", 1).toInt();
    Serial1.print("ON " + sConvSpeed);
  }
}
//////////////////////////////////MES/////////////////////////////
void GetMes() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\r');
    Command(msg);
  }
}
void SendMes() {
  if (send == 1) {
    DK.forward(dTheta1, dTheta2, dTheta3);
    Serial.println(String(DK.x) + " " 
                  + String(DK.y) + " " 
                  + String(DK.z)+ " " 
                  + String(dTheta1) + " " 
                  + String(dTheta2) + " " 
                  + String(dTheta3));
  }
}

void Home() {
  SetMaxSpeed(80, 50);
  //read S1 limit switch
  SOneLimitState = digitalRead(LS1);
  //find M1 limit
  if (SOneLimitState == HIGH && RobotHomed == false
      && SOneLimitReached == false 
      && STwoLimitReached == false 
      && SThreeLimitReached == false) {
    //move 1 degree at a time
    RunTheta(-n1, 0, 0);
    n1 = n1 + 1;
  } else if (SOneLimitState == LOW) {

    S1Position = 0;
    stepper1.setCurrentPosition(0);

    if (stepper1.distanceToGo() <= 0) {
      SOneLimitReached = true;
      n1 = 0;
    }
  }
  //read S1 limit switch
  STwoLimitState = digitalRead(LS2);
  //find M2 limit
  if (STwoLimitState == HIGH && RobotHomed == false
      && SOneLimitReached == true 
      && STwoLimitReached == false 
      && SThreeLimitReached == false) {
    //move 1 degree at a time
    ////////////
    SetMaxSpeed(70, 50);
    RunTheta(0, -n2, 0);
    n2 = n2 + 1;
  } else if (STwoLimitState == LOW) {
    S2Position = 0;
    stepper2.setCurrentPosition(0);
    if (stepper2.distanceToGo() <= 0) {
      STwoLimitReached = true;
      n2 = 0;
    }
  }

  //read S1 limit switch
  SThreeLimitState = digitalRead(LS3);

  //find M3 limit
  if (SThreeLimitState == HIGH && RobotHomed == false
      && SOneLimitReached == true 
      && STwoLimitReached == true 
      && SThreeLimitReached == false) {
    //move 1 degree at a time
    SetMaxSpeed(70, 50);
    RunTheta(0, 0, -n3);
    n3 = n3 + 1;
  } else if (SThreeLimitState == LOW) {
    S3Position = 0;
    stepper3.setCurrentPosition(ThetaToStep(0));
    if (stepper3.distanceToGo() <= 0) {
      SThreeLimitReached = true;
      RobotHomed = true;
      S1Homed = false;
      S2Homed = false;
      S3Homed = false;
      n3 = 0;
      SetMaxSpeed(60, 50);
      RunTheta(5, 5, 5);
      RunTheta(10, 10, 10);
      RunTheta(20, 20, 20);
      RunTheta(25, 25, 25);
      RunTheta(30, 30, 30);
      RunTheta(40, 40, 40);

      SOneLimitReached = false;
      STwoLimitReached = false;
      SThreeLimitReached = false;
    }
  }
}

void loop() {
  GetMes();
  if ((unsigned long)(millis() - time2) > 50) {
    SendMes();
    time2 = millis();
  }
  //TestSS();
  Safety();
  /////////////////////////////////////////////////////////
}