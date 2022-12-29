//#define ENCA PB4
//#define ENCB PB6
//#define U PB10
//#define IN1 PB7
//#define IN2 PB11

#define ENCA 2
#define ENCB 3
#define U 5
#define IN1 10
#define IN2 11

// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float eintegral = 0;
float derivative = 0;

void setup() {
  Serial.begin(19200);
  while (!Serial);
  TCCR1A = 0;
  TCCR1B = 0B111;

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(U, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  //16 MHz
}
void PID_cal()
{
  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1 / 600 * 60; //RPM, T=0.01, N=6000
  // Convert pos form encoder
  int pos_r = pos_i / 3.6;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;

  /////////////////Set a target//////////////////////////

  float vt = 20; //~max145(95% u) //77 rpm
  float post = 500;


  /////////////Compute the control signal u///////////////////////
  float kp = 2.839; //1.743
  float ki = 7.901; //7.49
  float kd = 0.0000; //0.0000005
  float u = 0;
  float u_p = 0;

  float e = vt - v1Filt;
  static float e_p = 0;

  // anti_windup
  eintegral = eintegral + e * deltaT + 0.1896 * deltaT * (u - u_p) / ki;
  //kb=cp*w=0.24*0.79

  derivative =  (e - e_p) / deltaT;
  e_p = e;


  //PWM
  u = kp * e + ki * eintegral + kd * derivative;
  u_p = u;

  
}
void loop() {

  PID_cal();
  // Set the motor speed and direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if (pwr >= 255) {
    pwr = 255;
  }
  if (pwr <= -255) {
    pwr = -255;
  }
  setMotor(dir, pwr, U, IN1, IN2);


  //int setVelLow(pos_r, pos) {
  // hilim = 10; lolim = 0;
  // float ev = pos - pos_r;
  //  if (ev < 0) {
  //   ev = -ev;
  //   dir = -1;
  //  }
  // else dir = 1;
  //  u = kp * ev;
  // if (u > hilim)u = hilim;
  //  else if (u < lolim) u = lolim;
  // return u;
  //}

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(U, pwmVal); // Motor speed
  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    // Or dont turn
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    // If B is high, increment forward
    increment = 1;
  }
  else {
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

}
