//teensy 4.0 board
//motor driver HR8833
//motor pololu geared DC motor
//encoder pololu incremental encoder


#include <TimerOne.h>
int t_l = 10000; //control loop time [micro seconds]

const int MA = 5; //Motor Input
const int MB = 6;
const int EnA = 12; //Motor Encoder
const int EnB = 11;

unsigned long cnt = 0;
unsigned long cnt_p = 0;
int cnt_d = 0;

double cnt_v = 0;
double cnt_v_p = 0;
double cnt_v_ref = 0;
double kp = 0.25*1*t_l/1000;
double kd = 0.25*4*t_l/1000;

int pwm_m = 0;
int pwm_m_d = 0;
int maxd = 50;

void setup() {
  Serial.begin(115200);
  
  Timer1.initialize(t_l);
  Timer1.attachInterrupt(timerIsr);

  pinMode(MA,OUTPUT);
  pinMode(MB,OUTPUT);
  analogWriteResolution(10);
  analogWriteFrequency(MB, 146484);
  
  pinMode(EnA,INPUT);
  pinMode(EnA,INPUT);
  attachInterrupt(digitalPinToInterrupt(EnA),EnFunA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EnB),EnFunB,CHANGE);
}

void timerIsr() {
  cnt_d = cnt - cnt_p;
  cnt_v = cnt_d*1000000/t_l;

  pwm_m_d = (cnt_v_ref - cnt_v)*kp + (cnt_v_p - cnt_v)*kd;
  if(pwm_m_d > maxd){pwm_m_d = maxd;}
  else if(pwm_m_d < -maxd){pwm_m_d = -maxd;}
  
  pwm_m += pwm_m_d;
  if(pwm_m > 1023){pwm_m = 1023;}
  else if(pwm_m < 0){pwm_m = 0;}

  digitalWrite(MA,0);
  analogWrite(MB,pwm_m);
  
  Serial.print(cnt_v_ref);
  Serial.print(",");
  Serial.print(cnt_d);
  Serial.print(",");
  Serial.print(cnt_v);
  Serial.print(",");
  Serial.println(pwm_m);

  cnt_p = cnt;
  cnt_v_p = cnt_v;
}

void loop() {
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case '1':
        cnt_v_ref += 500;
        break;
      case '2':
        cnt_v_ref -= 500;
        break;
    }
  }
  cnt_v_ref = 500;
  delay(500);
  cnt_v_ref = 1000;
  delay(500);
  cnt_v_ref = 1500;
  delay(500);
  cnt_v_ref = 2000;
  delay(500);
  cnt_v_ref = 1500;
  delay(500);
  cnt_v_ref = 1000;
  delay(500);
  cnt_v_ref = 500;
  delay(500);
  cnt_v_ref = 2000;
  delay(1000);
  cnt_v_ref = 500;
  delay(1000);
}

void EnFunA(){
  if(digitalRead(EnA) == 0){
    if(digitalRead(EnB) == 0){
      cnt--;
    }
    else{
      cnt++;
    }
  }
  else{
    if(digitalRead(EnB) == 0){
      cnt++;
    }
    else{
      cnt--;
    }
  }
}
void EnFunB(){
  if(digitalRead(EnB) == 0){
    if(digitalRead(EnA) == 0){
      cnt++;
    }
    else{
      cnt--;
    }
  }
  else{
    if(digitalRead(EnA) == 0){
      cnt--;
    }
    else{
      cnt++;
    }
  }
}
