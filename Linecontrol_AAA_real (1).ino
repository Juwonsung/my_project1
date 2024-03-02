#include <MsTimer2.h>     //internal timer 2
#include <MPU6050.h>      //MPU6050 library
#include <Wire.h>         //IIC library


 
/********** 모터 드라이버(TB6612)에 관한 핀 **********/
#define LM_POS    6     //좌측 모터에 들어가는 Direction 핀 설정
#define LM_NEG    7     //좌측 모터에 들어가는 Break 핀 설정
#define LM_PWM    9     //좌측 모터에 들어가는 PWM 핀 설정
#define RM_POS   12     //우측 모터에 들어가는 Direction 핀 설정  
#define RM_NEG    8     //우측 모터에 들어가는 Break 핀 설정
#define RM_PWM   10     //우측 모터에 들어가는 PWM 핀 설정

/********** Encoder Interrupt Pins **********/
#define LT_ENCODER_A 3  //external interrupts
#define LT_ENCODER_B 4  
#define RT_ENCODER_A 2  //external interrupts
#define RT_ENCODER_B 5  

/********** MPU6050 **********/
MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     // Define three-axis acceleration, three-axis gyroscope variables

/********** Kalman Filter **********/
float CovAngle = 0.001;  //Covariance of gyroscope noise : Q_angle
float CovBias  = 0.003;  //Covariance of gyroscope drift noise : Q_gyro
float CovMeasure = 0.5;  //Covariance of accelerometer : R_angle
float K0,K1;      //Kalman Gains
float GyroY;      //[deg/s] angular velocity by gyroscope calculation
float GyroBias;   //Gyro drift : q_bias
float Theta;      //[deg] angle from Kalman Filter
float Omega;      //[deg/s] angular velocity from Kalman Filter
float P[2][2] = {{ 1, 0 }, { 0, 1 }};

/********** Left/Right Motor PWM **********/
float RightDuty=0,LeftDuty=0;
/********** Encoder Couters **********/
volatile long RtEncCount = 0; // volatile long type : to ensure the value
volatile long LtEncCount = 0;
int RtEncN = 0;
int LtEncN = 0;
int RtVelPulse,LtVelPulse;

/********** Wheel Position[mm] & Velocity[mm/s] **********/
int PositionLoopCnt;
float Pulse2mm  = 0.65; //펄스 수를 거리[mm]로 변환하는 비율
float VelFilterOld = 0; //[mm/s]
float Position = 0;     //[mm]
float VelFilter;        //[mm/s]

/********** Balancing Control : PD parameters **********/
float ThetaRef = -4.5; //[deg] Theta angle reference (ideally 0 degrees)
double BalKp = -36, BalKd = -0.6;  //Balancing: PD Control
int BalPwm;  //angle output

/********** Position Control : PD parameters **********/
int PositionLoopN = 8;
double PosKp = 0.0,  PosKd = -0.05*PositionLoopN;  //Position : PD Control
double PosRef = 0.0,  VelRef = 0.0;   //Position Reference
double PosPwm;

#define BAUD_RATE 9600  
boolean T2Flag = LOW;
float StrPwm = 0.0;

float  scaling_L;
float  scaling_R;

float cut_L = 0;
float cut_R = 0;
float line_pos = 0;
float Pdly = 0;       //  pos_err saved ( 이전 위치값 저장 )
float pos_err;
float Kp, Kd;

#define IR_ANAL_L A2  // 주행선을 감지하는 왼쪽 IR (Analog 신호)
#define IR_ANAL_R A3  // 주행선을 감지하는 오른쪽 IR (Analog 신호)

int IR_Anal_L_Value;            //좌측 IR센서의 ADC : 0~1023
int IR_Anal_R_Value;            ////float iR_Left_Position = -10.0; //[mm]좌측 iR의 위치

float iR_Left_Max = 939.0;      //좌측 iR 측정값의 최대치
float iR_Left_Min = 222.0;      //좌측 iR 측정값의 최소치
float iR_Right_Max = 974.0;     //우측 iR 측정값의 최대치
float iR_Right_Min = 261.0;     //우측 iR 측정값의 최소치
float iR_Cut_Value = 130.0;     //iR 측정값에서 iR_Cut_Value만큼 빼고
float iR_Left_Position = -10.0; //[mm]좌측 iR의 위치
float iR_Right_Position = 10.0; //[mm]우측 iR의 위치                              

/*********** 제어 주기 ***************/
int Ts_ms = 5;            //[ms] 제어 주기: 5ms
float Ts = Ts_ms/1000.0;  //[s] 제어 주기: 0.005s

//Bluetooth//
int left = 0;//turn left
int right = 0;//turn right
float ThetaRef_init = 0.;
bool TeleMode = false;
int Move_Step = 1.0;
char val;


void setup()
{
  //set the pins of iR Sensors to Analog INPUT
  pinMode(IR_ANAL_L, INPUT);
  pinMode(IR_ANAL_R, INPUT);

  Serial.begin(BAUD_RATE);   //open the serial monitor to set the baud rate to 9600
  delay(500);

  /* set Timer2 Interrupt
   * (note：timer2 will affect the PWM output of Pin3 & Pin11)
   */

  //set the pins of motor to OUTPUT
  pinMode(LM_POS,OUTPUT);
  pinMode(LM_NEG,OUTPUT);
  pinMode(LM_PWM,OUTPUT);
  pinMode(RM_POS,OUTPUT);      
  pinMode(RM_NEG,OUTPUT);
  pinMode(RM_PWM,OUTPUT);

  //assign initial state value
  digitalWrite(LM_POS,1);
  digitalWrite(LM_NEG,0);
  analogWrite (LM_PWM,0);
  digitalWrite(RM_POS,1);
  digitalWrite(RM_NEG,0);
  analogWrite (RM_PWM,0);

  //set the pins of encoders to INPUT
  pinMode(LT_ENCODER_A,INPUT);
  pinMode(LT_ENCODER_B,INPUT);
  pinMode(RT_ENCODER_A,INPUT);
  pinMode(RT_ENCODER_B,INPUT);

  // join I2C bus
  Wire.begin();         //join I2C bus sequence
  Serial.begin(9600);   //open the serial monitor to set the baud rate to 9600
  delay(1500);
  mpu6050.initialize(); //initialize MPU6050
  delay(2);

  /* set Timer2 Interrupt
   * (note：timer2 will affect the PWM output of Pin3 & Pin11)
   */
  //set Interrupt to count encoder pulses
  attachInterrupt(INT1,LtEnc_isr,CHANGE); //Edge: RISING, FALLING, CHANGE
  attachInterrupt(INT0,RtEnc_isr,CHANGE); //Edge: RISING, FALLING, CHANGE

  MsTimer2::set(Ts_ms,ControlLoop); //5ms마다 ControlLoop 실행
  MsTimer2::start();    //start interrupt
 
}

int t0,t1; //Check Sampling Time
void loop()
{
   if(T2Flag == HIGH){
    Serial.print("irLeft:  ");
    Serial.print(IR_Anal_L_Value);
    Serial.print("\t");
    Serial.print("irRight:  ");
    Serial.print(IR_Anal_R_Value);
    Serial.println("");
    T2Flag = LOW;
  }
  //ControlLoop();
}

/********** Pulse Counting Interrupts : use with Pulse Calculation 1번 **********/
void LtEnc_isr()  //Left Encoder
{
  LtEncCount ++;
}
void RtEnc_isr()  //Right Encoder
{
  RtEncCount ++;
}

/********** Pulse Counting Interrupts : use with Pulse Calculation 2번 **********/
//void LtEnc_isr(){          //왼쪽 엔코더 펄스수
//  if(digitalRead(LT_ENCODER_A) == digitalRead(LT_ENCODER_B))  LtEncCount++;
//  else                                                        LtEncCount--;
//}
//
//void RtEnc_isr(){  
//  if(digitalRead(RT_ENCODER_A) == digitalRead(RT_ENCODER_B))  RtEncCount--;
//  else                                                        RtEncCount++;
//}

/********** Pulse Calculation : 1번 **********/
void CountPulse()
{
  LtEncN = LtEncCount;     //Assign the value counted by the code wheel to lz
  RtEncN = RtEncCount;

  LtEncCount = 0;     //Clear the code counter count
  RtEncCount = 0;

  if (LeftDuty < 0)    LtEncN = -LtEncN;
  if (RightDuty < 0)   RtEncN = -RtEncN;

  //enter interrupts per 5ms; pulse number superposes
  RtVelPulse += RtEncN;
  LtVelPulse += LtEncN;
}

/********** Pulse Calculation : 2번 **********/
//void CountPulse()
//{
//  LtEncN = LtEncCount;     //Assign the value counted by the code wheel to lz
//  RtEncN = RtEncCount;
//
//  LtEncCount = 0;     //Clear the code counter count
//  RtEncCount = 0;
//
//  //enter interrupts per 5ms; pulse number superposes
//  RtVelPulse += RtEncN;
//  LtVelPulse += LtEncN;
//}

/********** Timer2 Interrupt : Control Loop **********/
void ControlLoop()
{
  /////////////////add part
  sei();  //Allow global interrupts
  T2Flag = HIGH;
  LineDetection();      //[mm] Line 검출 : -10mm(left) ~ 10mm(right)
  LineStr();
  //////////////////
 
//  t0 = millis();
  sei();  //Allow global interrupts
  CountPulse();        //Pulse superposition subfunction
  GetAngle();
  BalancingControl();         //Balancing PD control
  MotorDuty();

  PositionLoopCnt++;
  if(PositionLoopCnt>=PositionLoopN)     //5*8=40，40ms entering once speed PI algorithm  
  {
    WheelPosition();
    PositionControl();  
    PositionLoopCnt = 0;  //Clear
  }
//  t1 = millis();
}

/********** 축 변환 **********/
void ChangeAxis()
{
  //센서에서 읽은 값을 시리얼모니터로 확인하고 정의한 축과 동일한지 확인
  //가속도 센서 축 변환
  int16_t tmpaxis = ax;
  ax = -ay;
  ay = tmpaxis;
  az = az;
  //자이로 센서 축 변환
  tmpaxis = gx;
  gx = -gy;
  gy = tmpaxis;
  gz = gz;
}

/********** get Theta angle : MPU6050 & Kalman Filter **********/
void GetAngle()
{
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //IIC to get MPU6050 six-axis data
  ChangeAxis();
  float Acc2Angle = atan2(-ax,az)*(180/PI);  //[deg] theta angle from accelerometer
  GyroY = gy/131;                     //[deg/s] Y-axis angular velocity from gyroscope
  Kalman_Filter(Acc2Angle,GyroY);            //Kalman Filtering
}

/********** Kalman Filter **********/
void Kalman_Filter(double acc_angle, double gyro_y)
{
  double y;     //Angle difference between AccAngle and Prior Estimate
  double S;     //Innovation Covariance
 
  //Prediction
  /* Step 1: Prior Estimate */
  Theta += (gyro_y - GyroBias) * Ts;          //prior estimated angle
  /* Step 2: Prior error covariance */
  P[0][0] += Ts * (Ts*P[1][1] - P[0][1] - P[1][0] + CovAngle);
  P[0][1] -= Ts * P[1][1];
  P[1][0] -= Ts * P[1][1];
  P[1][1] += CovBias * Ts;
 
  //Update
  /* Step 3: Innovation : Difference between AccAngle and Prior Estimate */
  y = acc_angle - Theta;
  /* Step 4: Innovation Covariance */
  S = P[0][0] + CovMeasure;
  /* Step 5: Kalman Gain */
  K0 = P[0][0] / S;
  K1 = P[1][0] / S;
  /* Step 6: Posteriori Estimte */
  Theta += K0 * y;
  GyroBias += K1 * y;
  /* Step 7: Posteriori Error Covariance */
  P[0][0] -= K0 * P[0][0];
  P[0][1] -= K0 * P[0][1];
  P[1][0] -= K1 * P[0][0];
  P[1][1] -= K1 * P[0][1];

  Omega = gyro_y - GyroBias;  //로봇이 기울어지는 각속도
}

/********** get Wheel Position[mm] & Velocity[mm/s] **********/
/*  WheelPosition의 Sampling Time : PositionLoopN*Ts */
void WheelPosition()
{
  int average_pulse = (LtVelPulse + RtVelPulse)>>1; //Vehicle velocity pulse value
  RtVelPulse = LtVelPulse = 0;      //Clear
  double velocity = (double)average_pulse*Pulse2mm/(PositionLoopN*Ts);    //[mm/s] Vehicle velocity
  /*first-order filter: velocity에 LPF 적용.
   *            1                  a*Ts
   *  y(k) = -------- * y(k-1) + -------- * u(k)
   *         1 + a*Ts            1 + a*Ts
   *  y : filtered velocity
   *  u : velocity
   *  Ts: sampling time
   *  a : bandwidth(rad/s)
   *  Ex: When Ts = 8*0.005
   *            a = 10.7[rad/s] = 1.7[Hz]
   *      Filter Eq. ---> y(k) = 0.7*y(k-1) + 0.3*u(k)
   */
  VelFilter = VelFilterOld*0.7 + velocity * 0.3;
  VelFilterOld = VelFilter;
  Position += VelFilter*(PositionLoopN*Ts);
}

/********** Balancing : PD Control **********/
void BalancingControl()
{
  BalPwm = BalKp * (ThetaRef - Theta) - BalKd * Omega;
}

/********** Positioning : PD Control **********/
void PositionControl()
{
  PosPwm = PosKp * (PosRef - Position) + PosKd * (VelRef - VelFilter);      //speed loop control PI
}

/********** Motoring : PWM Duty(-255 ~ 255) **********/
void MotorDuty()
{

  //targetPwm = LineStr();
  LeftDuty  = BalPwm + PosPwm - StrPwm;//PWM Duty of Left  Motor
  RightDuty = BalPwm + PosPwm + StrPwm;//PWM Duty of Right Motor

  /*limit PWM value : -255 ~ 255 */
  if(RightDuty > 255)   RightDuty =  255;
  if(RightDuty <-255)   RightDuty = -255;
  if(LeftDuty  > 255)   LeftDuty  =  255;
  if(LeftDuty  <-255)   LeftDuty  = -255;

  /* the inclined angle of balance car is greater than 45°, motor will stop. */
  if(Theta>45 || Theta<-45) RightDuty = LeftDuty = 0;

  /* Supply PWM to Left/Right Motors */
  if(LeftDuty >= 0){    //Left Forward
    digitalWrite(LM_POS,HIGH);
    digitalWrite(LM_NEG,LOW);
    analogWrite(LM_PWM,LeftDuty);
  }
  else{                 //Left Backward
    digitalWrite(LM_POS,LOW);
    digitalWrite(LM_NEG,HIGH);
    analogWrite(LM_PWM,-LeftDuty);
  }
  if(RightDuty >= 0){   //Right Forward
    digitalWrite(RM_POS,HIGH);
    digitalWrite(RM_NEG,LOW);
    analogWrite(RM_PWM,RightDuty);
  }
  else{                 //Right Backward
    digitalWrite(RM_POS,LOW);
    digitalWrite(RM_NEG,HIGH);
    analogWrite(RM_PWM,-RightDuty);
  }
}



void LineDetection()

{
  /* Step 1: IR 센서 값 읽어오기 */
  IR_Anal_L_Value = analogRead(IR_ANAL_L);
  IR_Anal_R_Value = analogRead(IR_ANAL_R);

  /* Step 2: IR 센서 값 Scaling : 0 ~ 1000 */
  scaling_L = (IR_Anal_L_Value-iR_Left_Min)/(iR_Left_Max-iR_Left_Min)*1000;
  scaling_R = (IR_Anal_R_Value-iR_Right_Min)/(iR_Right_Max-iR_Right_Min)*1000;

  /* Step 3: IR 센서 값 Cutting : Cut below iR_Cut_Value */
  cut_L = max(0,scaling_L - iR_Cut_Value);
  cut_R = max(0,scaling_R - iR_Cut_Value);

  /* Step 4: 무게중심법으로 LinePosition 계산 */
  if ((cut_L+cut_R)== 0)
    line_pos = 0;
  else
    line_pos = (cut_L*iR_Left_Position + cut_R*iR_Right_Position)/(cut_L + cut_R);
}

float LineStr(){
//  StrPwm = (-line_pos/(past_pos - line_pos))*StrPwm;
//  past_pos = line_pos;
   //Kp = 0.5;    //안움직임
   //Kp = 5;  //너무 많이 움직임
   Kp = 3;
   Kd = 0;
   pos_err = - line_pos;
   float Pval = pos_err;
   float Dval = Pval - Pdly;    // 오차의 차분 값 ( 현재오차 - 이전오차 )
   if(line_pos != 0){
   Pdly = pos_err;  // 위치오차 저장
   }
   
    StrPwm =  Kp*pos_err + Kd*Dval;      // PD 제어 값
   
   if(abs(Dval) > 9){
    Kp = 7;
    StrPwm = Kp*Pdly + Kd*Dval;
  //왼쪽바퀴 조향해서 다시 돌아가는 코드 필요
   }

  //오른쪽 바퀴 조향해서 다시 돌아가는 코드 필요      
   
}


//float LineStr(){
////  StrPwm = (-line_pos/(past_pos - line_pos))*StrPwm;
////  past_pos = line_pos;
//   //Kp = 0.5;    //안움직임
//   //Kp = 5;  //너무 많이 움직임
//   Kp = 2.5;
//   Kd = 0;
//   pos_err = - line_pos;
//   float Pval = pos_err;
//   float Dval = Pval - Pdly;    // 오차의 차분 값 ( 현재오차 - 이전오차 )
//   
//   if(line_pos != 0){
//   Pdly = pos_err;                 // 위치오차 저장
//   }
//   
//   StrPwm =  Kp*Pval + Kd*Dval  ;      // PD 제어 값
//   
//   if(Dval>9){
//    Kp=5;
//    StrPwm =  Kp*Pval + Kd*Dval  ;
//    }
//
//    else if(Dval<-9){
//    Kp=5;
//    StrPwm =  Kp*Pval + Kd*Dval  ;
//    }
//}
