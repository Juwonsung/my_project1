#define IR_FL A0        //IR_FrontLeft
#define IR_FR A1        //IR_FrontRight
#define IR_BL A2        //IR_BackLeft
#define IR_BR A3        //IR_BackRight
#define MODE_CHANGE 5

#define DIR_FL 8        //Direction
#define PWM_FL 10       //PWM Value (Uno pwm pin : 3,5,6,9,10,11)(5,6 : 980Hz, others : 490Hz)
#define DIR_FR 7
#define PWM_FR 9
#define DIR_BL 4
#define PWM_BL 3
#define DIR_BR 12
#define PWM_BR 11

#define SPEED_MAX 255
#define SPEED_MIN 0
#define SPEED_MID 150

int  state;
float SFL, SFR, SBL, SBR, DFL, DFR, DBL, DBR;
int past_state;
unsigned long past = 0;

void setup() {
 
  // put your setup code here, to run once:
  Serial.begin(9600);
    pinMode(DIR_FL, OUTPUT);         //pinMode setup
    pinMode(PWM_FL, OUTPUT);
    pinMode(DIR_FR, OUTPUT);
    pinMode(PWM_FR, OUTPUT);
    pinMode(DIR_BL, OUTPUT);        
    pinMode(PWM_BL, OUTPUT);
    pinMode(DIR_BR, OUTPUT);
    pinMode(PWM_BR, OUTPUT);
    pinMode(MODE_CHANGE ,INPUT_PULLUP);

      if(digitalRead(MODE_CHANGE) == HIGH){//1,2
    Serial.println("Switch is ON");
    TurnLeft();
    delay(1078);
    GoForward(SPEED_MAX);
    delay(3000);
  }
  else if(digitalRead(MODE_CHANGE) == LOW ){//3
    Serial.print("Switch is OFF");
    GoForward(SPEED_MAX);
    delay(3000);
  }
     
}

void loop() {
 
 
  //S : sensorvalue, D : distance//
    SFL = analogRead(IR_FL);
    DFL = 10650.08*pow(SFL,-0.935)-10;//앞왼거리
    SFR = analogRead(IR_FR);
    DFR = 10650.08*pow(SFR,-0.935)-10;//앞오른거리
    SBL = analogRead(IR_BL);
    DBL = 10650.08*pow(SBL,-0.935)-10;//뒤왼거리
    SBR = analogRead(IR_BR);
    DBR = 10650.08*pow(SBR,-0.935)-10;//뒤오거리

  //avr = (cm1+cm2)/2;
 /* Serial.print("Distance1(cm)");
  Serial.println(DFL);
  Serial.print("Distance2(cm)");
  Serial.println(DFR);
  Serial.print("Distance3(cm)");
  Serial.println(DBL);
  Serial.print("Distance4(cm)");
  Serial.println(DBR);
  delay(1000);
  */
 
 state = statesetting();

     //Serial.println(state);

   //put your main code here, to run repeatedly
    //TurnRight();
   
  unsigned long now = 0;
        Serial.println("past_state: ");
       Serial.println(past_state);
    delay(500);
   
    if(state == 100)
    {
       now = millis();
       if(state != past_state){
        Serial.println("new_state");
          if (now - past <= 5000){    
            GoForward(SPEED_MAX);
            past = now;
          }
       }
      else if(state == past_state){
        Serial.println("past_state == state");
        Serial.println(state);
        keep_Go(SPEED_MAX);
        }  
      }
   
    else if(state == 200){
    now = millis();
    if (now - past <= 5000)
    TurnLeft();
          past = now;
    }
   
    else if(state ==300){
    now = millis();
    if (now-past <= 5000)
      TurnRight();
            past = now;
    }
   
    else if(state == 400)
    {
      now = millis();
        if(state != past_state){
        Serial.println("new_state");
          if (now - past <= 5000){    
            GoBackward(SPEED_MAX);
            past = now;
          }
       }
      else if(state == past_state){
        Serial.println("past_state == state");
        Serial.println(state);
        Keep_Back(SPEED_MAX);
        }
    }
   
    else if(state == 500){ //물체를 찾아야 하는 상황
    now = millis();
    if(now - past <= 5000){
      Search();
      past =now;
    }
      }

      else if(state == 501){ //이상하게 인식이 된 상황  
    now = millis();
    if(now - past <= 5000){
      Search_P();
      past =now;
    }
      }
   
    else {
    now = millis();
    if(now - past <= 5000){
    Search();
          past = now;
    }
   }
 
   past_state = state;
   return  past_state;
}

void GoForward(int velocity)  {
    for(int i=0; i<velocity; i++){
      digitalWrite(DIR_FL,LOW);
      analogWrite(PWM_FL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,LOW);
      analogWrite(PWM_FR,i);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,LOW);
      analogWrite(PWM_BL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,LOW);
      analogWrite(PWM_BR,i);  // write into PWM value 0~255（speed)
      delay(10);
      if(i > 255){
        i =255;
        }
    }
}

void keep_Go(int velocity)  {
      digitalWrite(DIR_FL,LOW);
      analogWrite(PWM_FL,velocity);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,LOW);
      analogWrite(PWM_FR,velocity);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,LOW);
      analogWrite(PWM_BL,velocity);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,LOW);
      analogWrite(PWM_BR,velocity);  // write into PWM value 0~255（speed)
      delay(10);
    if(velocity > 255){
        velocity =255;
        }
}

void GoBackward(int velocity)  {
      for(int i=0; i<velocity; i++){
      digitalWrite(DIR_FL,HIGH);
      analogWrite(PWM_FL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,HIGH);
      analogWrite(PWM_FR,i);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,HIGH);
      analogWrite(PWM_BL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,HIGH);
      analogWrite(PWM_BR,i);  // write into PWM value 0~255（speed)
      delay(10);
      if(i > 255){
        i =255;
        }
    }
}

void Keep_Back(int velocity)  {
      digitalWrite(DIR_FL,HIGH);
      analogWrite(PWM_FL,velocity);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,HIGH);
      analogWrite(PWM_FR,velocity);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,HIGH);
      analogWrite(PWM_BL,velocity);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,HIGH);
      analogWrite(PWM_BR,velocity);  // write into PWM value 0~255（speed)
      delay(10);
      if(velocity > 255){
        velocity =255;
        }
     
}

void TurnLeft() {
    for(int i=0; i<SPEED_MID; i++){
      digitalWrite(DIR_FL,HIGH);
      analogWrite(PWM_FL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,LOW);
      analogWrite(PWM_FR,i);  // write into PWM value 0~255（speed）  
      digitalWrite(DIR_BL,HIGH);
      analogWrite(PWM_BL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,LOW);
      analogWrite(PWM_BR,i);  // write into PWM value 0~255（speed）
      delay(25);
    }
}

void TurnRight() {
    for(int i=0; i<SPEED_MID; i++){
      digitalWrite(DIR_FL,LOW);
      analogWrite(PWM_FL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,HIGH);
      analogWrite(PWM_FR,i);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,LOW);
      analogWrite(PWM_BL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,HIGH);
      analogWrite(PWM_BR,i);  // write into PWM value 0~255（speed）
      delay(25);
    }
  }



void Stop() {
    int SPEED_FL = analogRead(PWM_FL);
    int SPEED_FR = analogRead(PWM_FR);  
    int nowvelocity = max(SPEED_FL,SPEED_FR);

    int DIR_L = digitalRead(DIR_FL);
    int DIR_R = digitalRead(DIR_FR);
   
    for(int i=nowvelocity; i=0; i--){
      digitalWrite(DIR_FL,DIR_L);
      analogWrite(PWM_FL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,DIR_R);
      analogWrite(PWM_FR,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,DIR_L);
      analogWrite(PWM_BL,i);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,DIR_R);
      analogWrite(PWM_BR,i);   // write into PWM value 0~255（speed）
      delay(10);
       if(i<20){
          i = 20;        
      }
    }
}

void Search() {
      digitalWrite(DIR_FL,LOW);
      analogWrite(PWM_FL,SPEED_MID);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,HIGH);
      analogWrite(PWM_FR,SPEED_MID);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,LOW);
      analogWrite(PWM_BL,SPEED_MID);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,HIGH);
      analogWrite(PWM_BR,SPEED_MID);  // write into PWM value 0~255（speed）
      delay(25);
  }

  void Search_P() {
      digitalWrite(DIR_FL,LOW);
      analogWrite(PWM_FL,SPEED_MAX);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_FR,HIGH);
      analogWrite(PWM_FR,SPEED_MAX);  // write into PWM value 0~255（speed）
      digitalWrite(DIR_BL,LOW);
      analogWrite(PWM_BL,SPEED_MAX);   // write into PWM value 0~255（speed）
      digitalWrite(DIR_BR,HIGH);
      analogWrite(PWM_BR,SPEED_MAX);  // write into PWM value 0~255（speed）
      delay(25);
  }
 
//void Decrease(int now_vel, int purpose_vel){
//  for(int i=now_vel; i=purpose_vel; i--){
//      digitalWrite(DIR_FL,DIR_L);
//      analogWrite(PWM_FL,i);   // write into PWM value 0~255（speed）
//      digitalWrite(DIR_FR,DIR_R);
//      analogWrite(PWM_FR,i);   // write into PWM value 0~255（speed）
//      digitalWrite(DIR_BL,DIR_L);
//      analogWrite(PWM_BL,i);   // write into PWM value 0~255（speed）
//      digitalWrite(DIR_BR,DIR_R);
//      analogWrite(PWM_BR,i);   // write into PWM value 0~255（speed）
//      delay(25);
//       if(i<purpose_vel){
//          i = purpose_vel;        
//      }
//    }
//  

int statesetting(){
  if(((DFL <= 100) && (DFR <= 100))){
    state= 100;
  }
  else if((DFL <= 70) && (DFR > 70)){
    state= 200;
  }
  else if((DFL > 70) && (DFR <= 70)){
    state = 300;
  }
  else if((DBL <= 100) && (DBR <= 100)){
    state = 400;
  }
  else if((DBL <= 70) && (DBR > 70)){
    state = 300;
  }
  else if((DBL > 70) && (DBR <= 70)){
    state = 200;
  }
  else if(((DFL > 100) && (DFR > 100))||((DBL > 100) && (DBR > 100))){
    state=500;
  }
  else if (DFL <=70 && DBR <=70 || DFL <=70 && DBL <=70 || DFR <=70 && DBL <=70||DFR <=70 && DBR <=70){
    state = 501;
    }
  return state;
}
