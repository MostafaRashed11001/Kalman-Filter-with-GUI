/************* Pins *************/
#define encoderA1  2 
#define encoderB1  5
#define encoderA2  3 
#define encoderB2  4 
/************* Motor *************/
#define motor_in_1 7
#define motor_en_1 6
#define motor_in_2 9
#define motor_en_2 10
/**********Switch Pin**************/
#define switch_1 8
#define switch_2 11
/********Input Velocitie**********/
#define vl 100
#define v2 -100
/************* Encoder *************/
#define encoderPPR 600.0  
double encoder1Count = 0 ; 
double encoder2Count = 0 ; 
double old_encoder1Count = 0 ; 
double old_encoder2Count = 0 ; 
/************* Time *************/
#define readRate  50 
unsigned long lastTime =0 ; 
/************* velocities *************/
#define radius 0.035 
double pulse_rate1=0.0 ,pulse_rate2=0.0,rps1=0.0 ,rps2=0.0 ,rpm1=0.0 ,rpm2 = 0.0 ; 
double meterPerSec1=0.0 ,meterPerSec2=0.0;
/************* Distance *************/
double Dis_1=0.0 ,Dis_2=0.0;
////////////////////////////////
void countEncoder1(void) ; 
void countEncoder2(void) ; 
/***************Setup******************/
void setup() 
{
  pinMode(encoderA1,INPUT_PULLUP) ; 
  pinMode(encoderB1,INPUT_PULLUP) ; 
  pinMode(encoderA2,INPUT_PULLUP) ; 
  pinMode(encoderB2,INPUT_PULLUP) ; 
  pinMode(motor_in_1,OUTPUT) ; 
  pinMode(motor_en_1,OUTPUT) ; 
  pinMode(switch_1,OUTPUT) ; 
  pinMode(motor_in_2,OUTPUT) ; 
  pinMode(motor_en_2,OUTPUT) ; 
  pinMode(switch_2,OUTPUT) ; 
  attachInterrupt(digitalPinToInterrupt(encoderA1),&countEncoder1,RISING) ; 
  attachInterrupt(digitalPinToInterrupt(encoderA2),&countEncoder2,RISING) ; 

  Serial.begin(9600) ;  #115200 
}

void loop() 
{
  /* get current time */
  unsigned long currentTime = millis() ; 
  double delta = currentTime-lastTime ;
  if(delta>=readRate)
  {
    pulse_rate1 = ((encoder1Count)*1000)/delta ;  
    pulse_rate2 = ((encoder2Count)*1000)/delta ;  
    rps1 = pulse_rate1 /encoderPPR ;  
    rps2 = pulse_rate2 /encoderPPR ; 
    //rpm1 = rps1*60.0 ; 
    //rpm2 = rps2*60.0 ;
    meterPerSec1= 2*PI*rps1*radius;
    meterPerSec2= 2*PI*rps2*radius;
    Dis_1+=(meterPerSec1*delta)/1000;
    Dis_2+=(meterPerSec2*delta)/1000;
    encoder1Count=0;
    encoder2Count=0;
    lastTime = currentTime ; 
  }
  if(digitalRead(switch_1)==1)
  {
    digitalWrite(motor_in_1,1); 
    analogWrite (motor_en_1,vl); 
  }
  else if(digitalRead(switch_2)==1)
  {
    digitalWrite(motor_in_1,0); 
    analogWrite (motor_en_1,v2);
  }
  else
  {
    digitalWrite(motor_in_1,0); 
    analogWrite (motor_en_1,0);
  }
  //Serial.print(Dis_1) ;
  //Serial.print("\t") ; 
  Serial.println(Dis_2) ; 
}


void countEncoder1(void) 
{
  if(digitalRead(encoderB1)==digitalRead(encoderA1))
  {
       encoder1Count++ ; 
  }
  else
  {
       encoder1Count-- ; 
  }
  
}
void countEncoder2(void)
{
   if(digitalRead(encoderB2)==digitalRead(encoderA2))
   {
      encoder2Count++ ; 
   }
   else
   {
    encoder2Count-- ; 
   }
}
