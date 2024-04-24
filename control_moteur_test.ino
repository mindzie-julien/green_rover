#include <ArduinoJson.h>
#include <Wire.h>

#define ENCA 2//18 moteur M
#define ENCB 3//19 moteur M 
#define ENCA_I 18//moteur 
#define ENCB_I 19
#define PWM 5//6 moteur M, moteur M
#define IN2 4 // moteur M
#define IN1 7 //moteur M

#define SPEED_0 'a'//0x0000
#define SPEED_25 'b'//0x0001
#define SPEED_50 'c'//0x0002
#define SPEED_75 'd'//0x0003
#define SPEED_100 'e'//0x0004
#define TURN_RIGHT_5 'f'//0x0005// reparer // ok
#define TURN_RIGHT_30 'g'//0x0006// reparer// ok
#define TURN_RIGHT_60 'h'//0x0007// reparer// ok
#define TURN_RIGHT_90 'i'//0x0008// reparer// ok 
#define TURN_RIGHT_180 'j'//0x0009//ok// ok

#define BACKWARD 'k'//0x000A
#define FORWARD 'l'//0x000B
#define TURN_LEFT_5 'm'//0x000C// reparer// reparer
#define TURN_LEFT_30 'n'//0x000D// ok// ok
#define TURN_LEFT_60 'o'//0x00E// reparer// reparer
#define TURN_LEFT_90 'p'//0x000F// ok// ok
#define TURN_LEFT_180 'q'//0x0010// ok// reparer
#define RIDE_1CM 'r'//0x0011// reparer    // reparer
#define RIDE_5CM 's'//0x0012// ok   // reparer
#define RIDE_15CM 't'//0x0013// reparer    // reparer
#define RIDE_25CM 'u'//0x0014// reparer    // ok
#define RIDE_50CM 'v'//0x0015// reparer   // ok  
#define RIDE_75CM 'w'//0x0016// ok    // reparer
#define RIDE_100CM 'x'//0x0017// reparer  // reparer  
#define RIDE_150CM 'y'//0x0018// reparer    // reparer
#define RIDE_200CM 'z'//0x0019// ok    // reparer
#define RIDE_FORWARD_FREE 'A'//0x001A
#define RIDE_FREE 'B'//0x001B
#define RIDE_RIGHT_FREE 'C'//0x001C
#define RIDE_LEFT_FREE 'D'//0x001D
#define STOP 'E'//0x001E =
#define NONE 'F'//0x001F
#define TURN_RIGHT_45 'G'//0x0020
#define TURN_LEFT_45 'H'//0x0021
#define REQUEST_OPEN 'I'//0x0022
#define REQUEST_FEEDBACK 'J'//0x0023
#define FEEDBACK 'K'//0x0024
#define REQUEST_CLOSE 'L'//0x0025

#include <SoftwareSerial.h>

String msgData;// =Serial.readStringUntil('\n');
char request;    
     //msg = Serial.read();
     int ui=0;
     String command = "";
      int value = 0;
      
      
byte requestIndex = 0;
const byte I2C_SLAVE_ADDR = 0x8;
const byte I2C_LENGTH_LIMIT = 32;
const byte ASK_FOR_LENGTH = 0x0;
const byte ASK_FOR_DATA = 0x1;
String received_data_global ="";
//DynamicJsonDocument doc(1024);
char directionSense = 'F', turnSide = '0', msg = ' ';
bool wait = false, taskProcessed = true, infiniteRide = false, stopMeasure = false, stopEncode = false;
char taskProcessing = NONE;
int target = 0,speedMotor = 0, countStep = 0, currentStepTime = 0, previousStepTime = 0, countStepDistance = 0, targetGive = 0;
long int security_counter, timeDelay;
long int pos = 0, prevPos = 0, pos_i = 0;
long int prevT = 0;
float eprev = 0;
float eintegral = 0;
float currentSpeed = 0;
float timeSample = 0;
float distance = 0;
//SoftwareSerial mySerial(TX,RX);

void setup() {
  //Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(ENCA_I,INPUT);
  pinMode(ENCB_I,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_I),readEncoder_I,RISING);
  Serial.begin(19200);
  while(!Serial){}
  
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
}

void loop() {

  //test();
  mainFunction();
  //printParameter();

}

void receiveEvent(int nBytes) {
    String received_data = "";
    
    while (Wire.available()) {
        request = (char) Wire.read();
        received_data  += request;
    }
    
    received_data.remove(0, 1);
    received_data += '}';
    ui++;
    StaticJsonDocument<10000> doc;
    DeserializationError error = deserializeJson(doc, received_data);
    if(!error){ 
     command = doc["order"].as<String>();
      value = doc["value"];
      if( command == "right" )
      {
        msg = TURN_RIGHT_5;
        targetGive = value*8;
      }
      else if( command == "left" ){
        msg = TURN_LEFT_5;
        targetGive = value*8;
      }
      else if( command == "forward" ){
        msg = FORWARD;
        targetGive = value*50;
      }
      
     speedMotor = 51;}
     else{
      Serial.println("error");}
      
     Serial.println(received_data);
     Serial.println(ui);
     Serial.println(msg);
  // Parse les données JSON reçues
  
  
  Wire.end();            // Termine la communication I2C
  Wire.begin(I2C_SLAVE_ADDR);
}

void requestEvent() {
    String jsonString;
    if (request == ASK_FOR_LENGTH) {
        Wire.write(jsonString.length());
        requestIndex = 0;
    }
    long dist = pos/50;
    StaticJsonDocument<10000> dataJson;
    dataJson["distance"] = dist;
    serializeJson(dataJson, jsonString);
    if (request == ASK_FOR_DATA) {
        if (requestIndex < (jsonString.length() / I2C_LENGTH_LIMIT)) {
            Wire.write(jsonString.c_str() + requestIndex * I2C_LENGTH_LIMIT, I2C_LENGTH_LIMIT);
            requestIndex++;
        }
        else {
            Wire.write(jsonString.c_str() + requestIndex * I2C_LENGTH_LIMIT, jsonString.length() % I2C_LENGTH_LIMIT);
            requestIndex = 0;
        }
    }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm + 1,pwmVal);//PWM du moteur gauche pin 6
  
  analogWrite(pwm,pwmVal);
  
  if(dir == 1){
    if(turnSide == 'R')
    {
      digitalWrite(in1,HIGH);
      digitalWrite(in2,HIGH);
    }
    else if(directionSense == 'B') 
    {
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
    }

  }
  else if(dir == -1){
    if(turnSide == 'L')
    {
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
    }
    else if(directionSense == 'F')
    {
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
    }
  }
  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
    updateSpeed(true);
    updateDistance(true);
  }
  else{
    pos--;
    updateSpeed(false);
    updateDistance(false);
  }
  if(stopEncode == true)
    pos = 0;
}

void readEncoder_I(){
  int b = digitalRead(ENCB_I);
  if(b > 0){
    pos_i++;
  }
  else{
    pos_i--;
  }
  if(stopEncode == true)
    pos_i = 0;
}

void PID_regulation_motor()
{ 
  //Serial.print("\n \t");Serial.print("PID");
  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = pos-target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
    // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  /**************** a tester ******************/
  if(infiniteRide == true)
  {
    
    if((abs(target - pos) > 300)||(abs(target - pos_i) > 300))
    {
      pos = pos - 3000;
      pos_i = pos_i - 3000;
    }
    e = 0;
    prevT = 0;
    eprev = 0;
    eintegral = 0;
    
    if(directionSense == 'F')
    {
      if(turnSide == 'L')
      {
        setMotor(-1,speedMotor,PWM,IN1,IN2);
      }
      else if(turnSide == 'R')
      {
        setMotor(1,speedMotor,PWM,IN1,IN2);
      }
      else
      {
        setMotor(-1,speedMotor,PWM,IN1,IN2); 
      }
    }
    else if(directionSense == 'B')
    {
      if(turnSide == 'L')
      {
        setMotor(1,speedMotor,PWM,IN1,IN2);
      }
      else if(turnSide == 'R')
      {
        setMotor(-1,speedMotor,PWM,IN1,IN2);
      }
      else
      {
        setMotor(1,speedMotor,PWM,IN1,IN2); 
      }
    }
    
    trajectoryCorrection();
  }
  else
  {
    targetReached(dir);
    trajectoryCorrection();
  
    eprev = e;
  }
 
  
}

void speedChecker()
{
  switch(msg)
  {
    case SPEED_0 :
    speedMotor = 51;
    break;
    case SPEED_25 :
    speedMotor = 102;
    break;
    case SPEED_50 :
    speedMotor = 153;
    break;
    case SPEED_75 :
    speedMotor = 204;
    break;
    case SPEED_100 :
    speedMotor = 255;
    break;
  }
}

void commandChecker()
{
  switch(msg)
  {

    case BACKWARD :
    directionSense = 'B';
    break;
    case FORWARD :
    directionSense = 'F';
    break;
    case TURN_RIGHT_5 :
    taskProcessing = TURN_RIGHT_5;
    turnSide = 'R';
    taskProcessed = false;
    break;
    case TURN_LEFT_5 :
    taskProcessing = TURN_LEFT_5;
    turnSide = 'L';
    taskProcessed = false;
    break;
    case TURN_RIGHT_30 :
    taskProcessing = TURN_RIGHT_30;
    turnSide = 'R';
    taskProcessed = false;
    break;
    case TURN_LEFT_30 :
    taskProcessing = TURN_LEFT_30;
    turnSide = 'L';
    taskProcessed = false;
    break;
    case TURN_RIGHT_45 :
    taskProcessing = TURN_RIGHT_45;
    turnSide = 'R';
    taskProcessed = false;
    break;
    case TURN_LEFT_45 :
    taskProcessing = TURN_LEFT_45;
    turnSide = 'L';
    taskProcessed = false;
    break;
    case TURN_RIGHT_60 :
    taskProcessing = TURN_RIGHT_60;
    turnSide = 'R';
    taskProcessed = false;
    break;
    case TURN_LEFT_60 :
    taskProcessing = TURN_LEFT_60;
    turnSide = 'L';
    taskProcessed = false;
    break;
    case TURN_RIGHT_90 :
    taskProcessing = TURN_RIGHT_90;
    turnSide = 'R';
    taskProcessed = false;
    break;
    case TURN_LEFT_90 :
    taskProcessing = TURN_LEFT_90;
    turnSide = 'L';
    taskProcessed = false;
    break;
    case TURN_RIGHT_180 :
    taskProcessing = TURN_RIGHT_180;
    turnSide = 'R';
    taskProcessed = false;
    break;
    case TURN_LEFT_180 :
    taskProcessing = TURN_LEFT_180;
    turnSide = 'L';
    taskProcessed = false;
    break;
    case RIDE_1CM :
    taskProcessing = RIDE_1CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_5CM :
    taskProcessing = RIDE_5CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_15CM :
    taskProcessing = RIDE_15CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_25CM :
    taskProcessing = RIDE_25CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_50CM :
    taskProcessing = RIDE_50CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_75CM :
    taskProcessing = RIDE_75CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_100CM :
    taskProcessing = RIDE_100CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_150CM :
    taskProcessing = RIDE_150CM;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_200CM :
    taskProcessing = RIDE_200CM;
    taskProcessed = false;
    turnSide = '0';
    break;
     case RIDE_FORWARD_FREE :
    taskProcessing = RIDE_FORWARD_FREE;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_FREE :
    taskProcessing = RIDE_FREE;
    taskProcessed = false;
    turnSide = '0';
    break;
    case RIDE_RIGHT_FREE :
    taskProcessing = RIDE_RIGHT_FREE;
    taskProcessed = false;
    turnSide = 'R';
    break;
    case RIDE_LEFT_FREE :
    taskProcessing = RIDE_LEFT_FREE;
    taskProcessed = false;
    turnSide = 'L';
    break;
    /*case STOP :
    taskProcessing = STOP;
    taskProcessed = false;
    turnSide = '0';
    stop();
    break;*//*******************************************/
    default:
    //
    break;
  }

}

void stop()
{
  taskProcessed = true; infiniteRide = false;
  bool M1 = !(digitalRead(IN1));
  bool M2 = !(digitalRead(IN2)); 
  digitalWrite(IN2, M2);
  analogWrite (PWM, speedMotor);
  digitalWrite(IN1, M1);
  analogWrite (PWM + 1, speedMotor);
  switch(speedMotor)
  {
    case 51 :
    delay(200);
    break;
    case 102 :
    delay(100);
    break;
    case 153 :
    delay(75);
    break;
    case 204 :
    delay(45);
    break;
    case 255 :
    delay(31);
    break;
  }
  target = 0;
  speedMotor = 0;
  analogWrite (PWM, 0);
  analogWrite (PWM + 1, 0);

  pos = 0; prevPos = 0; pos_i = 0;
  prevT = 0;
  eprev = 0;
  eintegral = 0;

}

void initMsg()
{
  request =' ';
  command = "";
      value = 0;
      Wire.onRequest(requestEvent);
  if(Wire.available())
  {
    Wire.onReceive(receiveEvent);
    /*
    //receiveEvent(1);
    
      String command = doc["order"];
      int value = doc["value"];
      if( command == "rigth" )
      {
        msg = TURN_RIGHT_5;
        targetGive = value*8;
      }
      else if( command == "left" ){
        msg = TURN_LEFT_5;
        targetGive = value*8;
      }
      else if( command == "forward" ){
        msg = FORWARD;
        targetGive = value*50;
      }
     speedMotor = 51;*/
    
     
     //Serial.println("ok");
      
    //security_counter = millis();
  }
  else
  {
    taskProcessing = NONE;
    
  }
 
}

void updateSpeed(bool trend)// vitesse en mm/ms
{
  if(trend == true)
  {
    countStep++;
  }
  else
  {
    countStep--;
  }
  if((turnSide != 'R')&&(turnSide != 'L'))
  {
    if(countStep == 0)
    {
      previousStepTime = currentStepTime;
    }
    if((countStep >= 5)||(countStep <= -5))
    {
      countStep = 0;
      currentStepTime = millis();
      timeSample = currentStepTime - previousStepTime;
      currentSpeed = 1/timeSample; //speed unit : mm/ms
    }
  }
}

void updateDistance(bool trend)// distance en mm
{
  if(trend == true)
  {
    countStepDistance++;
  }
  else
  {
    countStepDistance--;
  }
  if((turnSide != 'R')&&(turnSide != 'L'))
  {
    if(countStepDistance >= 5)
    {
      countStepDistance = 0;
      distance = distance + 1;// distance en mm
    }
    if(countStepDistance <= -5)
    {
      countStepDistance = 0;
      distance = distance - 1;// distance en mm
    }
  }
}

void sendMsg()
{
  String data;
  StaticJsonDocument<500> doc;
  doc["distance"] = 0;
  doc["speed"] = currentSpeed;
  doc["sampleTime"] = timeSample;
  if((turnSide != 'R')&&(turnSide != 'L'))
  {
    Serial.println(data);
  }
    
}

void sendDist()
{
  long dist = pos/50;
  /*my*/Serial.print("\t");Serial.print("dist : ");Serial.print(dist);Serial.print("\n");
}

void turn()
{

      if(taskProcessing == TURN_RIGHT_5)//10 degree
      {

         if(directionSense == 'F')
         {
          target = pos - targetGive;//40
         }
         if(directionSense == 'B')
         {
          target = pos - targetGive;//
         }
      }
      if(taskProcessing == TURN_RIGHT_30)
      {

         if(directionSense == 'F')
         {
          target = pos - 240;
         }
          if(directionSense == 'B')
         {
          target = pos - 240;
         }
      }
      if(taskProcessing == TURN_RIGHT_60)
      {

         if(directionSense == 'F')
         {
          target = pos - 480;
         }
          if(directionSense == 'B')
         {
          target = pos - 480;
         }
      }
      if(taskProcessing == TURN_RIGHT_90)
      {

         if(directionSense == 'F')
         {
          target = pos - 720;
         }
          if(directionSense == 'B')
         {
          target = pos - 720;
         }
      }
      if(taskProcessing == TURN_RIGHT_180)
      {

         if(directionSense == 'F')
         {
          target = pos - 1440;
         }
          if(directionSense == 'B')
         {
          target = pos - 1440;
         }
      }
      if(taskProcessing == TURN_LEFT_5)// 10 degree
      {

         if(directionSense == 'F')
         {
          target = pos + targetGive;//40
         }
         if(directionSense == 'B')
         {
          target = pos + targetGive;//40
         }
          
      }
      if(taskProcessing == TURN_LEFT_30)
      {

         if(directionSense == 'F')
         {
          target = pos + 240;
         }
          if(directionSense == 'B')
         {
          target = pos + 240;
         }
      }
      if(taskProcessing == TURN_LEFT_60)
      {

         if(directionSense == 'F')
         {
          target = pos + 480;
         }
          if(directionSense == 'B')
         {
          target = pos + 480;
         }
      }
      if(taskProcessing == TURN_LEFT_90)
      {

         if(directionSense == 'F')
         {
          target = pos + 720;
         }
          if(directionSense == 'B')
         {
          target = pos + 720;
         }
      }
      if(taskProcessing == TURN_LEFT_180)
      {

         if(directionSense == 'F')
         {
          target = pos + 1440;
         }
          if(directionSense == 'B')
         {
          target = pos + 1440;
         }
      }
      if(taskProcessing == RIDE_RIGHT_FREE)
      {
          infiniteRide = true;
         if(directionSense == 'F')
         {
          target = pos - 2880;
         }
          if(directionSense == 'B')
         {
          target = pos - 2880;
         }
      }
      if(taskProcessing == RIDE_LEFT_FREE)
      {
          infiniteRide = true;
         if(directionSense == 'F')
         {
          target = pos + 2880;
         }
          if(directionSense == 'B')
         {
          target = pos + 2880;
         }
      }

}

void distanceRide()
{

      if(taskProcessing == RIDE_1CM)
      {
         if(directionSense == 'F')
         {

            target = pos + 50;
         }
         else if(directionSense == 'B')
            target = pos - 50;
      }
      if(taskProcessing == RIDE_5CM)
      {
         if(directionSense == 'F')
         {

            target = pos + 250;
         }
         else if(directionSense == 'B')
            target = pos - 250;
      }
      if(taskProcessing == RIDE_15CM)
      {
          if(directionSense == 'F')
         {

            target = pos + 750;
         }
         else if(directionSense == 'B')
            target = pos - 750;
      }
      if(taskProcessing == RIDE_25CM)
      {
          if(directionSense == 'F')
         {

            target = pos + 1250;
         }
         else if(directionSense == 'B')
            target = pos - 1250;
      }
      if(taskProcessing == RIDE_50CM)
      {
          if(directionSense == 'F')
         {

            target = pos + 2500;
         }
         else if(directionSense == 'B')
            target = pos - 2500;
      }
      if(taskProcessing == RIDE_75CM)
      {
          if(directionSense == 'F')
         {

            target = pos + 3750;
         }
         else if(directionSense == 'B')
            target = pos - 3750;
      }
      if(taskProcessing == RIDE_100CM)
      {
          if(directionSense == 'F')
          {

            target = pos + 5000;
          }
          else if(directionSense == 'B')
          {

             target = pos - 5000;
          }

      }
      if(taskProcessing == RIDE_150CM)
      {
          if(directionSense == 'F')
         {

            target = pos + 7500;
         }
         else if(directionSense == 'B')
            target = pos - 7500;
      }
      if(taskProcessing == RIDE_200CM)
      {
          if(directionSense == 'F')
         {

            target = pos + 10000;
         }
         else if(directionSense == 'B')
            target = pos - 10000;
      }

      if(taskProcessing == RIDE_FREE)
      {
          if(directionSense == 'F')
         {
            infiniteRide = true;
            target = pos + 10000;
         }
         else if(directionSense == 'B')
            infiniteRide = true;
            target = pos - 10000;
      }
      
}

void targetReached(int dir)
{
    if((target > 0)&&(pos > target - 1))
  {
    stop();
    target = 0;
    taskProcessed = true;
    pos = 0; prevPos = 0; pos_i = 0;
    prevT = 0;
    eprev = 0;
    eintegral = 0;
    setMotor(dir,0,PWM,IN1,IN2);
  }
  else if((target < 0)&&(pos < target + 1))
  {
    stop();
    target = 0;
    taskProcessed = true;
    pos = 0; prevPos = 0; pos_i = 0;
    prevT = 0;
    eprev = 0;
    eintegral = 0;
    setMotor(dir,0,PWM,IN1,IN2);
  }
  else
  {
    setMotor(dir,speedMotor,PWM,IN1,IN2);
  }

  
}

void mainFunction()
{
  initMsg();
  
  speedChecker();
  if(msg == STOP)
  {
    stop();
    
  }
  else if(taskProcessed == false)
  {
    PID_regulation_motor();
    
  }
  else
  {
    commandChecker();
    turn();
    distanceRide();
    
  }
  stopEncoder();
  //Wire.end();            // Termine la communication I2C
            //Wire.begin(I2C_SLAVE_ADDR);
            
}

void test()
{
    msg = Serial.read();
  speedMotor =255;
  analogWrite (PWM + 1, speedMotor);
  digitalWrite(IN2, LOW);
  if(msg == 'E')
  {
    stop();
    delay(1000);
  }
}

void printParameter()
{
  Serial.print("\n \t");Serial.print(" pos : ");Serial.print(pos);
  Serial.print("\t target : ");Serial.print(target);Serial.print("\n \t");
  Serial.print(" speed : ");Serial.print(speedMotor);Serial.print("\n");
  Serial.print(" taskProcessing : ");Serial.print(taskProcessing);Serial.print("\n");
  Serial.print(" taskProcessed : ");Serial.print(taskProcessed);Serial.print("\n");
  Serial.print(" directionSense : ");Serial.print(directionSense);Serial.print("\n");
  Serial.print(" turnSide : ");Serial.print(turnSide);Serial.print("\n");
  Serial.print("\n \t -------------------------------");
  Serial.print("\n \t target : "); Serial.print(target);Serial.print("\n");
  Serial.print("\n \t pos : "); Serial.print(pos);Serial.print("\n");
  Serial.print("\n \t -------------------------------");
}

void trajectoryCorrection()
{

  long int prevPos = pos, prevPos_i = pos_i;
  if((turnSide != 'R')&&(turnSide != 'L'))
  {
    if(directionSense == 'F')
    {
        if( pos > pos_i + 4)
        {
          analogWrite(PWM,(speedMotor/(abs(pos - pos_i)))); 
        }
        if( pos_i > pos + 4)
        {
          analogWrite(PWM + 1,(speedMotor/(abs(pos - pos_i))));
        }
    }
    if(directionSense == 'B')
    {
        if( pos < pos_i - 4)
        {
          analogWrite(PWM,(speedMotor/(abs(pos - pos_i))));
        }
        if( pos_i < pos - 4)
        {
          analogWrite(PWM + 1,(speedMotor/(abs(pos - pos_i))));
        }
    }
    delay(15);
    analogWrite(PWM, speedMotor);
    analogWrite(PWM + 1, speedMotor);
    pos = prevPos;
    pos_i = prevPos_i;
  }
}

void stopEncoder()
{
  if(target == 0)
  {
    stopEncode = true;
  }
  else
  {
    stopEncode = false;
  }
}
