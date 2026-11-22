// Nano ATmega328P, 4WD L298N. Pin map from supplied do_line_4wd + wiring slide.
// D0/D1 drive the motor bridge: DO NOT enable hardware Serial.
#define CUSTOM_SETTINGS
#include <Arduino.h>
#include <Dabble.h>
#include <Servo.h>
#include "BtLinkMonitor.h"
#include "SonarCapture.h"
#ifdef ROBOT_USB_DIAG
#include <stdio.h>
#endif
#ifndef ROBOT_HOST_TEST
#include <NeoSWSerial.h>
#include <avr/interrupt.h>
extern NeoSWSerial DabbleSoftSerial;
#endif

const uint8_t ENA=5, ENB=6, IN1=0, IN2=1, IN3=4, IN4=7;
const uint8_t ENC_L=2, ENC_R=3, TRIG=A0, ECHO=A1, SERVO_PIN=A5;
const uint8_t LINE_PINS[5]={8,9,10,11,12};
const unsigned long SEGMENT_TIMEOUT=6000, STALL_TIMEOUT=900;
// HW-871 measured 22/11: white paper = HIGH (LED on), lifted/no reflection = LOW
// (LED off). A non-reflective black line is therefore LOW. Glossy vinyl tape read
// as white (HIGH) on this module, so the track needs a matte IR-absorbing line.
const uint8_t LINE_ACTIVE_LEVEL=LOW;
const int LINE_BASE_PWM=110, LINE_TRIM_LIMIT=40;
const unsigned long LINE_SPEED_WINDOW=200;
const float LINE_SPEED_SET=.4f, LINE_SPEED_KP=60, LINE_SPEED_KI=80;
enum Mode { MANUAL, LINE, OBSTACLE };
Mode mode=MANUAL;
bool running=false, fault=false, needRelease=true;
Servo head;
bool headAttached=false;
int headAngle=90;
unsigned long headMoved=0, lastPacket=0;
bool havePacket=false;
uint8_t faces=0, arrows=0, previousFaces=0;
bool newPacket=false;
volatile unsigned long ticksL=0, ticksR=0, edgeL=0, edgeR=0;
int outputL=0, outputR=0;
void controls();
void stopRobot(bool latchFault);
BtLinkMonitor btLink;
SonarCapture sonar;
uint16_t squarePresses=0,circlePresses=0;
#ifndef ROBOT_HOST_TEST
volatile uint8_t previousPortC=0;
// Called both from PCINT1 and from NeoSWSerial's TX loop while IRQs are masked.
void robotPortCChange(uint8_t pins){
 uint8_t changed=pins^previousPortC;previousPortC=pins;
 if(changed&_BV(3))NeoSWSerial::rxISR(pins);
 if(changed&_BV(1))sonar.edge((pins&_BV(1))!=0,micros());
}
ISR(PCINT1_vect){robotPortCChange(PINC);}
#endif
#ifndef ROBOT_HOST_TEST
class SafeBtStream:public Stream {
 public:
 int available() override{
  while(!btLink.pending()&&DabbleSoftSerial.available())
   btLink.feed((uint8_t)DabbleSoftSerial.read(),millis());
  return btLink.pending()?btLink.readyLen-btLink.readyAt:0;
 }
 int peek() override{return available()?btLink.ready[btLink.readyAt]:-1;}
 int read() override{return available()?btLink.read():-1;}
 void flush() override{DabbleSoftSerial.flush();}
 size_t write(uint8_t b) override{return DabbleSoftSerial.write(b);}
} safeBt;
void beginBt(){
 previousPortC=PINC;
 DabbleSoftSerial.begin(9600);Dabble.begin(safeBt);
 PCMSK1|=_BV(1);PCICR|=_BV(PCIE1); // A1 echo shares PORTC with A3 Bluetooth RX.
}
void serviceBt(){
 btLink.tick(millis());
 if(btLink.canPing(millis())){
  btLink.lastPing=millis();
  // Use the complete Dabble QUERY_SELECTED frame. A lone NUL can produce
  // TX= even without a confirmed link and is not a valid liveness probe.
  const uint8_t query[5]={0xFF,1,0xFF,0,0};
  for(uint8_t c:query)DabbleSoftSerial.write(c);
 }
}
#else
void beginBt(){Dabble.begin(9600,A3,A2);}
void serviceBt(){btLink.tick(millis());}
#endif

// Dabble 1.5.2's setOnNewModuleFrame is not invoked by sendToModules.
// Observe actual completed digital GamePad frames via ModuleParent instead.
class PacketObserver : public ModuleParent {
 public:
  PacketObserver():ModuleParent(1){}
 protected:
  void processFrame() override {
    if(Dabble.getModuleId()==1 && Dabble.getFunctionId()!=1){
      stopRobot(false);havePacket=false;needRelease=true;faces=arrows=previousFaces=0;
      return; // Never keep an old Digital direction after switching app mode.
    }
    if(Dabble.getModuleId()!=1 || Dabble.getFunctionId()!=1 ||
       Dabble.getArgumentNo()!=1 || Dabble.getArgumentLength(0)!=2) return;
    byte *data=Dabble.getArgumentData(0);
    faces=data[0]&0x3f; arrows=data[1]&0x0f;
    if(faces&(1<<5))++squarePresses;
    if(faces&(1<<3))++circlePresses;
    lastPacket=millis(); havePacket=true; newPacket=true;
    // Honour a stop even if a later frame is in the same serial buffer.
    if(faces & ((1<<4)|(1<<1))) { running=false; }
    controls(); // Process every completed frame, including short press/release pairs.

  }
} packetObserver;

void encoderL(){unsigned long n=micros();if(n-edgeL>=500){++ticksL;edgeL=n;}}
void encoderR(){unsigned long n=micros();if(n-edgeR>=500){++ticksR;edgeR=n;}}
void readTicks(unsigned long &l,unsigned long &r){noInterrupts();l=ticksL;r=ticksR;interrupts();}
void motor(int left,int right){
  left=constrain(left,-200,200); right=constrain(right,-200,200);
  if(!running || fault) left=right=0;
#ifdef ROBOT_USB_DIAG
  analogWrite(ENA,0);analogWrite(ENB,0); // USB diagnostic: requested output only
#else
  digitalWrite(IN1,left>0); digitalWrite(IN2,left<0);
  digitalWrite(IN3,right>0); digitalWrite(IN4,right<0);
  analogWrite(ENA,abs(left)); analogWrite(ENB,abs(right));
#endif
  outputL=left; outputR=right;
}
void pointHead(int angle){
  angle=constrain(angle,10,180);
  if(headAngle!=angle){headAngle=angle;head.write(angle);headMoved=millis();}
}

bool segmentActive=false, segmentFindLine=false;
unsigned long segL=0,segR=0,segTarget=0,segStart=0,progressL=0,progressR=0;
unsigned long progressAtL=0,progressAtR=0;
int segPwmL=0,segPwmR=0;
int routeStep=-1;
unsigned long lineLostAt=0;
bool lineLost=false,lineSeen=false;
int lastLineSide=0;
float lineIntegral=0,lastLineError=0;
int lineTrim=0,lineSteer=0;bool steerFresh=true;
unsigned long speedAt=0,speedL=0,speedR=0,steerAt=0;
enum AvoidState { CRUISE, SCAN_LEFT, SCAN_RIGHT, CENTER_HEAD, TURNING };
AvoidState avoidState=CRUISE;
int leftCM=-1,rightCM=-1;
float distanceCM=-1;
unsigned long distanceAt=0;
bool freshDistance=false;
unsigned long blinkEpoch=0;
unsigned long motionL=0,motionR=0,motionAtL=0,motionAtR=0;

void resetLineControl(){
 lineIntegral=0;lineTrim=lineSteer=0;steerFresh=true;
 speedAt=steerAt=millis();readTicks(speedL,speedR);
}
void resetMotion(){
  segmentActive=false; routeStep=-1; avoidState=CRUISE;
  resetLineControl();lineLost=false;lineSeen=false;lastLineSide=0;
  readTicks(motionL,motionR);motionAtL=motionAtR=millis();
  distanceCM=-1; distanceAt=millis()-100;
  noInterrupts();sonar.phase=0;sonar.ready=false;interrupts();
  pointHead(90);
}
void stopRobot(bool latchFault=false){
  running=false; fault=latchFault;
  if(headAttached){head.detach();headAttached=false;}
  resetMotion(); motor(0,0);
}
void selectMode(Mode selected){stopRobot();mode=selected;blinkEpoch=millis();}
// Also monitor ordinary driving, outside encoder-controlled route segments.
// A stopped/disconnected encoder must not leave a wheel driven indefinitely.
void motionGuard(){
#ifdef ROBOT_USB_DIAG
  return; // No actual motion is requested on USB; encoder stall is tested separately.
#endif
  unsigned long l,r;readTicks(l,r);
  if(abs(outputL)<80 || l!=motionL)motionAtL=millis();
  if(abs(outputR)<80 || r!=motionR)motionAtR=millis();
  motionL=l;motionR=r;
  if(running && ((abs(outputL)>=80 && millis()-motionAtL>STALL_TIMEOUT) ||
                 (abs(outputR)>=80 && millis()-motionAtR>STALL_TIMEOUT)))stopRobot(true);
}
void controls(){
  // Button frames are events, not heartbeats. Use measured BT05 TX replies.
  bool badTransport=btLink.takeBadEvent();
  if(badTransport || (havePacket && !btLink.healthy(millis()))){
    stopRobot();havePacket=false;needRelease=true;faces=arrows=previousFaces=0;
    newPacket=false;
  }
  if(!newPacket)return;
  newPacket=false;
  if(needRelease){
    if(faces==0 && arrows==0){needRelease=false;previousFaces=0;}
    return;
  }
  uint8_t pressed=faces & ~previousFaces;
  previousFaces=faces;
  if(faces & ((1<<4)|(1<<1))){stopRobot();return;} // Cross / SELECT
  if(pressed & (1<<2)){selectMode(MANUAL);return;} // Triangle
  if(pressed & (1<<5)){selectMode(LINE);return;}   // Square
  if(pressed & (1<<3)){selectMode(OBSTACLE);return;} // Circle
  if(pressed & 1){
    if(running)stopRobot();
    else if(!arrows){
      fault=false;resetMotion();
      if(mode!=MANUAL){
#ifdef ROBOT_USB_DIAG
        head.attach(LED_BUILTIN); // Test Timer1 timing without driving the physical servo on A5.
#else
        head.attach(SERVO_PIN);
#endif
        head.write(90);headMoved=millis();headAttached=true;
      }
      running=true;
    }
  }
}

void ultrasonic(){
  freshDistance=false;
#ifndef ROBOT_HOST_TEST
  uint32_t pulse=0;bool got;
  noInterrupts();got=sonar.take(micros(),pulse);interrupts();
  if(got){distanceCM=pulse?pulse*0.01715f:-1;freshDistance=true;}
  // After an open-range reading Echo may still be high; a trigger then is ignored
  // by the module and would look like a missing sensor, so wait for Echo low.
  if(!sonar.phase && !sonar.ready && millis()-distanceAt>=100 && digitalRead(ECHO)==LOW){
    distanceAt=millis();
    noInterrupts();sonar.start(micros());interrupts();
    digitalWrite(TRIG,LOW);delayMicroseconds(2);
    digitalWrite(TRIG,HIGH);delayMicroseconds(10);digitalWrite(TRIG,LOW);
  }
#else
  if(millis()-distanceAt<100)return;
  distanceAt=millis();
  unsigned long pulse=pulseInLong(ECHO,HIGH,25000UL);
  distanceCM=pulse?pulse*0.01715f:-1;freshDistance=true;
#endif
}
uint8_t readLine(){
  uint8_t bits=0;for(uint8_t i=0;i<5;i++)if(digitalRead(LINE_PINS[i])==LINE_ACTIVE_LEVEL)bits|=1<<i;
  return bits;
}
bool validLine(uint8_t b){return b!=0 && b!=31;}
unsigned long spinTicks(float deg){
  // 4WD correction from original do_line_4wd: 20 ticks/rev, 3x spin factor.
  return (unsigned long)(3.0f*deg*PI/180.0f*0.0475f/0.2042035f*20.0f+0.5f);
}
unsigned long forwardTicks(float meters){return (unsigned long)(1.5f*meters/0.2042035f*20.0f+0.5f);}
void startSegment(int left,int right,unsigned long target,bool findLine=false){
  readTicks(segL,segR);progressL=segL;progressR=segR;
  segStart=progressAtL=progressAtR=millis();
  segTarget=target;segPwmL=left;segPwmR=right;
  segmentFindLine=findLine;segmentActive=true;
}
// Return 1 for completion, 0 while active, -1 for fault. Each wheel stops at target.
int segmentUpdate(){
  unsigned long l,r;readTicks(l,r);
  if(l!=progressL){progressL=l;progressAtL=millis();}
  if(r!=progressR){progressR=r;progressAtR=millis();}
  bool doneL=l-segL>=segTarget,doneR=r-segR>=segTarget;
  uint8_t sample=readLine();
  bool found=segmentFindLine && validLine(sample) && (sample&14);
  if(found || (doneL && doneR)){segmentActive=false;motor(0,0);return 1;}
  if(millis()-segStart>SEGMENT_TIMEOUT ||
     (!doneL && millis()-progressAtL>STALL_TIMEOUT) ||
     (!doneR && millis()-progressAtR>STALL_TIMEOUT)){
    stopRobot(true);return -1;
  }
  motor(doneL?0:segPwmL,doneR?0:segPwmR);return 0;
}
void nextRoute(){
  switch(++routeStep){
    case 0:startSegment(-110,110,spinTicks(75));break;
    case 1:startSegment(100,100,forwardTicks(0.30f));break;
    case 2:startSegment(110,-110,spinTicks(65));break;
    case 3:startSegment(100,100,forwardTicks(0.35f));break;
    case 4:startSegment(110,-110,spinTicks(60));break;
    case 5:startSegment(100,100,forwardTicks(1.0f),true);break;
    case 6:startSegment(-110,110,spinTicks(15));break;
    default:
      routeStep=-1;resetLineControl();
      lineLost=false;break;
  }
}
void updateLineControl(float error){
 if(millis()-speedAt>=LINE_SPEED_WINDOW){
  unsigned long l,r;readTicks(l,r);
  float dt=(millis()-speedAt)/1000.0f;
  float measured=((l-speedL)+(r-speedR))*.5f*.2042035f/20.0f/dt;
  float e=LINE_SPEED_SET-measured;
  lineIntegral=constrain(lineIntegral+LINE_SPEED_KI*e*dt,-40.0f,40.0f);
  lineTrim=constrain((int)(LINE_SPEED_KP*e+lineIntegral),-LINE_TRIM_LIMIT,LINE_TRIM_LIMIT);
  speedL=l;speedR=r;speedAt=millis();
 }
 if(steerFresh){lastLineError=error;steerFresh=false;}
 if(millis()-steerAt>=20){
  lineSteer=(int)(25.0f*error+40.0f*(error-lastLineError));
  lastLineError=error;steerAt=millis();
 }
}
void followLine(){
  if(routeStep>=0){
    if(segPwmL>0 && segPwmR>0 && freshDistance &&
       (distanceCM<=0 || distanceCM<10)){stopRobot(true);return;}
    if(segmentUpdate()==1)nextRoute();
    return;
  }
  // No echo means stop and wait, rather than drive blind.
  if(distanceCM<=0 || millis()-headMoved<450){motor(0,0);resetLineControl();return;}
  uint8_t b=readLine();
  if(validLine(b) && distanceCM<18){motor(0,0);routeStep=-1;nextRoute();return;}
  if(!validLine(b)){
    if(!lineLost){lineLost=true;lineLostAt=millis();}
    if(!lineSeen){motor(0,0);resetLineControl();return;} // Armed, waiting for first line.
    if(millis()-lineLostAt>1500){stopRobot();return;}
    resetLineControl();
    if(distanceCM<18){motor(0,0);return;}
    motor(lastLineSide<0?-100:100,lastLineSide>0?-100:100);return;
  }
  lineLost=false;lineSeen=true;
  int weight=0,count=0;
  for(int i=0;i<5;i++)if(b&(1<<i)){weight+=i-2;++count;}
  float error=(float)weight/count;
  if(error<-0.1f)lastLineSide=-1;else if(error>0.1f)lastLineSide=1;
  updateLineControl(error);
  motor(constrain(LINE_BASE_PWM+lineTrim+lineSteer,0,180),
        constrain(LINE_BASE_PWM+lineTrim-lineSteer,0,180));
}
void avoidObstacles(){
  switch(avoidState){
    case CRUISE:
      if(millis()-headMoved<450 || distanceCM<=0){motor(0,0);return;}
      if(distanceCM<20){motor(0,0);pointHead(180);avoidState=SCAN_LEFT;}
      else motor(115,115);
      break;
    case SCAN_LEFT:
      if(millis()-headMoved>=700 && freshDistance){
        leftCM=(int)distanceCM;pointHead(10);avoidState=SCAN_RIGHT;
      }break;
    case SCAN_RIGHT:
      if(millis()-headMoved>=700 && freshDistance){
        rightCM=(int)distanceCM;pointHead(90);avoidState=CENTER_HEAD;
      }break;
    case CENTER_HEAD:
      if(millis()-headMoved>=500){
        if(leftCM<20 && rightCM<20){stopRobot(true);return;}
        bool right=rightCM>=20; // Prefer right if both clear, as original.
        // Original obstacle-only 4WD code uses 6x spin factor vs 3x in line mode.
        unsigned long target=2*spinTicks(90);
        startSegment(right?115:-115,right?-115:115,target);
        avoidState=TURNING;
      }break;
    case TURNING:
      if(segmentUpdate()==1){distanceCM=-1;avoidState=CRUISE;}
      break;
  }
}
void indicate(){
#ifdef ROBOT_USB_DIAG
 static unsigned long reportAt=0;
 static char logBuffer[240];static uint16_t logLen=0,logAt=0;
 // Never block the RX parser while USB UART drains a long diagnostic line.
 if(logAt==logLen && millis()-reportAt>=250){
  reportAt=millis();
  unsigned long l,r;readTicks(l,r);
  logLen=snprintf(logBuffer,sizeof(logBuffer),
   "t=%lu mode=%u run=%u fault=%u cm10=%d mask=%u head=%d servo=%u scan=%u route=%d alive=%u ack=%lu err=%lu bad=%lu frames=%lu sq=%u ci=%u L=%d R=%d encL=%lu encR=%lu pwmL=0 pwmR=0 A5=OFF\n",
   millis(),(unsigned)mode,(unsigned)running,(unsigned)fault,(int)(distanceCM*10),
   readLine(),headAngle,(unsigned)headAttached,(unsigned)avoidState,routeStep,
   (unsigned)btLink.healthy(millis()),btLink.acks,btLink.errors,btLink.badFrames,
   btLink.digitalFrames,squarePresses,circlePresses,outputL,outputR,l,r);
  if(logLen>=sizeof(logBuffer))logLen=sizeof(logBuffer)-1;
  logAt=0;
 }
 while(logAt<logLen && Serial.availableForWrite()>0)Serial.write(logBuffer[logAt++]);
#endif
  unsigned long phase=(millis()-blinkEpoch)%2200;
  uint8_t count=(uint8_t)mode+1;
  digitalWrite(LED_BUILTIN,fault?((millis()/100)%2):
      (running?HIGH:(phase<count*300UL && phase%300<120)));
}
void setup(){
  // Disable PWM before configuring direction pins.
  digitalWrite(ENA,LOW);digitalWrite(ENB,LOW);
  pinMode(ENA,OUTPUT);pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);
  motor(0,0);
  for(uint8_t i=0;i<5;i++)pinMode(LINE_PINS[i],INPUT);
  pinMode(ENC_L,INPUT_PULLUP);pinMode(ENC_R,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L),encoderL,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R),encoderR,RISING);
  pinMode(TRIG,OUTPUT);digitalWrite(TRIG,LOW);pinMode(ECHO,INPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  // Servo remains detached at boot; attach only when starting an autonomous mode.
  pinMode(SERVO_PIN,INPUT);headMoved=millis();
#ifdef ROBOT_USB_DIAG
  Serial.begin(115200);Serial.println(F("ROBOT_AUTO_V23_USB MOTOR_PWM=0 SERVO_A5=OFF"));
#endif
  beginBt();
  resetMotion();
}
void loop(){
  Dabble.processInput();serviceBt();controls();
  if(!running || fault || !havePacket || needRelease){
#ifdef ROBOT_USB_DIAG
    ultrasonic();
#endif
    motor(0,0);indicate();return;
  }
  if(mode==MANUAL){
    if(arrows&1)motor(140,140);
    else if(arrows&2)motor(-140,-140);
    else if(arrows&4)motor(-140,140);
    else if(arrows&8)motor(140,-140);
    else motor(0,0);
  }else{
    ultrasonic();
    // Input may have arrived while the bounded sonar read was in progress.
    Dabble.processInput();controls();
    if(running && !fault && havePacket && !needRelease){
      if(mode==LINE)followLine();else if(mode==OBSTACLE)avoidObstacles();
    }else motor(0,0);
  }
  motionGuard();
  indicate();
}
