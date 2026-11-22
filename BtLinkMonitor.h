#pragma once
#include <stdint.h>
#include <string.h>

// Specific to the measured BT05 v6.1 UART: connected transmission => TX=\r\n;
// disconnected transmission => ERROR\r\n. A missing reply also expires.
class BtLinkMonitor {
 public:
 static const uint32_t timeoutMs=800, pingMs=250, frameTimeoutMs=40;
 uint32_t lastAck=0,lastByte=0,lastPing=0,acks=0,errors=0,badFrames=0,frames=0;
 uint32_t digitalFrames=0;
 bool haveAck=false,badEvent=false;
 uint8_t frame[8],frameLen=0,ready[8],readyLen=0,readyAt=0;
 char text[8];uint8_t textLen=0;
 bool healthy(uint32_t now)const{return haveAck&&(uint32_t)(now-lastAck)<=timeoutMs;}
 bool pending()const{return readyAt<readyLen;}
 int read(){return pending()?ready[readyAt++]:-1;}
 bool takeBadEvent(){bool b=badEvent;badEvent=false;return b;}
 void bad(){++badFrames;badEvent=true;haveAck=false;frameLen=textLen=0;}
 void tick(uint32_t now){
  if((frameLen||textLen)&&(uint32_t)(now-lastByte)>frameTimeoutMs)bad();
 }
 bool canPing(uint32_t now)const{
  return !frameLen&&!textLen&&!pending()&&(uint32_t)(now-lastByte)>=25&&
    (uint32_t)(now-lastPing)>=pingMs;
 }
 void publish(){
  readyLen=frameLen;readyAt=0;memcpy(ready,frame,frameLen);
  ++frames;if(frame[1]==1&&frame[2]==1)++digitalFrames;
  frameLen=0;
 }
 void feed(uint8_t c,uint32_t now){
  tick(now);lastByte=now;
  if(c==0xFF){
   if(frameLen||textLen)bad();
   frame[0]=c;frameLen=1;return;
  }
  if(frameLen){
   if(frameLen>=sizeof(frame)){bad();return;}
   frame[frameLen++]=c;
   if(frameLen==2&&frame[1]>1){bad();return;}
   if(frameLen==3&&((frame[1]==0&&frame[2]!=3)||
       (frame[1]==1&&(frame[2]<1||frame[2]>3)))){bad();return;}
   if(frameLen==5){
    if(frame[1]==0){
     if(frame[3]==0&&frame[4]==0)publish();else bad();
    }else if(frame[3]!=1||frame[4]!=2)bad();
   }else if(frameLen==8){
    if(frame[1]==1&&frame[3]==1&&frame[4]==2&&frame[7]==0&&
       frame[5]<=0x3F&&(frame[2]!=1||frame[6]<=0x0F))publish();else bad();
   }
   return;
  }
  if(textLen||c=='T'||c=='E'){
   if(textLen>=sizeof(text)){bad();return;}
   text[textLen++]=(char)c;
   if(c=='\n'){
    if(textLen==5&&!memcmp(text,"TX=\r\n",5)){haveAck=true;lastAck=now;++acks;}
    else if(textLen==7&&!memcmp(text,"ERROR\r\n",7)){
     haveAck=false;badEvent=true;++errors;
    }else bad();
    textLen=0;
   }
   return;
  }
  // Some app startup sequences include a spare end-of-frame byte.
  if(c!=0&&c!='\r'&&c!='\n')bad();
 }
};
