#pragma once
#include <stdint.h>
class SonarCapture {
 public:
 volatile uint8_t phase=0;
 volatile bool ready=false;
 volatile uint32_t started=0,rise=0,width=0;
 void start(uint32_t now){ready=false;started=now;phase=1;}
 void edge(bool high,uint32_t now){
  if(phase==1&&high){rise=now;phase=2;}
  else if(phase==2&&!high){width=(uint32_t)(now-rise);ready=true;phase=0;}
 }
 static const uint32_t TIMEOUT=25000, OPEN_RANGE=TIMEOUT;
 // Call with interrupts disabled on AVR; false means no new result.
 // pulse==0: Echo never rose (sensor missing/broken). pulse==OPEN_RANGE: Echo rose
 // but no target returned in range (HC-SR04 holds Echo ~38 ms): path is clear.
 bool take(uint32_t now,uint32_t& pulse){
  if(ready){pulse=width;ready=false;return true;}
  if(phase&&(uint32_t)(now-started)>=TIMEOUT){
   pulse=phase==2?OPEN_RANGE:0;phase=0;return true;
  }
  return false;
 }
};
