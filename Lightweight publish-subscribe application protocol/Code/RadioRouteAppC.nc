
#define NEW_PRINTF_SEMANTICS
#include "printf.h"
#include "RadioRoute.h"

configuration RadioRouteAppC {}
implementation {
/****** COMPONENTS *****/

  components MainC, RadioRouteC as App;
  //add the other components here
  
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new TimerMilliC() as Timer2;
  components new TimerMilliC() as Timer3;
  components new TimerMilliC() as Timer4;
  components new TimerMilliC() as Timer5;
  components new TimerMilliC() as Timer6;
  components new TimerMilliC() as Timer7;
  components new TimerMilliC() as Timer8;
  components SerialPrintfC;
  components SerialStartC;
  components new AMSenderC(AM_RADIO_COUNT_MSG);
  components new AMReceiverC(AM_RADIO_COUNT_MSG);
  components ActiveMessageC;
  
  
  
  /****** INTERFACES *****/
  //Boot interface
  App.Boot -> MainC.Boot;
  
  /****** Wire the other interfaces down here *****/
  App.Receive -> AMReceiverC;
  App.AMSend -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.Timer0 -> Timer0;
  App.Timer1 -> Timer1;
  App.Timer2 -> Timer2;
  App.Timer3 -> Timer3;
  App.Timer4 -> Timer4;
  App.Timer5 -> Timer5;
  App.Timer6 -> Timer6;
  App.Timer7 -> Timer7;
  App.Timer8 -> Timer8;
  App.Packet -> AMSenderC;
  

}


