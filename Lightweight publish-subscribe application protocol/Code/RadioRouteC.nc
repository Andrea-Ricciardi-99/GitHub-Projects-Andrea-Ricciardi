
#include "Timer.h"
#include "RadioRoute.h"
#include <string.h>
#include "printf.h" //used to implement the printing for the COOJA simulation


module RadioRouteC @safe() {
  uses {
  
    /****** INTERFACES *****/
	interface Boot;

    interface Receive;                      //interfaces for communication
    interface AMSend; 						
	interface Timer<TMilli> as Timer0;      //interface for timers 
    interface Timer<TMilli> as Timer1;	
    interface Timer<TMilli> as Timer2;	
    interface Timer<TMilli> as Timer3;
    interface Timer<TMilli> as Timer4;
    interface Timer<TMilli> as Timer5;
    interface Timer<TMilli> as Timer6;
    interface Timer<TMilli> as Timer7;
    interface Timer<TMilli> as Timer8;				
    interface Packet;						//other interfaces
    interface SplitControl as AMControl;
  }
}
implementation {

  message_t packet;
  
  // Variables to store the message to send
  message_t queued_packet;								
  uint16_t queue_addr;
  uint16_t time_delays[9]={0,0,0,0,0,0,0,0,0}; //Time delay in milli seconds
  message_t queue1[3]; //Array used to store the data messages, which are due to PUB messages to the 1st topic
  message_t queue2[3]; //Array used to store the data messages, which are due to PUB messages to the 2nd topic
  message_t queue3[2]; //Array used to store the data messages, which are due to PUB messages to the 3rd topic
  int currentElement1 = 0; //Index used to scroll the queue1 array
  int currentElement2 = 0; //Index used to scroll the queue2 array
  int currentElement3 = 0; //Index used to scroll the queue3 array
  bool r_con=FALSE; //Variable used to signal if it is needed to retransmit a CONNECT message for a specific node
  bool r_sub=FALSE; //Variable used to signal if it is needed to retransmit a SUBSCRIBE message for a specific node
  radio_route_msg_t* rcm2; //Pointer used in the receive event
  message_t mex1; //Variable used to store a single message in the queue1 array
  message_t mex2; //Variable used to store a single message in the queue2 array
  message_t mex3; //Variable used to store a single message in the queue3 array
  					
  uint8_t j; //INdex used to scroll
  int topic1 [8] = {0}; // List of nodes subscribed to the first topic
  int topic2 [8] = {0}; // List of nodes subscribed to the second topic
  int topic3 [8] = {0}; // List of nodes subscribed to the third topic
  uint8_t dest;
  
  bool locked;
  
  bool actual_send (uint16_t address, message_t* packet);
  bool generate_send (uint16_t address, message_t* packet, uint8_t type);
  
  
  
  
  
  
  bool generate_send (uint16_t address, message_t* packet, uint8_t type){
  /*
  * 
  * Function to be used when performing the send.
  * It stores the packet and address into a global variable and start the timer execution to schedule the send.
  * It allows the sending of only one message for each type
  * @Input:
  *		address: packet destination address
  *		packet: full packet to be sent (Not only Payload)
  *		type: payload message type
  *
  */
  	if (call Timer0.isRunning()){
  		return FALSE;
  	}else{
  	if (type == 1){
  		queued_packet = *packet;
  		queue_addr = address;  		
  		call Timer0.startOneShot( time_delays[TOS_NODE_ID-1] );
  	}else if (type == 2){
  		queued_packet = *packet;
  		queue_addr = address;
  		call Timer0.startOneShot( time_delays[TOS_NODE_ID-1] );
  	}else if (type == 0){
  		queued_packet = *packet;
  		queue_addr = address;	
  		call Timer0.startOneShot( time_delays[TOS_NODE_ID-1] );
  	
  	}else if (type == 3){
  		queued_packet = *packet;
  		queue_addr = address;
  		call Timer0.startOneShot( time_delays[TOS_NODE_ID-1] );
  	}
  	}
  	return TRUE;
  	}
  
  event void Timer0.fired() {
  	/*
  	* Timer triggered to perform the send.
  	*/
  	actual_send (queue_addr, &queued_packet);
  }
  
  bool actual_send (uint16_t address, message_t* packet){
	/*
	* Checks if the radio is being used through the variable locked, then, if not, it sends the packet through the function AMSend.send and put locked to true
	*/
	if (locked) {
		dbg("radio_rec", "locked\n");
      	return;
    }else {
      	radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(packet, sizeof(radio_route_msg_t));
    	if (rcm == NULL) {
			return;
		}
		if (call AMSend.send(address, packet, sizeof(radio_route_msg_t)) == SUCCESS) {
			dbg("radio_send", "Sending packet");	
			locked = TRUE;
			dbg_clear("radio_send", " at time %s \n", sim_time_string());
      }
    }
    
  }
  
  
  event void Boot.booted() {
    dbg("boot","Application booted.\n");
    /* Starts the radio */
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
	/* Starts the timer 1 immediately, the timer 3 after 50 seconds and then timer 4 every 120 seconds  */
	 if (err == SUCCESS) {
      dbg("radio","Radio on on node %d!\n", TOS_NODE_ID);
      call Timer1.startOneShot(0); // Connect
      call Timer3.startOneShot(50000); // Subscribe
      call Timer4.startPeriodic(120000); // Publish
    }
    else {
      dbgerror("radio", "Radio failed to start, retrying...\n");
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    /* Stopping the radio */
    dbg("boot", "Radio stopped!\n");
  }
  
  event void Timer1.fired() {
  //Timer needed to script the sending of all the connection messages by all nodes to the PANC (Node 1)
  radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&packet, sizeof(radio_route_msg_t));
  uint32_t backoffDelay=0;
	if (TOS_NODE_ID!=1){
          //CONNECTION LOGIC FOR THE NODES:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 0;
	      dbg("radio_rec","Source of the message: %d \n",rcm->source);
	      printf("Source of the message: %d \n",rcm->source);
	      printfflush();
	      backoffDelay = TOS_NODE_ID*1000 +2000 ; //Specific delay for each node (apart from the PANC) which allows not to have collisions among the CONNECT messages 
	      call Timer2.startOneShot(backoffDelay);
	      
     }
    }
	 

  event void Timer2.fired(){
   //Timer needed to generate the sending of messages to the PANC. This has been called in the first 3 scripted timers (Timer 1, 3 and 4) 
    radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&packet, sizeof(radio_route_msg_t));
    printf("Timer 2 working \n");
    printfflush();
    generate_send(1,&packet,rcm->type);
    if (rcm->type!=2){call Timer8.startOneShot(40000);} //Timer8 is started for each CONNECT and SUB message (on each Node) to check whether if it is needed to retransmit the message 
    dbg("radio_rec","Message Source: %d \n",rcm->source);
    printf("Message Source: %d \n",rcm->source);
    printfflush();
    dbg("radio_rec","Message Destination : %d \n",rcm->destination);
    printf("Message Destination : %d \n",rcm->destination);
    printfflush();
    dbg("radio_rec","Message Type: %d \n",rcm->type);
    printf("Message Type: %d \n",rcm->type);
    printfflush();
    if (rcm->type==0){r_con=TRUE;}  //If we perform the send of a CONNECT message we set this variable to TRUE as to eventually signal that a retransmission is needed
    if (rcm->type==1){
    dbg("radio_rec","Message Topic: %d \n",rcm->topic_name);
    printf("Message Topic: %d \n",rcm->topic_name);
    printfflush();
    r_sub=TRUE; //If we perform the send of a SUBSCRIBE message we set this variable to TRUE as to eventually signal that a retransmission is needed
    }
    else if (rcm->type==2){
    dbg("radio_rec","Message Topic: %d \n",rcm->topic_name);
    printf("Message Topic: %d \n",rcm->topic_name);
    printfflush();
    dbg("radio_rec","Message Payload: %d \n",rcm->payload);
    printf("Message Payload: %d \n",rcm->payload);
    printfflush();
    }
    }
    
  event void Timer3.fired() {
  //Timer needed to script the sending of all the subscription messages by all nodes to the PANC (Node 1)
  radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&packet, sizeof(radio_route_msg_t));
  uint32_t backoffDelay=0;
	if (TOS_NODE_ID==2 || TOS_NODE_ID==3 || TOS_NODE_ID==4){
          //SUBSCRIPTION LOGIC FOR THE NODES 1,2 AND 3:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 1;
	      rcm->topic_name=1;
	      dbg("radio_rec","Source of the message: %d \n",rcm->source);
	      printf("Node %d has sent a sub request \n",rcm->source);
	      printfflush();
	      backoffDelay = TOS_NODE_ID*1000 +2000 ; //Specific delay for nodes 2, 3 and 4 which allows not to have collisions among the SUBSCRIBE messages 
	      printf("Node %d starting timer of %u\n",TOS_NODE_ID,backoffDelay);
	      call Timer2.startOneShot(backoffDelay); 
     }else if (TOS_NODE_ID==5 || TOS_NODE_ID==6 || TOS_NODE_ID==7){
          //SUBSCRIPTION LOGIC FOR THE NODES 5, 6 AND 7:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 1;
	      rcm->topic_name=2;
	      dbg("radio_rec","Source of the message: %d \n",rcm->source);
	      printf("Node %d has sent a sub request \n",rcm->source);
	      printfflush();
	      backoffDelay = TOS_NODE_ID*1000 +2000 ; //Specific delay for nodes 5, 6 and 7 which allows not to have collisions among the SUBSCRIBE messages
	      printf("Node %d starting timer of %d\n",TOS_NODE_ID,backoffDelay);
	      call Timer2.startOneShot(backoffDelay);  
     }else if (TOS_NODE_ID==8 || TOS_NODE_ID==9){
          //SUBSCRIPTION LOGIC FOR THE NODES 8 AND 9:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 1;
	      rcm->topic_name=3;
	      dbg("radio_rec","Source of the message: %d \n",rcm->source);
	      printf("Node %d has sent a sub request \n",rcm->source);
	      printfflush();
	      backoffDelay = TOS_NODE_ID*1000 +2000 ; //Specific delay for nodes 8 and 9 which allows not to have collisions among the SUBSCRIBE messages
	      printf("Node %d starting timer of %d\n",TOS_NODE_ID,backoffDelay);
	      call Timer2.startOneShot(backoffDelay);    
     }
    }
    
  event void Timer4.fired() {
  //Timer needed to script the sending of all the publication messages by all nodes to the PANC (Node 1)
  uint32_t backoffDelay=0;
  radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&packet, sizeof(radio_route_msg_t));
  currentElement1 = 0; //These three variables are reset to 0 each time this timer is fired
  currentElement2 = 0;
  currentElement3 = 0;
	if (TOS_NODE_ID==2){
          //PUBLICATION LOGIC FOR NODE 2:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 2;
	      rcm->topic_name=1;
	      rcm->payload=(-10) + rand() % (60 - (-10) + 1); //Random payload for Temperature messages
	      dbg("radio_rec","Payload of the message: %d \n",rcm->payload);
	      backoffDelay = TOS_NODE_ID*1000 +2000 ;  //Specific delay for node 2 which allows not to have collisions among the PUBLISH messages
	      call Timer2.startOneShot(backoffDelay);
	      
     }else if (TOS_NODE_ID==6){
          //PUBLICATION LOGIC FOR NODE 6:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 2;
	      rcm->topic_name=2;
	      rcm->payload= 0 + rand() % (100 - 0 + 1); //Random payload for Humidity messages
	      dbg("radio_rec","Payload of the message: %d \n",rcm->payload);
	      backoffDelay = TOS_NODE_ID*1000 +2000 ; //Specific delay for node 6 which allows not to have collisions among the PUBLISH messages
	      call Timer2.startOneShot(backoffDelay);
	      
     }else if (TOS_NODE_ID==9){
          //PUBLICATION LOGIC FOR NODE 9:		
          rcm->source = TOS_NODE_ID;
	      rcm->destination = 1;
	      rcm->type = 2;
	      rcm->topic_name=3;
	      rcm->payload= 0 + rand() % (100 - 0 + 1); //Random payload for Luminosity messages
	      dbg("radio_rec","Payload of the message: %d \n",rcm->payload);
	      backoffDelay = TOS_NODE_ID*1000 +2000 ; //Specific delay for node 9 which allows not to have collisions among the PUBLISH messages
	      call Timer2.startOneShot(backoffDelay);
	      
     }
    }
    
    event void Timer5.fired(){
    //Timer needed to manage the sending of data messages to the nodes subscribed to topic 1 by node 1 after having received a publish message on topic 1
    if (currentElement1<3){
    radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&queue1[currentElement1], sizeof(radio_route_msg_t));
    dbg("radio_rec","The message sent to node %d is of type %d \n",rcm->destination,rcm->type);
    dbg("radio_rec","The message payload is %d \n",rcm->payload);
    printf("| The message sent to node %d is of type %d |\n",rcm->destination,rcm->type);
    printfflush();
    printf("| The message payload is %d |\n",rcm->payload);
    printfflush();
    generate_send(rcm->destination,&queue1[currentElement1],3);
    currentElement1++;
    	if (currentElement1<3){
    	call Timer5.startOneShot(100);
    	}
    }
    }
    
    event void Timer6.fired(){
    //Timer needed to manage the sending of data messages to the nodes subscribed to topic 2 by node 1 after having received a publish message on topic 2
    if (currentElement2<3){
    radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&queue2[currentElement2], sizeof(radio_route_msg_t));
    dbg("radio_rec","The message sent to node %d is of type %d \n",rcm->destination,rcm->type);
    dbg("radio_rec","The message payload is %d \n",rcm->payload);
    printf("| The message sent to node %d is of type %d |\n",rcm->destination,rcm->type);
    printfflush();
    printf("| The message payload is %d |\n",rcm->payload);
    printfflush();
    generate_send(rcm->destination,&queue2[currentElement2],3);
    currentElement2++;
    	if (currentElement2<3){
    	call Timer6.startOneShot(100);
    	}
    }
    }
    
    event void Timer7.fired(){
    //Timer needed to manage the sending of data messages to the nodes subscribed to topic 3 by node 1 after having received a publish message on topic 3
    if (currentElement3<2){
    radio_route_msg_t* rcm = (radio_route_msg_t*)call Packet.getPayload(&queue3[currentElement3], sizeof(radio_route_msg_t));
    dbg("radio_rec","The message sent to node %d is of type %d \n",rcm->destination,rcm->type);
    dbg("radio_rec","The message payload is %d \n",rcm->payload);
    printf("| The message sent to node %d is of type %d |\n",rcm->destination,rcm->type);
    printfflush();
    printf("| The message payload is %d |\n",rcm->payload);
    printfflush();
    generate_send(rcm->destination,&queue3[currentElement3],3);
    currentElement3++;
    	if (currentElement3<2){
    	call Timer7.startOneShot(100);
    	}
    }
    }
    
    event void Timer8.fired(){
    //Timer used to implement the retransmission of connect and subscribe messages
    if (r_con==TRUE){
		dbg("radio_rec","Node %d is retransmitting a CONNECT message to PANC \n",TOS_NODE_ID);
		printf("Node %d is retransmitting a CONNECT message to PANC \n",TOS_NODE_ID);
		printfflush();
		call Timer1.startOneShot(0);
    }else if (r_sub==TRUE){
        dbg("radio_rec","Node %d is retransmitting a SUBSCRIBE message to PANC \n",TOS_NODE_ID);
		printf("Node %d is retransmitting a SUBSCRIBE message to PANC \n",TOS_NODE_ID);
		printfflush();
    	call Timer3.startOneShot(0);
    }
    }    

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
	if (len != sizeof(radio_route_msg_t)) {return bufPtr;}                  
    else {
      radio_route_msg_t* rcm = (radio_route_msg_t*)payload;                  
      
      dbg("radio_rec", "Received packet at time %s\n", sim_time_string());
      dbg("radio_pack",">>>Pack \n \t Payload length %hhu \n", call Packet.payloadLength( bufPtr ));
      printf("| Received packet |\n");
      printfflush();
      
      if (TOS_NODE_ID == 1){
        switch(rcm->type){  //Switch used to deal with the three different types of radio_route_msg_t "type" field differently
          case 0: //Case in which the PANC receives CONNECT messages by other nodes. It performs the sending of the CONNACK message to the node which has previously sent the CONNECT msg
                rcm2 = (radio_route_msg_t*)call Packet.getPayload(bufPtr, sizeof(radio_route_msg_t));             
                dbg("radio_rec", "Received CONNECT message by PANC from node %d \n",rcm->source);
                printf("| Received CONNECT message by PANC from node %d |\n",rcm->source);
                printfflush();
                dest=rcm2->source;
                rcm2->source = TOS_NODE_ID;
      			rcm2->destination = dest;
                generate_send(rcm->destination,bufPtr, 0);
                dbg("radio_rec","PANC is sending CONNACK to node %d \n", rcm2->destination);
                printf("| PANC is sending CONNACK to node %d |\n", rcm2->destination);
                printfflush();
             
                
              break;
              
          case 1: //Case in which the PANC receives SUB messages by other nodes. It performs the sending of the SUBACK message to the node which has previously sent the SUB msg
                rcm2 = (radio_route_msg_t*)call Packet.getPayload(bufPtr, sizeof(radio_route_msg_t));
                dbg("radio_rec", "Received SUBSCRIBE message by PANC from node %d \n",rcm->source);
                printf("| Received SUBSCRIBE message by PANC from node %d |\n",rcm->source);
                printfflush();
                dest=rcm->source;
                rcm2->source = TOS_NODE_ID;
      			rcm2->destination = dest;
      			rcm2->type = 0;
                generate_send(dest,bufPtr, 0);
                dbg("radio_rec","PANC is sending SUBACK to node %d \n", dest);
                printf("| PANC is sending SUBACK to node %d |\n", dest);
                printfflush();
                switch(rcm->topic_name){ //Switch used to update the arrays that save the ID of the subscribed nodes to a specific topic
                  case 1:
                  for ( j=0; j<7; j++ ){
                    if (topic1[j]==0){
                      topic1[j] = dest;
                      j=8;
                     }
                     }
                  break;
                  case 2:
                  for ( j=0; j<7; j++ ){
                    if (topic2[j]==0){
                      topic2[j] = dest;
                      j=8;
                      }
                      }
                  break;
                  case 3:
                  for ( j=0; j<7; j++ ){
                    if (topic3[j]==0){
                      topic3[j] = dest;
                      j=8;
                      }
                      }
                  break;
                  } 
           break;
           
           case 2: //Case in which the PANC receives PUB messages by other nodes. It performs the sending of data messages to the nodes subscribed to the specified topic 
                dbg("radio_rec", "Received PUBLISH message on topic %d by PANC from node %d\n",rcm->topic_name,rcm->source);
                printf("| Received PUBLISH message on topic %d by PANC from node %d with payload %d|\n",rcm->topic_name,rcm->source,rcm->payload);
                printfflush();
                rcm2 = (radio_route_msg_t*)call Packet.getPayload(bufPtr, sizeof(radio_route_msg_t));
                switch (rcm->topic_name){ //Switch used to create, save and send the data messages to the nodes subscribed to the specified topic
                case 1:
                    for (j=0;j<8; j++){
                      if (topic1[j]!=0){
      			        radio_route_msg_t* rcm3 = (radio_route_msg_t*)call Packet.getPayload(&mex1, sizeof(radio_route_msg_t));
      			        rcm3->type=3;
      			        rcm3->source= TOS_NODE_ID;
      			        rcm3->destination= topic1[j];
      			        rcm3->topic_name= 1;
      			        rcm3->payload= rcm2->payload;
      			        dbg("radio_rec","Publish Message Type: %d \n",rcm3->type);
						queue1[j]= mex1; //The data message is saved in position j of the queue1 array. The sending will be performed in the Timer5
                      }
                      else if (topic1[j]==0){
                        j=8;
                      }
                     }
                     call Timer5.startOneShot(0);
                 break;
                 case 2:
                    for (j=0;j<8; j++){
                      if (topic2[j]!=0){
      			       radio_route_msg_t* rcm3 = (radio_route_msg_t*)call Packet.getPayload(&mex2, sizeof(radio_route_msg_t));
      			        rcm3->type=3;
      			        rcm3->source= TOS_NODE_ID;
      			        rcm3->destination= topic2[j];
      			        rcm3->topic_name= 2;
      			        rcm3->payload= rcm2->payload;
      			        dbg("radio_rec","Publish Message Type: %d \n",rcm3->type);
						queue2[j]= mex2; //The data message is saved in position j of the queue2 array. The sending will be performed in the Timer6
                      }
                      else if (topic2[j]==0){
                        j=8;
                      }
                     }
                     call Timer6.startOneShot(0);
                  break;
                  case 3:
                    for (j=0;j<8; j++){
                      if (topic3[j]!=0){
      			        radio_route_msg_t* rcm3 = (radio_route_msg_t*)call Packet.getPayload(&mex3, sizeof(radio_route_msg_t));
      			        rcm3->type=3;
      			        rcm3->source= TOS_NODE_ID;
      			        rcm3->destination= topic3[j];
      			        rcm3->topic_name= 3;
      			        rcm3->payload= rcm2->payload;
      			        dbg("radio_rec","Publish Message Type: %d \n",rcm3->type);
						queue3[j]= mex3; //The data message is saved in position j of the queue3 array. The sending will be performed in the Timer7
                      }
                      else if (topic3[j]==0){
                        j=8;
                      }
                     }
                     call Timer7.startOneShot(0);
                  break;
                    } 
                  break; 
                  case 3: //Case in which the PANC receives data messages. It will never be triggered
                    dbg("radio_rec", "The PANC has received data message \n");
                  break;
                  }
                  }
                  else{ //Cases in which the node to receive a generic message is NOT the PANC
                  if (rcm->type!=3){ //Case in which a generic node (not the PANC) receives a CONNACK or a SUBACK message by PANC
                  dbg("radio_rec","Node %d has received a message of type %d \n",TOS_NODE_ID,rcm->type);
                  printf("Node %d has received a message of type %d \n",TOS_NODE_ID,rcm->type);
                  printfflush();
                  r_con=FALSE; //The CONNECT retransmission variable is set to false as to signal that the specific node has NOT to retransmit the CONNECT message to PANC
                  r_sub=FALSE; //The SUB retransmission variable is set to false as to signal that the specific node has NOT to retransmit the SUB message to PANC
                  }else { //Case in which a generic node (not the PANC) receives a data message by PANC
                  dbg("radio_rec","Node %d has received a message with payload %d \n",TOS_NODE_ID,rcm->payload);
                  printf("Node %d has received a message with payload %d \n",TOS_NODE_ID,rcm->payload);
                  printfflush();
                  }
                  } 
                  }
                  return bufPtr;   
                  }           

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
	/* This event is triggered when a message is sent 
	*/ 
	if (&queued_packet == bufPtr) {		// If the send has been done and the sent message was the one that was waiting the timer 0 to be fired then we set the locked variable to false, 										allowing other messages to be sent
      locked = FALSE;
      dbg("radio_send", "Packet sent...");
      dbg_clear("radio_send", " at time %s \n", sim_time_string());
    }else{dbg("radio_rec","Unable to send anything \n");}
  }
 }





