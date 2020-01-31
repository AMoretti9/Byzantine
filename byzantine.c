/* The planet orbit demonstration from the kilobotics-labs
 * https://www.kilobotics.com/labs#lab4-orbit
 *
 * Lightly modified to work in the simulator, in particular:
 * - mydata->variable for global variables
 * - callback function cb_botinfo() to report bot state back to the simulator for display
 * - spin-up motors only when required, using the helper function  smooth_set_motors()
 *
 * Modifications by Fredrik Jansson 2015
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <kilombo.h>

#include "byzantine.h"

#ifdef SIMULATOR
#include <stdio.h> // for printf
#else
#include <avr/io.h>  // for microcontroller register defs
//  #define DEBUG          // for printf to serial port
//  #include "debug.h"
#endif

REGISTER_USERDATA(USERDATA)

message_t message; //TX

int message_sent=0; //TX - variablile flag per tenere traccia della trasmissione dei messaggi
int new_message=0; //RX - variabile usata per indicare la presenza di nuovi messaggi (usata dal listener)


//VARIABILI DI SERVIZIO

int general; //uid of general
int traitor; //uid of traitor

int random_general;  // used to draw the general only one time
int random_traitor;  // used to draw the traitor only one time
int random_command; //used to draw the initial command
int generalElected; // uid of the kilobot chosen as general
int traitorElected; // uid of the kilobot chosen as traitor
int traitor_reverse; //used, by the traitor, to reverse the command only one time
int declare_command; //used by declare the command only one time
int generalCasualMovement ; //used to define the general movement, if it is the traitor

int time_to_move[4]; // used to define the round of movement for each kilobot

int G_commands[4]; // command to forward to other kilobot
int L1_commands[3]; // command to forward to other kilobot
int L2_commands[3]; // command to forward to other kilobot
int L3_commands[3]; // command to forward to other kilobot


// TRASMETTERE MESSAGGIO
message_t *message_tx() {
	return &message;
}

void message_tx_success(){
	message_sent=1;
}

//RICEVERE MESSAGGIO
void message_rx(message_t *message, distance_measurement_t *distance){
	if(kilo_uid==time_to_move[0]){
	new_message=time_to_move[0]+1;
	}
	
	if(kilo_uid==time_to_move[1]){
	new_message=time_to_move[1]+1;
	}
	
	if(kilo_uid==time_to_move[2]){
	new_message=time_to_move[2]+1;
	}
	
	if(kilo_uid==time_to_move[3]){
	new_message=time_to_move[3]+1;
	}
}

void smooth_set_motors(uint8_t ccw, uint8_t cw)
{
  // OCR2A = ccw;  OCR2B = cw;  
#ifdef KILOBOT 
  uint8_t l = 0, r = 0;
  if (ccw && !OCR2A) // we want left motor on, and it's off
    l = 0xff;
  if (cw && !OCR2B)  // we want right motor on, and it's off
    r = 0xff;
  if (l || r)        // at least one motor needs spin-up
    {
      set_motors(l, r);
      delay(15);
    }
#endif
  // spin-up is done, now we set the real value
  set_motors(ccw, cw);
}

void set_motion(motion_t new_motion)
{
  switch(new_motion) {
  case STOP:
    smooth_set_motors(0,0);
    break;
  case FORWARD:
    smooth_set_motors(kilo_straight_left, kilo_straight_right);
    break;
  case LEFT:
    smooth_set_motors(kilo_turn_left, 0); 
    break;
  case RIGHT:
    smooth_set_motors(0, kilo_turn_right); 
    break;
  }
}



void setup()
{
	
	message.type=NORMAL; //TX
	message.data[0] = NULL; message.data[1] = NULL; message.data[2] = NULL; message.data[3]=NULL; //TX - (contenuto dei messaggi)
	message.crc=message_crc(&message); //TX
	
	random_general=0;
	random_traitor=0;
	random_command=0;
	traitor_reverse=0;
	
	if (kilo_uid < 4)
    set_color(RGB(3,3,3));  // all kilobots start white
	
	
}

void loop() {   //32 ticks = 1 sec
	
	if(kilo_ticks<32){
		if(random_general==0){
			generalElected=general_draw();
			
		}
	}
	
	if(kilo_ticks>32 && kilo_ticks<96){
		if(kilo_uid==generalElected){
			set_color(RGB(0,0,3));
		}
	}
	
	if(kilo_ticks>96 && kilo_ticks<128){
		if(random_traitor==0){
			traitorElected=traitor_draw();
		}
	}
	
	if(kilo_ticks>128 && kilo_ticks<192){
		if(kilo_uid==traitorElected){
			set_color(RGB(3,2,0));
		}
	}
	
	if(kilo_ticks>192 && kilo_ticks<200){
		if(random_command==0){
			createGeneralCommands(general, traitor); // here the general creates the command to forward
		}
	}
	if(kilo_ticks>200 && kilo_ticks<216){
		
		message.data[1] = G_commands[1];
		message.data[2] = G_commands[2];
		message.data[3] = G_commands[3];
		
		
	}
	
	//=================================== MOVEMENT OF 1 (GENERAL)
	if(kilo_ticks>210 && kilo_ticks<252 ){   //+42
		if(kilo_uid==time_to_move[0]){
			set_motion(RIGHT);
		}
	}
	if(kilo_ticks>252 && kilo_ticks<341 ){  //+89
		if(kilo_uid==time_to_move[0]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>341 && kilo_ticks<441 ){ //+100 com
		if(kilo_uid==time_to_move[0]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){  //RX of L1
					new_message=0;
					L1_commands[0]=message.data[1];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[1]);
					printf("%d ___", L1_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
					new_message=0;
					L2_commands[0]=message.data[2];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[2]);
					printf("%d ___", L2_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[0]=message.data[3];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}		
	}
	
	if(kilo_ticks>441 && kilo_ticks<535 ){  //+94
		if(kilo_uid==time_to_move[0]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>535 && kilo_ticks<590 ){  //+55
		if(kilo_uid==time_to_move[0]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>590 && kilo_ticks<690 ){  // +100 com
		if(kilo_uid==time_to_move[0]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){  //RX of L1
					new_message=0;
					L1_commands[0]=message.data[1];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[1]);
					printf("%d ___", L1_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
					new_message=0;
					L2_commands[0]=message.data[2];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[2]);
					printf("%d ___", L2_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[0]=message.data[3];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
	}
	
	if(kilo_ticks>690 && kilo_ticks<817 ){  //+127
		if(kilo_uid==time_to_move[0]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>817 && kilo_ticks<877 ){  //+60
		if(kilo_uid==time_to_move[0]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>877 && kilo_ticks<1002 ){  //+125 com
		if(kilo_uid==time_to_move[0]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){  //RX of L1
					new_message=0;
					L1_commands[0]=message.data[1];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[1]);
					printf("%d ___", L1_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
					new_message=0;
					L2_commands[0]=message.data[2];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[2]);
					printf("%d ___", L2_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[0]=message.data[3];
					printf("\n___ K id %d  ___ receives 1st command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[0]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}	
	}
		
	if(kilo_ticks>1002 && kilo_ticks<1120 ){  //+118
		if(kilo_uid==time_to_move[0]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>1120 && kilo_ticks<1213 ){  //+93
		if(kilo_uid==time_to_move[0]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>1213 && kilo_ticks<1220 ){  //+7
		if(kilo_uid==time_to_move[0]){
			set_motion(STOP);
		}		
			
	}
	if(kilo_ticks>=1220 && kilo_ticks<1230){ // +10
		//IF L1 IS THE TRAITOR, NOW REVERSE THE COMMAND
		if(traitor==time_to_move[1]){
			
			if(traitor_reverse==0){
				L1_commands[0]=T_reverseCommand(L1_commands[0], time_to_move[1]);
			}
			
			message.data[1] = L1_commands[0];
			message.data[2] = NULL; message.data[3] = NULL; //unused
		} else{
			message.data[1] = L1_commands[0];
			message.data[2] = NULL; message.data[3] = NULL; //unused
		}
	}
	
	
		//=================================== MOVEMENT OF 2 (LIEUTENANT 1)
	if(kilo_ticks>1230 && kilo_ticks<1272 ){   //+42
		if(kilo_uid==time_to_move[1]){
			set_motion(RIGHT);
		}
	}
	if(kilo_ticks>1272 && kilo_ticks<1361 ){  //+89
		if(kilo_uid==time_to_move[1]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>1361 && kilo_ticks<1461 ){ //+100 com
		if(kilo_uid==time_to_move[1]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
					new_message=0;
					L2_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[2]);
					printf("%d ___", L2_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		
		
	}
	
	if(kilo_ticks>1461 && kilo_ticks<1555 ){  //+94
		if(kilo_uid==time_to_move[1]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>1555 && kilo_ticks<1610 ){  //+55
		if(kilo_uid==time_to_move[1]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>1610 && kilo_ticks<1710 ){  // +100 com
		if(kilo_uid==time_to_move[1]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
					new_message=0;
					L2_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[2]);
					printf("%d ___", L2_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		
		
		
	}
	
		if(kilo_ticks>1710 && kilo_ticks<1837 ){  //+127
		if(kilo_uid==time_to_move[1]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>1837 && kilo_ticks<1897 ){  //+60
		if(kilo_uid==time_to_move[1]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>1897 && kilo_ticks<1997 ){  //+125 com
		if(kilo_uid==time_to_move[1]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
					new_message=0;
					L2_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[2]);
					printf("%d ___", L2_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		
	}
	
	if(kilo_ticks>1897 && kilo_ticks<2015 ){  //+118
		if(kilo_uid==time_to_move[1]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>2015 && kilo_ticks<2108 ){  //+93
		if(kilo_uid==time_to_move[1]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>2108 && kilo_ticks<2115 ){  //+7
		if(kilo_uid==time_to_move[1]){
			set_motion(STOP);
			
			
		}
	}
		if(kilo_ticks>=2115 && kilo_ticks<2125){ // +10
		//IF L2 IS THE TRAITOR, NOW REVERSE THE COMMAND
		if(traitor==time_to_move[2]){
			
			if(traitor_reverse==0){
				L2_commands[0]=T_reverseCommand(L2_commands[0], time_to_move[2]);
			}
			
			message.data[1] = L2_commands[0];
			message.data[2] = NULL; message.data[3] = NULL; //unused
		} else{
			message.data[1] = L2_commands[0];
			message.data[2] = NULL; message.data[3] = NULL; //unused
		}
	}
	
		//=================================== MOVEMENT OF 3 (LIEUTENANT 2)
	if(kilo_ticks>2125 && kilo_ticks<2167 ){   //+42
		if(kilo_uid==time_to_move[2]){
			set_motion(RIGHT);
		}
	}
	if(kilo_ticks>2167 && kilo_ticks<2256 ){  //+89
		if(kilo_uid==time_to_move[2]){
			set_motion(FORWARD);
		}
	}

		if(kilo_ticks>2256 && kilo_ticks<2356 ){ //+100 com
		if(kilo_uid==time_to_move[2]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){ //RX of L1
					new_message=0;
					L1_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[1]);
					printf("%d ___", L1_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[2]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[2]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
	}
	
		if(kilo_ticks>2356 && kilo_ticks<2450 ){  //+94
		if(kilo_uid==time_to_move[2]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>2450 && kilo_ticks<2505 ){  //+55
		if(kilo_uid==time_to_move[2]){
			set_motion(FORWARD);
		}
	}
	
	if(kilo_ticks>2505 && kilo_ticks<2605 ){  // +100 com
		if(kilo_uid==time_to_move[2]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){ //RX of L1
					new_message=0;
					L1_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[1]);
					printf("%d ___", L1_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[2]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[2]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
			
	}
	
	if(kilo_ticks>2605 && kilo_ticks<2732 ){  //+127
		if(kilo_uid==time_to_move[2]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>2732 && kilo_ticks<2792 ){  //+60
		if(kilo_uid==time_to_move[2]){
			set_motion(FORWARD);
		}
	}
	
	if(kilo_ticks>2792 && kilo_ticks<2917 ){  //+125 com
		if(kilo_uid==time_to_move[2]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){ //RX of L1
					new_message=0;
					L1_commands[1]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[1]);
					printf("%d ___", L1_commands[1]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		if(kilo_uid==time_to_move[3]){
			if(new_message==time_to_move[3]+1){ //RX of L3
					new_message=0;
					L3_commands[2]=message.data[1];
					printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[3]);
					printf("%d ___", L3_commands[2]);
					printf(" at tick: %d", kilo_ticks);
					printf("\n");
			}
		}
		
	}
	
		if(kilo_ticks>2917 && kilo_ticks<3035 ){  //+118
		if(kilo_uid==time_to_move[2]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>3035 && kilo_ticks<3128 ){  //+93
		if(kilo_uid==time_to_move[2]){
			set_motion(FORWARD);
		}
	}
	if(kilo_ticks>3128 && kilo_ticks<3135 ){  //+7
		if(kilo_uid==time_to_move[2]){
			set_motion(STOP);
			
			
		}
	}

	if(kilo_ticks>=3135 && kilo_ticks<3145){ // +10
		//IF L3 IS THE TRAITOR, NOW REVERSE THE COMMAND
		if(traitor==time_to_move[3]){
			
			if(traitor_reverse==0){
				L3_commands[0]=T_reverseCommand(L3_commands[0], time_to_move[3]);
			}
			
			message.data[1] = L3_commands[0];
			message.data[2] = NULL; message.data[3] = NULL; //unused
		} else{
			message.data[1] = L3_commands[0];
			message.data[2] = NULL; message.data[3] = NULL; //unused
		}
	}
	
		//=================================== MOVEMENT OF 4 (LIEUTENANT 3)
	if(kilo_ticks>3145 && kilo_ticks<3187 ){   //+42
		if(kilo_uid==time_to_move[3]){
			set_motion(RIGHT);
		}
	}
	if(kilo_ticks>3187 && kilo_ticks<3276 ){  //+89
		if(kilo_uid==time_to_move[3]){
			set_motion(FORWARD);
		}
	}
	
	if(kilo_ticks>3276 && kilo_ticks<3376 ){ //+100 com
	if(kilo_uid==time_to_move[3]){
		set_motion(STOP);
		message.crc=message_crc(&message); //TX
	}
	
	if(kilo_uid==time_to_move[1]){
		if(new_message==time_to_move[1]+1){ //RX of L1
				new_message=0;
				L1_commands[2]=message.data[1];
				printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[1]);
				printf("%d ___", L1_commands[2]);
				printf(" at tick: %d", kilo_ticks);
				printf("\n");
		}
	}
	if(kilo_uid==time_to_move[2]){
		if(new_message==time_to_move[2]+1){ //RX of L2
				new_message=0;
				L2_commands[2]=message.data[1];
				printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[2]);
				printf("%d ___", L2_commands[2]);
				printf(" at tick: %d", kilo_ticks);
				printf("\n");
		}
	}
	}
	
	if(kilo_ticks>3376 && kilo_ticks<3470 ){  //+94
		if(kilo_uid==time_to_move[3]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>3470 && kilo_ticks<3525 ){  //+55
		if(kilo_uid==time_to_move[3]){
			set_motion(FORWARD);
		}
	}
	
	
	if(kilo_ticks>3525 && kilo_ticks<3625 ){  // +100 com
		if(kilo_uid==time_to_move[3]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){ //RX of L1
				new_message=0;
				L1_commands[2]=message.data[1];
				printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[1]);
				printf("%d ___", L1_commands[2]);
				printf(" at tick: %d", kilo_ticks);
				printf("\n");
			}
		}
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
				new_message=0;
				L2_commands[2]=message.data[1];
				printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[2]);
				printf("%d ___", L2_commands[2]);
				printf(" at tick: %d", kilo_ticks);
				printf("\n");
			}
		}
			
	}
	
	if(kilo_ticks>3625 && kilo_ticks<3752 ){  //+127
		if(kilo_uid==time_to_move[3]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>3752 && kilo_ticks<3812 ){  //+60
		if(kilo_uid==time_to_move[3]){
			set_motion(FORWARD);
		}
	}
	
	if(kilo_ticks>3812 && kilo_ticks<3937 ){  //+125 com
		if(kilo_uid==time_to_move[3]){
			set_motion(STOP);
			message.crc=message_crc(&message); //TX
		}
		
		
		if(kilo_uid==time_to_move[1]){
			if(new_message==time_to_move[1]+1){ //RX of L1
				new_message=0;
				L1_commands[2]=message.data[1];
				printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[1]);
				printf("%d ___", L1_commands[2]);
				printf(" at tick: %d", kilo_ticks);
				printf("\n");
			}
		}
		if(kilo_uid==time_to_move[2]){
			if(new_message==time_to_move[2]+1){ //RX of L2
				new_message=0;
				L2_commands[2]=message.data[1];
				printf("\n___ K id %d  ___ receives 2nd command ___ ", time_to_move[2]);
				printf("%d ___", L2_commands[2]);
				printf(" at tick: %d", kilo_ticks);
				printf("\n");
			}
		}
		
	}
	
	if(kilo_ticks>3937 && kilo_ticks<4055 ){  //+118
		if(kilo_uid==time_to_move[3]){
			set_motion(LEFT);
		}
	}
	if(kilo_ticks>4055 && kilo_ticks<4148 ){  //+93
		if(kilo_uid==time_to_move[3]){
			set_motion(FORWARD);
		}
	}
	
	if(kilo_ticks>4148 && kilo_ticks<4155 ){  //+7
		if(kilo_uid==time_to_move[3]){
			set_motion(STOP);
			
			
		}
	}
	
	if(kilo_ticks>4155 && kilo_ticks<4160){
	// decide to the GOAL TO PERFORM	
	// IF GENERAL IS LOYAL perform his command; IF GENERAL IS TRAITOR perform a casual command
	// LIEUTENANTS perform the command received in MAJORITY
	
	// ALL LOYALS KILOBOT, MUST PERFORM THE SAME ACTION
	
	if(declare_command==0){
		declareCommand();
	}
	
	}
	
	// ATTACK:   led green, turn around
	// RETREAT: led red, run away
	if(kilo_ticks>4160 && kilo_ticks< 4270){  
		//////////////////////////
		if(kilo_uid==time_to_move[0]){
			if(general==traitor){
				if(generalCasualMovement==1){
					set_color(RGB(0,3,0));
					set_motion(RIGHT);
				}else{
					set_color(RGB(3,0,0));
					set_motion(FORWARD);
				}
			}else{
				if(G_commands[0]==1){
					set_color(RGB(0,3,0));
					set_motion(RIGHT);
				}else{
					set_color(RGB(3,0,0));
					set_motion(FORWARD);
				}
			}
		}
		
		if(kilo_uid==time_to_move[1]){
			if((L1_commands[0]+L1_commands[1]+L1_commands[2]) <= 4){
				set_color(RGB(0,3,0));
				set_motion(RIGHT);
			}else{
				set_color(RGB(3,0,0));
				set_motion(FORWARD);
			}
		}
		
		if(kilo_uid==time_to_move[2]){
			if((L2_commands[0]+L2_commands[1]+L2_commands[2]) <= 4){
				set_color(RGB(0,3,0));
				set_motion(RIGHT);
			}else{
				set_color(RGB(3,0,0));
				set_motion(FORWARD);
			}
		}
		
		if(kilo_uid==time_to_move[3]){
			if((L3_commands[0]+L3_commands[1]+L3_commands[2]) <= 4){
				set_color(RGB(0,3,0));
				set_motion(RIGHT);
			}else{
				set_color(RGB(3,0,0));
				set_motion(FORWARD);
			}
		}
	
	

	}
	
	
	
	if(kilo_ticks>4270 && kilo_ticks<4900){
		if(kilo_uid==time_to_move[0]){
			if(general==traitor){
				if(generalCasualMovement==1){
					set_color(RGB(0,3,0));
					set_motion(RIGHT);
				}else{
					set_color(RGB(3,0,0));
					set_motion(STOP);
				}
			}else{
				if(G_commands[0]==1){
					set_color(RGB(0,3,0));
					set_motion(RIGHT);
				}else{
					set_color(RGB(3,0,0));
					set_motion(STOP);
				}
			}
		}
		
		if(kilo_uid==time_to_move[1]){
			if((L1_commands[0]+L1_commands[1]+L1_commands[2]) <= 4){
				set_color(RGB(0,3,0));
				set_motion(RIGHT);
			}else{
				set_color(RGB(3,0,0));
				set_motion(STOP);
			}
		}
		
		if(kilo_uid==time_to_move[2]){
			if((L2_commands[0]+L2_commands[1]+L2_commands[2]) <= 4){
				set_color(RGB(0,3,0));
				set_motion(RIGHT);
			}else{
				set_color(RGB(3,0,0));
				set_motion(STOP);
			}
		}
		
		if(kilo_uid==time_to_move[3]){
			if((L3_commands[0]+L3_commands[1]+L3_commands[2]) <= 4){
				set_color(RGB(0,3,0));
				set_motion(RIGHT);
			}else{
				set_color(RGB(3,0,0));
				set_motion(STOP);
			}
		}
		
		
		
	}
	
}

///////////////// END LOOP

int main() {
    kilo_init();
	
	kilo_message_tx=message_tx; //TX - register message_tx callback function
	kilo_message_tx_success = message_tx_success; //TX - register message_tx_success callback function
	
	kilo_message_rx = message_rx; //RX - register message_rx callback function
		
    kilo_start(setup, loop);
    return 0;
}

int general_draw(){
	
	printf("===============\nINFO: drawing general... \n===============\n");
	
	srand ( time(NULL) );
	
	general = rand() %4;
	random_general=1;
	printf("===============\nINFO: the GENERAL is the kilobot with uid: %d\n===============\n", general);
	
	time_to_move_impl(general);
	
	return general;
	
}

int traitor_draw(){
	
	printf("===============\nINFO: drawing general... \n===============\n");
	
	srand ( time(NULL) );
	
	traitor = rand() %5;
	random_traitor=1;
	
	if(traitor==4){
		printf("===============\nINFO: No kilobot as the traitor\n===============\n");
	} else{
		printf("===============\nINFO: the TRAITOR is the kilobot with uid: %d\n===============\n", traitor);
	}
	//traitor=general; // ********************* uncomment to trigger the case: general == traitor
	return traitor;
}

void time_to_move_impl(int general){
		
	if(general == 0){
		time_to_move[0]=0; time_to_move[1]=1; time_to_move[2]=2; time_to_move[3] = 3;
		printf("===============\nINFO: order of movements (kilo_uid): 0, 1, 2, 3\n===============\n");
		
	} else if (general==1){
		time_to_move[0]=1; time_to_move[1]=0; time_to_move[2]=2; time_to_move[3] = 3;
		printf("===============\nINFO: order of movements (kilo_uid): 1, 0, 2, 3\n===============\n");
		
	} else if (general==2){
		time_to_move[0]=2; time_to_move[1]=0; time_to_move[2]=1; time_to_move[3] = 3;
		printf("===============\nINFO: order of movements (kilo_uid): 2, 0, 1, 3\n===============\n");
		
	} else if (general==3){
		time_to_move[0]=3; time_to_move[1]=0; time_to_move[2]=1; time_to_move[3] = 2;
		printf("===============\nINFO: order of movements (kilo_uid): 3, 0, 1, 2\n===============\n");
			
	}
	
}

void createGeneralCommands(int general, int traitor){
	
		random_command=1;
		int new_command;
		
		if (general != traitor){ // if general is loyal, create only one order
		
			new_command=(rand() %2)+1;
			
			for(int count = 0; count<4; count++){
				G_commands[count]=new_command;
			}
		} else{ //else if general is the traitor, create 2 equal orders and 1 different
			
			for(int count = 0; count<2; count++){
				new_command=(rand() %2)+1;
				G_commands[count] = new_command;
			}
			
			if(G_commands[0] == G_commands[1]){
				new_command=(rand() %2)+1;
				G_commands[2]=new_command;
				G_commands[3]=G_reverseCommand(G_commands[2]);
			}else{
				G_commands[2]=G_commands[0];
				G_commands[3]=G_commands[0];
			}
			
		}

		printf("\n=============== Random general command... ===============");
		printf("\n\ncommand 0 = %d\n", G_commands[0]);
		printf("\n\ncommand 1 = %d\n", G_commands[1]);
		printf("\n\ncommand 2 = %d\n", G_commands[2]);
		printf("\n\ncommand 3 = %d\n", G_commands[3]);
		printf("\n=============== \n");
}

int G_reverseCommand(int command){
	if(command==1){
		return 2;
	}else{
		return 1;
	}
}

int T_reverseCommand(int command, int traitorID){
	
	traitor_reverse=1;
	
	if(command==1){
		printf("\n\n=*=*=*=*=*=*=*=*=*=* TRAITOR uid: %d,  reverse the command 1  -->  2 =*=*=*=*=*=*=*=*=*=*\n\n", traitorID);
		return 2;
	}else{
		printf("\n\n=*=*=*=*=*=*=*=*=*=* TRAITOR uid: %d,  reverse the command 2  -->  1 =*=*=*=*=*=*=*=*=*=*", traitorID);
		return 1;
	}
}

void declareCommand(){
	
	declare_command = 1;
	
	printf("\n\n=============== COMMANDS ===============\n");
	
	if (general==traitor){
		printf("\n   ||      GENERAL:       I'm the traitor (do casual command)");
		
		generalCasualMovement = (rand() %2)+1;
		
	}else{
		if(G_commands[0]==1){
			printf("\n   ||      GENERAL:       ATTACK!");
		}else{
			printf("\n   ||      GENERAL:       RETREAT!");
		}
	}
	
	if((L1_commands[0]+L1_commands[1]+L1_commands[2]) <= 4){
		printf("\n   ||      LIEUTENANT 1:  ATTACK!");
	}else{
		printf("\n   ||      LIEUTENANT 1:  RETREAT!");
	}
	
	if((L2_commands[0]+L2_commands[1]+L2_commands[2]) <= 4){
		printf("\n   ||      LIEUTENANT 2:  ATTACK!");
	}else{
		printf("\n   ||      LIEUTENANT 2:  RETREAT!");
	}
	
	if((L3_commands[0]+L3_commands[1]+L3_commands[2]) <= 4){
		printf("\n   ||      LIEUTENANT 3:  ATTACK!");
	}else{
		printf("\n   ||      LIEUTENANT 3:  RETREAT!");
	}
	
		printf("\n\n========================================\n\n");

	
}