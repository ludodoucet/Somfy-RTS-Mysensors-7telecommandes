/*
  Somfy RTS remote control :

  Copyright (c) 2016, yogui
  
  code inspired from culfw http://culfw.de/culfw.html 
  and from this arduino forum thread : https://forum.arduino.cc/index.php?topic=208346.0
  
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
  Used In/Output :
  
  D00 : Not used
  D01 : Not used
  D02 : CC1101 : CSN/CS 
  D03 : CC1101 : GD00 Output for radio data 
  D04 : Remote Prog Button
  D05 : Remote Up Button
  D06 : Remote Down Button
  D07 : Remote My Button
  D08 : LED
  D09 : CC1101 : CE  
  D10 : not used
  D11 : CC1101 : MOSI
  D12 : CC1101 : MISO
  D13 : CC1101 : SCK 

  Analog inputs :
  
  A00 : not used
  A01 : not used
  A02 : not used
  A03 : not used
  A04 : not used
  A05 : not used
  A06 : not used
  A07 : not used
  
  Used In/Output Mega:
  
  D03 : Output for radio data 
  D53 : LED
  
  CC1101 pin layout : 
          ________
         |_       |		  
    VDD _|1|     2|_ VDD
         |¯	      |
	 SI _|3	     4|_ SCK
         |        |
	 SO _|5	     6|_ GDO2
         |        |
	CSn _|7	     8|_ GDO0
         |        |
	GND _|9	    10|_ GND
         |________|
		 
  Function description :

  setup your REMOTE_CONTROL_ADRESS and MyRolingCode 
  
  maybe you need to save your rolling code in the Eproom for my pergola its not needed 


  put your shutter in prog mode, your shutter should short goes down and up 
  send the prog command with this sketch the shutter goes again down and up 
  => you should be able to move your shutter with your arduino  
  
  Possible commande :

	SomfyCmd_My 		My			Stop or move to favourite position
	SomfyCmd_Up 		Up			Move up
	SomfyCmd_MyUp 		My + Up		Set upper motor limit in initial programming mode
	SomfyCmd_Down 		Down		Move down
	SomfyCmd_MyDown 	My + Down	Set lower motor limit in initial programming mode
	SomfyCmd_UpDown 	Up + Down	Change motor limit and initial programming mode
	SomfyCmd_Prog   	Prog		Used for (de-)registering remotes
	SomfyCmd_SunFlag 	Sun + Flag	Enable sun and wind detector (SUN and FLAG symbol on the Telis Soliris RC)
	SomfyCmd_Flag	 	Flag		Disable sun detector (FLAG symbol on the Telis Soliris RC)
  
  I have tested the code on an 
  Arduino nano with an Atmega328p
  Arduino Uno with an Atmega328p
  Arduino Pro mini 5.0V with an Atmega328p
  Arduino MEGA thanks @icare

 

   Version 1.1 add io mapping for analog input
   Version 1.0 Initial Version
 
 */



#include <somfy_rts.h>

#define PIN_PROG    4
#define PIN_UP      5
#define PIN_DOWN    6
#define PIN_MY      7



#define REMOTE_CONTROL_ADRESS  0x000001 // in hex
uint16_t MyRolingCode= 0x01;

//for TX the output 3 is used 
//for RX the analoge input 1 is used
//for the LED the ouput 8 is used 
//this is fixe define inside the lib 

static somfy_rts somfy;

uint8_t input_Down;
uint8_t input_Up;
uint8_t input_My;
uint8_t input_Prog;

uint8_t meminput_Down;
uint8_t meminput_Up;
uint8_t meminput_My;
uint8_t meminput_Prog;



void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_PROG,INPUT_PULLUP);
  pinMode(PIN_UP,INPUT_PULLUP);
  pinMode(PIN_DOWN,INPUT_PULLUP);
  pinMode(PIN_MY,INPUT_PULLUP);
  Serial.begin( 115200 );
   // Initialize CC1101
   somfy.beginCC1101(CC1101_TxPower_Plus7dBm);
}

void loop() {
      input_Prog = digitalRead(PIN_PROG);
      input_Down = digitalRead(PIN_DOWN);
      input_Up = digitalRead(PIN_UP);
      input_My = digitalRead(PIN_MY);
  
      //Prog
      if (input_Prog && !meminput_Prog  )
      {

        meminput_Prog =1;
        somfy.somfy_rts_func(SomfyCmd_Prog, MyRolingCode,REMOTE_CONTROL_ADRESS);
        MyRolingCode++;
        Serial.println( "Prog" );
        
      }
      else if (!input_Prog)
      {
        meminput_Prog =0;
      }
      
      //Down
      if (input_Down && !meminput_Down  )
      {

        meminput_Down =1;
        somfy.somfy_rts_func(SomfyCmd_Down, MyRolingCode,REMOTE_CONTROL_ADRESS);
        MyRolingCode++;
        Serial.println( "Down" );
        
      }
      else if (!input_Down)
      {
        meminput_Down =0;
      }

      // UP
      if (input_Up && !meminput_Up  )
      {

        meminput_Up =1;
        somfy.somfy_rts_func(SomfyCmd_Up, MyRolingCode,REMOTE_CONTROL_ADRESS);
        MyRolingCode++;
        Serial.println( "Up" );
      }
      else if (!input_Up)
      {
        meminput_Up =0;
      }  
      
      // My stop for me
      if (input_My && !meminput_My  )
      {

        meminput_My =1;
        somfy.somfy_rts_func(SomfyCmd_My, MyRolingCode,REMOTE_CONTROL_ADRESS);
        Serial.println( "My" );
        MyRolingCode++;
      }
      else if (!input_My)
      {
        meminput_My =0;
      }     
 
}
