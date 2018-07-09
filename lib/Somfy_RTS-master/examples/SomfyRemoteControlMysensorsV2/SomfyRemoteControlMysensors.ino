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
  D02 : Not used 
  D03 : Output for radio data 
  D04 : Not used 
  D05 : Not used 
  D06 : Not used 
  D07 : Not used 
  D08 : LED
  D09 : NRF24L01+ Radio : CE  
  D10 : NRF24L01+ Radio : CSN/CS  
  D11 : NRF24L01+ Radio : MOSI
  D12 : NRF24L01+ Radio : MISO
  D13 : NRF24L01+ Radio : SCK 

  Analog inputs :
  
  A00 : not used
  A01 : not used
  A02 : not used
  A03 : not used
  A04 : not used
  A05 : not used
  A06 : not used
  A07 : not used
  
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

  note :
      /\
     /  \  
    /    \    for 16 MHz arduino you should dived per 2 the serial speed. I clock down the CPU to 8MHz => 57600 for default mysensors
   /   !  \
  /________\

   Version 1.1 add rolling code management, add io mapping for analog input, disable repeater mode
   Version 1.0 Initial Version
 
 */



#include <somfy_rts.h>
// Mysensors libs
#include <SPI.h>
#include <MySensor.h>
//#define WITH_PROG_COMMANDE //Enable this line if you need the 

#ifdef WITH_PROG_COMMANDE
  #define CMD_PROG       2
#endif
#define CMD_SHUTTER      1



#define REMOTE_CONTROL_ADRESS  0x000001 // in hex
uint16_t MyRolingCode= 0x01;

//for TX the output 3 is used 
//for RX the analoge input 1 is used
//for the LED the ouput 8 is used 
//this is fixe define inside the lib 


static somfy_rts somfy;

// Construct MySensors library
MySensor gw;

#ifdef WITH_PROG_COMMANDE
  MyMessage msgProg(CMD_PROG, V_VAR1);
#endif


void setup() {

 
  // Initialize library and add callback for incoming messages
  gw.begin(incomingMessage,10, false);   
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Somfy RTS", "1.0");
  //Present Sensors-Commande
#ifdef WITH_PROG_COMMANDE
  gw.present(CMD_PROG, S_CUSTOM);
#endif

  gw.present(CMD_SHUTTER, S_COVER);

  
}

void loop() {
   // Always process incoming messages whenever possible
  gw.process();
  
}


void incomingMessage(const MyMessage &message) 
{

  
  // We only expect one type of message from controller. But we better check anyway.
  if (    message.sensor ==  CMD_SHUTTER
#ifdef CMD_PROG  
      ||  message.sensor ==  CMD_PROG
#endif      
     ) 
  {
      
    switch (message.type)
    {
      
  
 #ifdef WITH_PROG_COMMANDE
        case V_VAR1:
        {
            somfy.somfy_rts_func(SomfyCmd_Prog, MyRolingCode,REMOTE_CONTROL_ADRESS);
            MyRolingCode++;
            break;
        }
#endif
       case V_UP:
        {
            somfy.somfy_rts_func(SomfyCmd_Up, MyRolingCode,REMOTE_CONTROL_ADRESS);
            Serial.println("UP");
            MyRolingCode++;
            break;
        }
       case V_DOWN:
        {
            somfy.somfy_rts_func(SomfyCmd_Down, MyRolingCode,REMOTE_CONTROL_ADRESS);
            MyRolingCode++;
            break;
        }
        case V_STOP:
        {
            somfy.somfy_rts_func(SomfyCmd_My, MyRolingCode,REMOTE_CONTROL_ADRESS);
            MyRolingCode++;
            break;
        }       
     }
    
   } 
}
