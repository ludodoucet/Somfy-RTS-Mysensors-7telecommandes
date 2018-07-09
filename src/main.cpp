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
  D04 : Not used 
  D05 : Not used 
  D06 : Not used 
  D06 : Not used 
  D08 : LED
  D09 : NRF24L01+ Radio and CC1101 : CE  
  D10 : NRF24L01+ Radio            : CSN/CS  
  D11 : NRF24L01+ Radio and CC1101 : MOSI
  D12 : NRF24L01+ Radio and CC1101 : MISO
  D13 : NRF24L01+ Radio and CC1101 : SCK 

  Analog inputs :
  
  A00 : not used
  A01 : CC1001 : GD02 Input for radio data 
  A02 : not used
  A03 : not used
  A04 : not used
  A05 : not used
  A06 : not used
  A07 : not used

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

	SomfyCmd_My 		  My			    Stop or move to favourite position
	SomfyCmd_Up 		  Up			    Move up
	SomfyCmd_MyUp 		My + Up		  Set upper motor limit in initial programming mode
	SomfyCmd_Down 		Down		    Move down
	SomfyCmd_MyDown 	My + Down	  Set lower motor limit in initial programming mode
	SomfyCmd_UpDown 	Up + Down	  Change motor limit and initial programming mode
	SomfyCmd_Prog   	Prog		    Used for (de-)registering remotes
	SomfyCmd_SunFlag 	Sun + Flag	Enable sun and wind detector (SUN and FLAG symbol on the Telis Soliris RC)
	SomfyCmd_Flag	 	  Flag		    Disable sun detector (FLAG symbol on the Telis Soliris RC)
  
  I have tested the code on an 
  Arduino nano with an Atmega328p
  Arduino Uno with an Atmega328p
  Arduino Pro mini 5.0V with an Atmega328p



  Version 2.0
 
 */

/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* DESCRIPTION
* The ArduinoGateway prints data received from sensors on the serial link.
* The gateway accepts input on serial which will be sent out on radio network.
*
* The GW code is designed for Arduino Nano 328p / 16MHz
*
* Wire connections (OPTIONAL):
* - Inclusion button should be connected between digital pin 3 and GND
* - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
*
* LEDs (OPTIONAL):
* - To use the feature, uncomment any of the MY_DEFAULT_xx_LED_PINs
* - RX (green) - blink fast on radio message received. In inclusion mode will blink fast only on presentation received
* - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
* - ERR (red) - fast blink on error during transmission error or receive crc error
*
*/
#include <Arduino.h>
// Enable debug prints to serial monitor
#define MY_DEBUG


// Enable and select radio type attached
//#define MY_RADIO_NRF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level.
// #define MY_RF24_PA_LEVEL RF24_PA_LOW

// Enable serial gateway
#define MY_GATEWAY_SERIAL

// Define a lower baud rate for Arduinos running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
//#define MY_INCLUSION_BUTTON_FEATURE

// Inverses behavior of inclusion button (if using external pullup)
//#define MY_INCLUSION_BUTTON_EXTERNAL_PULLUP

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60
// Digital pin used for inclusion mode button
//#define MY_INCLUSION_MODE_BUTTON_PIN  3

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Inverses the behavior of leds
//#define MY_WITH_LEDS_BLINKING_INVERSE

// Flash leds on rx/tx/err
// Uncomment to override default HW configurations
//#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin
//#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin
//#define MY_DEFAULT_TX_LED_PIN  5  // the PCB, on board LED

#include <MySensors.h>

int CMD_BLADES[] = {1, 2, 3, 4, 5, 6, 7}


#include <somfy_rts.h>

#define PIN_PROG    4
#define PIN_UP      5
#define PIN_DOWN    6
#define PIN_MY      7
#define PIN_SEL     10
#define LED_1      A3
#define LED_2      A4
#define LED_3      A5


#include <EEPROM.h>

int EEPROM_ADDRESS[] = {0,1,2,3,4,5,6,7};
uint32_t REMOTE_CONTROL_ADRESS[] = { 0x000001,0x000002,0x000003,0x000004,0x000005,0x000006,0x000007};// in hex
uint16_t MyRolingCode[]= {0x01 , 0x01, 0x01 , 0x01 , 0x01 , 0x01 , 0x01};
uint16_t newRollingCode[]= {0x01 , 0x01 , 0x01 , 0x01 , 0x01 , 0x01 , 0x01};          // param (si EEPROM contient un code <)

//for TX the output 3 is used 
//for RX the analoge input 1 is used
//for the LED the ouput 8 is used 
//this is fixe define inside the lib 
static somfy_rts somfy;


uint8_t input_Down;
uint8_t input_Up;
uint8_t input_My;
uint8_t input_Prog;
uint8_t input_Sel;

uint8_t meminput_Down[9];
uint8_t meminput_Up[9];
uint8_t meminput_My[9];
uint8_t meminput_Prog[9];
uint8_t meminput_Sel;

//// pour appuie bouton Prog
long buttonTimer = 0;  // tempo BP
long longPressTime = 250;

boolean buttonActive = false;
boolean longPressActive = false;

//// pour écrire dans l'eeprom tout les 4 fois
int writeEeprom = 0 ;
int writeEepromStep = 4;

int counter = 0;

void setup()
{
	pinMode(PIN_PROG,INPUT_PULLUP);
	pinMode(PIN_UP,INPUT_PULLUP);
	pinMode(PIN_DOWN,INPUT_PULLUP);
	pinMode(PIN_MY,INPUT_PULLUP);
    pinMode(PIN_SEL,INPUT_PULLUP);

	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	pinMode(LED_3, OUTPUT);
	///test
	digitalWrite(LED_1, HIGH);
	digitalWrite(LED_2, HIGH);
	digitalWrite(LED_3, HIGH);
	delay(100);
	digitalWrite(LED_1, LOW);
	digitalWrite(LED_2, LOW);
	digitalWrite(LED_3, LOW);
	///test
	
	//Serial.print("Hello world.");
	// Initialize CC1101
	somfy.beginCC1101(CC1101_TxPower_Plus7dBm);
	// initialisation du rolling code
	for (int i = 0; i < 7; i++) {
        uint16_t code= 0x00;
		if (EEPROM.get(EEPROM_ADDRESS[i], code) < newRollingCode[i]) {
		EEPROM.put(EEPROM_ADDRESS[i], newRollingCode[i]);
	    }
    }
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Somfy RTS", "1.0");
  //Present Sensors-Commande
  present(CMD_BLADES[0], S_COVER);
  present(CMD_BLADES[1], S_COVER);
  present(CMD_BLADES[2], S_COVER);
  present(CMD_BLADES[3], S_COVER);
  present(CMD_BLADES[4], S_COVER);
  present(CMD_BLADES[5], S_COVER);
  present(CMD_BLADES[6], S_COVER);
}

void loop()
{
	
	input_Prog = digitalRead(PIN_PROG);
	input_Down = digitalRead(PIN_DOWN);
	input_Up = digitalRead(PIN_UP);
	input_My = digitalRead(PIN_MY);
	input_Sel = digitalRead(PIN_SEL);

	if(!input_Sel && !meminput_Sel)
	{
		meminput_Sel = 1;
		delay(50);  
		
		counter ++;
	
		//Reset count if over max mode number
		if(counter == 8)
		{
			counter = 1;
		}
	}
	else if (input_Sel){
		meminput_Sel = 0;
	}

	
		// counter = le numero du volet
		
		if (!input_Prog && !meminput_Prog[counter]) {
			meminput_Prog[counter] = 1;
			
			somfy.somfy_rts_func(SomfyCmd_Prog, MyRolingCode[counter - 1],REMOTE_CONTROL_ADRESS[counter - 1]);
			MyRolingCode[counter - 1] ++ ;
			writeEeprom ++;
			if (writeEeprom == 1) 
			{
				EEPROM.put(EEPROM_ADDRESS[counter - 1], MyRolingCode[counter - 1]);
			}
			if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}
			
						
		}
		else if (input_Prog) {
			meminput_Prog[counter] = 0;
		}

		if (!input_Down && !meminput_Down[counter]) {
			meminput_Down[counter] = 1;
			
			somfy.somfy_rts_func(SomfyCmd_Down, MyRolingCode[counter - 1],REMOTE_CONTROL_ADRESS[counter - 1]);
			MyRolingCode[counter - 1] ++ ;
			writeEeprom ++;
			if (writeEeprom == 1) 
			{
				EEPROM.put(EEPROM_ADDRESS[counter - 1], MyRolingCode[counter - 1]);
			}
			if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}
			}
		else if (input_Down) {
			meminput_Down[counter] = 0;
		}

		if (!input_Up && !meminput_Up[counter]) {
			meminput_Up[counter] = 1;
			
			somfy.somfy_rts_func(SomfyCmd_Up, MyRolingCode[counter - 1],REMOTE_CONTROL_ADRESS[counter - 1]);
			MyRolingCode[counter - 1] ++ ;
			writeEeprom ++;
			if (writeEeprom == 1) 
			{
				EEPROM.put(EEPROM_ADDRESS[counter - 1], MyRolingCode[counter - 1]);
			}
			if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}
			}
		else if (input_Up) {
			meminput_Up[counter] = 0;
		}

		if (!input_My && !meminput_My[counter]) {
			meminput_My[counter] = 1;
			
			somfy.somfy_rts_func(SomfyCmd_My, MyRolingCode[counter - 1],REMOTE_CONTROL_ADRESS[counter - 1]);
			MyRolingCode[counter - 1] ++ ;
			writeEeprom ++;
			if (writeEeprom == 1) 
			{
				EEPROM.put(EEPROM_ADDRESS[counter - 1], MyRolingCode[counter - 1]);
			}
			if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}
			}
		else if (input_My) {
			meminput_My[counter] = 0;
		}
		
		switch (counter) 
		{
			
			case 0:
			{
				digitalWrite(LED_1, LOW);
				digitalWrite(LED_2, LOW);
				digitalWrite(LED_3, LOW);
				break;
			}
			case 1:
			{
				digitalWrite(LED_1, LOW);
				digitalWrite(LED_2, LOW);
				digitalWrite(LED_3, HIGH);
				break;
			}
			case 2:
			{
				digitalWrite(LED_1, LOW);
				digitalWrite(LED_2, HIGH);
				digitalWrite(LED_3, LOW);
				break;
			}
			case 3:
			{
				digitalWrite(LED_1, LOW);
				digitalWrite(LED_2, HIGH);
				digitalWrite(LED_3, HIGH);
				break;
			}
			case 4:
			{
				digitalWrite(LED_1, HIGH);
				digitalWrite(LED_2, LOW);
				digitalWrite(LED_3, LOW);
				break;
			}
			case 5:
			{
				digitalWrite(LED_1, HIGH);
				digitalWrite(LED_2, LOW);
				digitalWrite(LED_3, HIGH);
				break;
			}
			case 6:  
			{
				
				digitalWrite(LED_1, HIGH);
				digitalWrite(LED_2, HIGH);
				digitalWrite(LED_3, LOW);
				break;
			}
			case 7:
			{
				
				digitalWrite(LED_1, HIGH);
				digitalWrite(LED_2, HIGH);
				digitalWrite(LED_3, HIGH);
				break;
			}
		}
		
	
}

void receive(const MyMessage &message)
{
	for (int i = 0; i < 7; i++) 
	{
		if (message.sensor ==  ( i + 1 ))
		{
			switch (message.type)
			{
			
				case V_VAR1:
				{
					somfy.somfy_rts_func(SomfyCmd_Prog, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);
					MyRolingCode[i]++;

					writeEeprom ++;
					if (writeEeprom == 1) 
					{
						EEPROM.put(EEPROM_ADDRESS[i], MyRolingCode[i]);
					}
					if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}
				

					break;
				}
				
				case V_UP:
				{
					somfy.somfy_rts_func(SomfyCmd_Up, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);
					
					MyRolingCode[i]++;

					writeEeprom ++;
					if (writeEeprom == 1) 
					{
						EEPROM.put(EEPROM_ADDRESS[i], MyRolingCode[i]);
					}
					if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}

					break;
				}
				case V_DOWN:
				{
					somfy.somfy_rts_func(SomfyCmd_Down, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);
					MyRolingCode[i]++;
					writeEeprom ++;
					if (writeEeprom == 1) 
					{
						EEPROM.put(EEPROM_ADDRESS[i], MyRolingCode[i]);
					}
					if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}					
					break;
				}
				case V_STOP:
				{
					somfy.somfy_rts_func(SomfyCmd_My, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);
					MyRolingCode[i]++;
					writeEeprom ++;
					if (writeEeprom == 1) 
					{
						EEPROM.put(EEPROM_ADDRESS[i], MyRolingCode[i]);
					}
					if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}							
					break;
				} 
				case V_SCENE_ON:
				{
					
					somfy.somfy_rts_func(SomfyCmd_Down, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);
					MyRolingCode[i]++;
					delay(15000);
					somfy.somfy_rts_func(SomfyCmd_Up, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);
					MyRolingCode[i]++;
					delay(10700);
					somfy.somfy_rts_func(SomfyCmd_My, MyRolingCode[i],REMOTE_CONTROL_ADRESS[i]);

					MyRolingCode[i]++;
					writeEeprom ++;
					if (writeEeprom == 1) 
					{
						EEPROM.put(EEPROM_ADDRESS[i], MyRolingCode[i]);
					}
					if (writeEeprom >= writeEepromStep) {writeEeprom = 0;}							
					
					break;
				}   
            
			}
		}
	}
		
}		