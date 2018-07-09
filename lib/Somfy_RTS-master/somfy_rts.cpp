/*
 * Somfy RTS communication protocol
 * 
 * Copyright (c) 2017, yogui
 * 
 * code inspired from culfw http://culfw.de/culfw.html 
 * and from this arduino forum thread : https://forum.arduino.cc/index.php?topic=208346.0
 * 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Version 4.0 : - remove clockdow from the CPU thanks @pphill for the tests
 *               - Optimisation for reception thanks @pphill for the tests
 *               - Bugfix reciving data will change fast commande
 *               - add support for arduino 101 board thanks @kenij
 * Version 3.0 : - Add support for CC1101 chip test are made with following chip 
 *               https://www.aliexpress.com/item/1PCS-CC1101-Wireless-Module-SMA-Antenna-Wireless-Transceiver-Module-For-Arduino/32287839674.html?spm=2114.13010608.0.0.JtqEv9
 *               thanks @guimcha for the tests
 *               - refound lib : split code in more files 
 *               - minor bugfix 
 * Version 2.1 : - bugfix invert data[4] and data[6] and frame[4] and frame[6] the data sending and reciving is little-endian thanks @Icare 
 *               - bugfix cast on address thanks @Icare
 *               - fix issue arduino MEGA thanks @Icare
 *               - configuration input output for MEGA thanks @Icare
 *               - add enable pull up resistor for RX input 
 * Version 2.0 : Add somfy rts recive functions
 * Version 1.1 : bugfix data[6] 
 * Version 1.0 : Initial version 
 *
 */

#ifdef __AVR__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <avr/power.h>
#endif
#include <stdio.h>
#include <string.h>
#include "somfy_rts.h"
#include "mydelay.h"
#include <SPI.h>

volatile long somfy_rts_pulse_l;
volatile long somfy_last_send;

uint8_t somfy_rts_repetition = 6;
uint16_t somfy_rts_interval_half = 620;

// Variable for reciving somfy control
const word k_tempo_synchro_hw_min   = 2000; 
const word k_tempo_synchro_hw_max   = 3200;
const word k_tempo_synchro_sw_min   = 3000;
const word k_tempo_synchro_sw_max   = 5500;
const word k_tempo_half_symbol_min  = 400;
const word k_tempo_half_symbol_max  = 900; 
const word k_tempo_symbol_min       = 1000;
const word k_tempo_symbol_max       = 3000;


volatile short somfy_rts::somfy_rts_status; 
volatile short somfy_rts::somfy_rts_cpt_bits;
volatile short somfy_rts::somfy_rts_cpt_synchro_hw;

volatile short somfy_rts::somfy_rts_previous_bit;
volatile bool somfy_rts::somfy_rts_waiting_half_symbol;
volatile short somfy_rts::somfy_rts_payload[SOMFY_RTS_FRAME_SIZE];
SomfyRXData  somfy_rts::SomfyData;
volatile SomfyRXData  somfy_rts::LastSendedSomfyData;
volatile bool somfy_rts::somfy_rts_CC1101_Used;
volatile bool somfy_rts::somfy_rts_Reception_Activated;

const word  k_waiting_synchro =1;
const word  k_receiving_data = 2;

#if defined(__arc__)
PinDescription *p = &g_APinDescription[CC1101_TX_PIN];

// From digitalWrite but removing interrupt_lock / unlock

// WRITE_ARC_REG for PIN 3
#define SET_TX_PIN_LOW \
do {\
    uint32_t reg = p->ulGPIOBase + SS_GPIO_SWPORTA_DR;\
    uint32_t bit = p->ulGPIOId;\
         SET_PIN_MODE(p->ulSocPin, GPIO_MUX_MODE);\
    WRITE_ARC_REG(READ_ARC_REG(reg) & ~(1 << bit), reg);\
    SET_PIN_PULLUP(p->ulSocPin,0);\
} while(0)

#define SET_TX_PIN_HIGH \
do {\
    uint32_t reg = p->ulGPIOBase + SS_GPIO_SWPORTA_DR;\
    uint32_t bit = p->ulGPIOId;\
         SET_PIN_MODE(p->ulSocPin, GPIO_MUX_MODE);\
    WRITE_ARC_REG(READ_ARC_REG(reg) | (1 << bit), reg);\
    SET_PIN_PULLUP(p->ulSocPin,1);\
} while(0)
    
/*
// MMIO_REG_VAL for PIN 4
#define SET_TX_PIN_LOW \
do {\
    uint32_t reg = p->ulGPIOBase + SOC_GPIO_SWPORTA_DR;\
    uint32_t bit = p->ulGPIOId;\
    MMIO_REG_VAL(reg) &= ~(1 << bit);\
    SET_PIN_PULLUP(p->ulSocPin,0);\
} while(0)

#define SET_TX_PIN_HIGH \
do {\
    uint32_t reg = p->ulGPIOBase + SOC_GPIO_SWPORTA_DR;\
    uint32_t bit = p->ulGPIOId;\
    MMIO_REG_VAL(reg) |= (1 << bit);\
    SET_PIN_PULLUP(p->ulSocPin,1);\
} while(0)
#endif
*/
#else // defined(__arc__)

#define SET_TX_PIN_LOW _TX_PORT &= ~_BV(_TX_PIN)
#define SET_TX_PIN_HIGH _TX_PORT |= _BV(_TX_PIN)

#endif // defined(__arc__)



// Interrupt handler
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__arc__)
void ext_int_1(void) {
#else
ISR(ANALOG_COMP_vect) {
#endif
    somfy_rts::Somfy_Recive_ISR();
}

// constructor
somfy_rts::somfy_rts()  
{     
    led_init();	
    // set TX pin as output    
#if defined(__arc__)
    pinMode(CC1101_TX_PIN, OUTPUT);
#else
    _TX_DDR  |= _BV(_TX_PIN);
#endif
    
    //set TX output to low
    SET_TX_PIN_LOW;

    //initialise recieving statemachine 
    somfy_rts_status =  k_waiting_synchro;
    
#if defined(__arc__)
    // 101 needs to wait Serial port with 
    //while(!Serial) ; // done as first line of your setup method
#else
    Serial.begin(115200);
#endif
 
}

// destructor
somfy_rts::~somfy_rts() {
	
}

//configure recive
void somfy_rts::beginRecive()
{
#if defined(__arc__)
    pinMode(CC1101_RX_PIN, INPUT);
    
    attachInterrupt(CC1101_RX_PIN, ext_int_1, CHANGE);       // interrupt number = pin number

#elif !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)

    // use analog comparator to switch at 1.1V bandgap transition
    ACSR = _BV(ACBG) | _BV(ACI) | _BV(ACIE);

    // set ADC mux to the proper port
    ADCSRA &= ~ bit(ADEN);
    ADCSRB |= bit(ACME);
    ADMUX = _RX_PIN;
   
    _RX_PORT &= ~_BV(_RX_PIN);                  //set pull up resisitor    

#else //#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
 
    _RX_DDR  &= ~_BV(_RX_PIN);                  //set pin as input
    _RX_PORT &= ~_BV(_RX_PIN);                  //set pull up resisitor    

    attachInterrupt(5, ext_int_1, CHANGE);       //pin 18
#endif //#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    
    //init recive state machine 
    somfy_rts_status =  k_waiting_synchro;
  
    //init recive data
    SomfyData.cmd = LastSendedSomfyData.cmd = SomfyCmd_None;                
    SomfyData.address = LastSendedSomfyData.address = 0;
	//Reception is activated
	somfy_rts_Reception_Activated = true;
 

}


// send bit = Zero
void somfy_rts::send_somfy_rts_bitZero(void) {
	// Somfy RTS bits are manchester encoded: 0 = high->low
	SET_TX_PIN_HIGH;
	delay_us(somfy_rts_interval_half);
	SET_TX_PIN_LOW;
	delay_us(somfy_rts_interval_half);
}

// send bit = One
void somfy_rts::send_somfy_rts_bitOne(void) {
	// Somfy RTS bits are manchester encoded: 1 = low->high
	SET_TX_PIN_LOW;
	delay_us(somfy_rts_interval_half);
	SET_TX_PIN_HIGH;
	delay_us(somfy_rts_interval_half);
}

// send command frame
void somfy_rts::send_somfy_rts_frame(somfy_rts_frame_t *frame, int8_t hwPulses) {
	// send hardware sync (pulses of about double length)
	for (int8_t i = 0; i < hwPulses; i++) {
		SET_TX_PIN_HIGH;

		delay_us(2550);
		SET_TX_PIN_LOW;

		delay_us(2550);
	}


	// send software sync (4 x symbol width high, half symbol width low)
	SET_TX_PIN_HIGH;
	delay_us(4860);
	SET_TX_PIN_LOW;
	delay_us(somfy_rts_interval_half);

	// Send the user data
	for (int8_t i = 0; i < SOMFY_RTS_FRAME_SIZE; i++) {
		uint16_t mask = 0x80; // mask to send bits (MSB first)
		uint8_t d = frame[i];
		for (int8_t j = 0; j < 8; j++) {
			if ((d & mask) == 0) {
				send_somfy_rts_bitZero();
			} else {
				send_somfy_rts_bitOne();
			}
			mask >>= 1; //get next bit
		} //end of byte
	} //end of data

	// send inter-frame gap
	// if last bit = 0, silence is 1/2 symbol longer
	SET_TX_PIN_LOW;
	delay_us(30415 + ((frame[6] >> 7) & 1) ? 0 : somfy_rts_interval_half);

}

// calculate cheksum
uint8_t somfy_rts::somfy_rts_calc_checksum(somfy_rts_frame_t *frame) {
	uint8_t checksum = 0;
	for (int8_t i = 0; i < 7; i++) {
		checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
	}

	return checksum;
}

void somfy_rts::somfy_rts_send(SomfyCmd cmd, uint16_t rolling_code, uint32_t address) {
    somfy_rts_send(cmd, 0xA0, rolling_code, address);
}

//send somfy RTS
void somfy_rts::somfy_rts_send(SomfyCmd cmd, uint8_t key, uint16_t rolling_code, uint32_t address) {
	int8_t i;

	// create somfy frame from the given input
	// 0   |    1     |   2     3    | 4   5   6
	// key | ctrl+cks | rolling_code | address


	uint8_t airdata[SOMFY_RTS_FRAME_SIZE];

	airdata[0] = key +  (rolling_code & 0x000F);
	airdata[1] = cmd << 4;
	airdata[2] = (rolling_code & 0xFF00) >> 8;
	airdata[3] = rolling_code & 0x00FF;
	airdata[4] = (address & 0x0000FF);       // Address
	airdata[5] = (address & 0x00FF00) >> 8;  // Address
	airdata[6] = (address & 0xFF0000) >> 16; // Address


	// calculate checksum
	airdata[1] |= (somfy_rts_calc_checksum(airdata) & 0x0F);
	
	// save unencrypted data to return later
	uint8_t unencrypted[SOMFY_RTS_FRAME_SIZE];
	memcpy(unencrypted, airdata, SOMFY_RTS_FRAME_SIZE);

	// "encrypt"
	for (i = 1; i < SOMFY_RTS_FRAME_SIZE; i++) {
		airdata[i] = airdata[i] ^ airdata[i-1];
	}
    
    
    // if CC1101 is used you should put it in TX mode befor sending 
    if (somfy_rts_CC1101_Used)
    {
	    ccStrobe(CC1101_SIDLE);
	    ccStrobe(CC1101_SFRX);
	    ccStrobe(CC1101_SFTX);    
	    // enable TX
	    ccTX();
    }


	// send wakeup pulse

	SET_TX_PIN_HIGH;
	delay_us(9415);
	SET_TX_PIN_LOW;
	delay_ms(89);// should be 89565 us
	delay_us(565);

	// send the whole frame several times
	for(i = 0; i < somfy_rts_repetition; i++){
		send_somfy_rts_frame(airdata, (i == 0) ? 2 : 7); // send 2 hw Sync pulses at first, and 7 for repeated frames
    }
    
    // if CC1101 is used put it in RX mode after sending 
    if (somfy_rts_CC1101_Used)
    {    
        ccRX();
    }
}

void somfy_rts::somfy_rts_func(SomfyCmd cmd, uint16_t rolling_code,uint32_t address) {
    somfy_rts_func(cmd, 0xA0, rolling_code, address);
}
//function somfy RTS
void somfy_rts::somfy_rts_func(SomfyCmd cmd, uint8_t key, uint16_t rolling_code,uint32_t address) {
    
    
    // Switch On led during sending
    LED_ON();
    //Stop interuppt during sending data
    noInterrupts();
    //Send data
    somfy_rts_send(cmd, key, rolling_code, address); // Send real data
    //enable again interuppt
    interrupts();
    // switch off led
    LED_OFF();
 
}


void somfy_rts::Somfy_Recive_ISR()
{
    // check if reception is active and not just interrupt active from an other library
	if (!somfy_rts_Reception_Activated)
		return;
	
	static long last;
    static long lastsend;

    // determine the pulse length in microseconds, for either polarity
    somfy_rts_pulse_l = micros()  - last;
    last += somfy_rts_pulse_l;
    long p = abs(somfy_rts_pulse_l);

    
    if ((p>= k_tempo_half_symbol_min)  && (p <= k_tempo_synchro_sw_max)) 
    {
	#ifdef DEBUG_PULSE
            Serial.println(p);
    #endif //DEBUG_PULSE  
        switch(somfy_rts_status) {
            //Waiting  Hardware and software synchro 
            case k_waiting_synchro:
                if (p > k_tempo_synchro_hw_min && p < k_tempo_synchro_hw_max) {
                    ++somfy_rts_cpt_synchro_hw;

                }

                else if (p > k_tempo_synchro_sw_min && p < k_tempo_synchro_sw_max && somfy_rts_cpt_synchro_hw >= 4) {
                    somfy_rts_cpt_bits = 0;
                    somfy_rts_previous_bit = 0;
                    somfy_rts_waiting_half_symbol = false;

                    for(int i=0; i<SOMFY_RTS_FRAME_SIZE; ++i) 
                       somfy_rts_payload[i] = 0;
                    
                    somfy_rts_status = k_receiving_data;
                } 
                else {
                    somfy_rts_cpt_synchro_hw = 0;
                }
                break;
            //Reciving data
            case k_receiving_data:
                if (p > k_tempo_symbol_min && p < k_tempo_symbol_max && !somfy_rts_waiting_half_symbol) {
                    somfy_rts_previous_bit = 1 - somfy_rts_previous_bit;
                    somfy_rts_payload[somfy_rts_cpt_bits/8] += somfy_rts_previous_bit << (SOMFY_RTS_FRAME_SIZE - somfy_rts_cpt_bits%8);
                    ++somfy_rts_cpt_bits;
                } 
                else if (p > k_tempo_half_symbol_min && p < k_tempo_half_symbol_max) {
                    if (somfy_rts_waiting_half_symbol) {
                        somfy_rts_waiting_half_symbol = false;
                        somfy_rts_payload[somfy_rts_cpt_bits/8] += somfy_rts_previous_bit << (SOMFY_RTS_FRAME_SIZE - somfy_rts_cpt_bits%8);
                        ++somfy_rts_cpt_bits;
                    } 
                    else {
                        somfy_rts_waiting_half_symbol = true;
                    }
                } 
                else {
                    somfy_rts_cpt_synchro_hw = 0;
                    somfy_rts_status = k_waiting_synchro;
                }
                  
                break;
      
            default:
                Serial.print(F("Internal error ! "));
				Serial.println(somfy_rts_status );
                break;
        }
  
 
  
        if (somfy_rts_status == k_receiving_data && somfy_rts_cpt_bits == 56) 
        {
            byte frame[SOMFY_RTS_FRAME_SIZE];
            frame[0] = somfy_rts_payload[0];
            for(int i = 1; i < SOMFY_RTS_FRAME_SIZE; ++i) frame[i] = somfy_rts_payload[i] ^ somfy_rts_payload[i-1];
        #ifdef DEBUG_SOMFY_RECIVE            
            // Print frame
            for(int i = 0; i < SOMFY_RTS_FRAME_SIZE; ++i) Serial.print(frame[i], HEX); 
            
            Serial.print(" ");
            Serial.println("");
        #endif //DEBUG_SOMFY_RECIVE  
            // checksum
            byte cksum = 0;
            for(int i = 0; i < SOMFY_RTS_FRAME_SIZE; ++i) cksum = cksum ^ frame[i] ^ (frame[i] >> 4);
            cksum = cksum & 0x0F;
            if (cksum != 0) Serial.println(F("Checksum error !"));
            
            //save last send time       
            somfy_last_send = micros()-lastsend;
            lastsend += somfy_last_send;
            long LastSendElapsedTime = abs(somfy_last_send);

            //Command
            SomfyCmd Command = (SomfyCmd)((frame[1] & 0xF0)>>4) ;

            // Rolling code
            uint16_t rolling_code = (frame[2] << 8) + frame[3];
        #ifdef DEBUG_SOMFY_RECIVE           
            Serial.print(F("Rolling code: ")); Serial.println(rolling_code);
        #endif  //#ifdef DEBUG_SOMFY_RECIVE  
            // Address
            unsigned long address = ((unsigned long)frame[6] << 16)   + ((unsigned long)frame[5] << 8) + frame[4];
            
       #ifdef DEBUG_SOMFY_RECIVE 
            Serial.print (F("Command "));
            Serial.println (Command);
            Serial.print (F("LastSendedSomfyData.cmd "));
            Serial.println ( LastSendedSomfyData.cmd);
            Serial.print (F("LastSendElapsedTime "));
            Serial.println (LastSendElapsedTime);            
        #endif //#ifdef DEBUG_SOMFY_RECIVE  
          // resend to sketch value only if different or sended for more that 1s (0.5e6 *2 due to half clock speed )
           if (    (Command != LastSendedSomfyData.cmd)
                ||  (address != LastSendedSomfyData.address)
                ||  (LastSendElapsedTime >=0.5e6 *2)
			   )
            {
                SomfyData.cmd = LastSendedSomfyData.cmd = Command;                
                SomfyData.address = LastSendedSomfyData.address = address;
        #ifdef DEBUG_SOMFY_RECIVE 
                Serial.print(F("Address: ")); Serial.println(address, HEX); 
        #endif  //#ifdef DEBUG_SOMFY_RECIVE  

        #ifdef DEBUG_SOMFY_RECIVE             
                Serial.print(F("Command sended ")); 
                Serial.println( Command, HEX);

                switch(Command) {
                    case SomfyCmd_My: Serial.println(F("My")); break;
                    case SomfyCmd_Up: Serial.println(F("Up")); break;
                    case SomfyCmd_MyUp: Serial.println(F("My + Up")); break;
                    case SomfyCmd_Down: Serial.println(F("Down")); break;
                    case SomfyCmd_MyDown: Serial.println(F("My + Down")); break;
                    case SomfyCmd_UpDown: Serial.println(F("Up + Down")); break;
                    case SomfyCmd_Prog: Serial.println(F("Prog")); break;
                    case SomfyCmd_SunFlag: Serial.println(F("Sun + Flag")); break;
                    case SomfyCmd_Flag: Serial.println(F("Flag")); break;
                default: 
                Serial.print(F("??? ")); 
                Serial.println( Command, HEX);
                break;
                }
        #endif //#ifdef DEBUG_SOMFY_RECIVE     
             
            }


            somfy_rts_status = k_waiting_synchro;
        } //if (somfy_rts_status == k_receiving_data && somfy_rts_cpt_bits == 56) 

    }//if ((p>= k_tempo_half_symbol_min)  && (p <= k_tempo_synchro_sw_max)) 

}

//start the CC1101 chip with defined PA table config
void somfy_rts::beginCC1101(CC1101_TxPower_e CC1101_TxPower)
{
    initSeqCC1101(CC1101_TxPower);

}

//start the CC1101 chip with default PA table config : 0dBm
void somfy_rts::beginCC1101()
{
    initSeqCC1101(CC1101_TxPower_0dBm);

}
//initialize the CC1101 chip
void somfy_rts::initSeqCC1101(CC1101_TxPower_e CC1101_TxPower)
{
    //set bit to remenber that CC1101 is used some feature are diferent for sending and reciving data     
    somfy_rts_CC1101_Used = true;

	//SET_BIT(CC1101_CS_DDR, CC1101_CS_PIN); // CS as output
    pinMode(CC1101_CS_PIN, OUTPUT);
    digitalWrite(CC1101_CS_PIN, HIGH);

    // init SPI    
    SPI.begin();
    // setup the CC1101 with configurated power   
    somfy_rts_CC1101_tunein(CC1101_TxPower);
    delay_ms(3);

  #ifdef DEBUG_CC1101
    ReadAllRegisterCC1101();
  #endif //#ifdef DEBUG_CC1101
    ccRX();
}


// configure CC1101 with the setting defined in the CC1101.h file
void somfy_rts::somfy_rts_CC1101_tunein(CC1101_TxPower_e CC1101_TxPower) 
{

	CC1101_DEASSERT;// Toggle chip select signal
	delay_us(30);
	CC1101_ASSERT;
	delay_us(30);
	CC1101_DEASSERT;
	delay_us(45);
    // reset the chip 
	ccStrobe( CC1101_SRES);// Send SRES command
	delay_us(100);

	// load configuration
	CC1101_ASSERT;
	// write all register
    CC1101_sendbyte( CC1101_IOCFG2 | CC1101_WRITE_BURST);
	for(uint8_t i = 0; i < CC1101_CFG_SIZE; i++) 
	{
        CC1101_sendbyte(CC1101_SOMFY_RTS_CFG[i]);
	}
	CC1101_DEASSERT;

	// setup PA table
	CC1101_ASSERT;
    CC1101_sendbyte( CC1101_PATABLE | CC1101_WRITE_BURST);
	CC1101_sendbyte(0x00); 
	CC1101_sendbyte(paTable[CC1101_TxPower]); // for ASK/OOK  should be writen in the 2nd byte from the pa table also if ramping is not used
	CC1101_DEASSERT;

	// Set CC_ON
	ccStrobe( CC1101_SCAL);
	delay_ms(1);

}

// read some setings and print it in the debug windows
void somfy_rts::ReadAllRegisterCC1101()
{
  char res[5];
  //read Partnum
  CC1101_ASSERT;
  CC1101_sendbyte(CC1101_PARTNUM | CC1101_READ_BURST);
  Serial.print(F("Partnum " ));
  Serial.print(F(" : ")); 	
  sprintf(&res[0], "%02X", CC1101_sendbyte(0));
  Serial.println(res);  
  
  //read version
  CC1101_ASSERT;
  CC1101_sendbyte(CC1101_VERSION | CC1101_READ_BURST);
  Serial.print(F("Version " ));
  Serial.print(F(" : "));
  sprintf(&res[0], "%02X", CC1101_sendbyte(0));
  Serial.println(res);  
  
  //read config register
  for (uint16_t i = CC1101_IOCFG2; i<CC1101_TEST0;i++)
  {
    Serial.print(F("Registre Address " ));
    sprintf(&res[0], "%02X", i);
    Serial.print(res);   
    Serial.print(F(" : "));
    sprintf(&res[0], "%02X", CC1101_readReg(i));
    Serial.println(res);
  }

  // read PATABLE Value
  CC1101_ASSERT;
  CC1101_sendbyte(CC1101_PATABLE | CC1101_READ_BURST);

  Serial.print(F("PA Table value " ));
  Serial.print(F(" : "));
  for (uint8_t i = 0; i < 8; i++) {
	sprintf(&res[0], "%02X", CC1101_sendbyte(0));
    Serial.print(res); 
    Serial.print(" "); 

  }
  Serial.println("");  

}

