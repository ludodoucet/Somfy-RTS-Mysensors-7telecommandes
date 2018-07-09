/*
 * this file is part of Somfy RTS communication protocol
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
 */


#ifndef _SomfyConfig_H
#define _SomfyConfig_H

//#define DEBUG_SOMFY_RECIVE // Enable this line for debug
//#define DEBUG_PULSE        // Enable this line to see pulse
//#define DEBUG_CC1101       // Enable this line for debug CC1101

#if ARDUINO >= 100
	#include <Arduino.h> // Arduino 1.0
#else
	#include <WProgram.h> // Arduino 0022
#endif


#define SOMFY_RTS_FRAME_SIZE 7

#if defined (__arc__) 

#define CC1101_CS_PIN 2     // Pin 2 for chip select
#define CC1101_TX_PIN 3     // Pin 3 for TX
#define CC1101_RX_PIN 7     // Pin 7 for RX

#define led_init()          pinMode(8, OUTPUT);
#define LED_ON()            digitalWrite(8, HIGH);
#define LED_OFF()           digitalWrite(8, LOW);

#else // defined (__arc__) 
//configuration output
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
    /*TX */
    #define _TX_DDR  	        DDRD    
    #define _TX_PORT       	    PORTD
    #define _TX_PIN             PD3     // Pin 3

    /*RX */
    #define _RX_PORT       	    PORTC
    #define _RX_PIN             PC1     // Pin A1

    /*SPI*/
    #define SPI_PORT		    PORTD
    #define SPI_DDR			    DDRD
    #define SPI_SS			    PD2     //Pin 2

#else
    /*TX */
    #define _TX_DDR  	        DDRE   
    #define _TX_PORT       	    PORTE
    #define _TX_PIN             PE5     // Pin 3
    
    /*RX */
    #define _RX_PORT       	    PORTD
    #define _RX_DDR  	        DDRD
    #define _RX_PIN             PD3     // Pin 18

    /*SPI*/
    #define SPI_PORT		    PORTB
    #define SPI_DDR			    DDRB
    #define SPI_SS			    PB0     //Pin 53

#endif

/* external LED */
#define _LED_DDR            DDRB
#define _LED_PORT           PORTB
#define _LED_PIN            PB0         //Pin 8 atmega328 / Pin 53 atmega1280 and atmega2560

#define led_init()          _LED_DDR  |= _BV(_LED_PIN)
#define LED_ON()            _LED_PORT |= _BV(_LED_PIN)
#define LED_OFF()           _LED_PORT &= ~_BV(_LED_PIN)

/*SPI*/
#define CC1101_CS_DDR		SPI_DDR
#define CC1101_CS_PORT      SPI_PORT
#define CC1101_CS_PIN		SPI_SS

#endif // defined (__arc__) 

#endif
