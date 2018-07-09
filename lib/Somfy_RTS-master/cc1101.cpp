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
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */

#include <stdio.h>
#include <string.h>
#include "mydelay.h"
#include "cc1101.h"
#include "somfyconfig.h"

#if ARDUINO >= 100
	#include <Arduino.h> // Arduino 1.0
#else
	#include <WProgram.h> // Arduino 0022
#endif

#include <SPI.h>


// send byte on SPI 
uint8_t CC1101_sendbyte(uint8_t data)
{
#if defined (__arc__) 
  return SPI.transfer(data);
#else
  SPDR = data;		        // send byte
  while (!(SPSR & _BV (SPIF)));	// wait until transfer finished
  return SPDR;
#endif
}

//--------------------------------------------------------------------
// set CC1101 in Transfer mode
void ccTX(void)
{
  uint8_t cnt = 0xff;

  // Going from RX to TX does not work if there was a reception less than 0.5
  // sec ago. Due to CCA? Using IDLE helps to shorten this period(?)
  ccStrobe(CC1101_SIDLE);
  while(cnt-- &&
        (ccStrobe(CC1101_STX) & CC1101_STATUS_STATE_BM) != CC1101_STATE_TX)
    delay_us(10);
}

//--------------------------------------------------------------------
// set CC1101 in recive mode
void ccRX(void)
{
  uint8_t cnt = 0xff;

  while(cnt-- &&
        (ccStrobe(CC1101_SRX) & CC1101_STATUS_STATE_BM) != CC1101_STATE_RX)
    delay_us(10);


}

//--------------------------------------------------------------------
// read single register on CC1101
uint8_t CC1101_readReg(uint8_t addr)
{
  CC1101_ASSERT;
  CC1101_sendbyte( addr|CC1101_READ_SINGLE );
  uint8_t ret = CC1101_sendbyte( 0 );
  CC1101_DEASSERT;
  return ret;
}
//--------------------------------------------------------------------
// write single register on CC1101
void CC1101_writeReg(uint8_t addr, uint8_t data)
{
  CC1101_ASSERT;
  CC1101_sendbyte( addr );
  CC1101_sendbyte( data );
  CC1101_DEASSERT;
}


//--------------------------------------------------------------------
// send strobe commande

uint8_t ccStrobe(uint8_t strobe)
{
  CC1101_ASSERT;
  uint8_t ret = CC1101_sendbyte( strobe );
  CC1101_DEASSERT;
  return ret;
}




