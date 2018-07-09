/*******************************************************************************
    Filename: CC1101.h

    Copyright 2007 Texas Instruments, Inc.
*******************************************************************************/
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
#ifndef CC1101_H
#define CC1101_H

#ifdef __AVR__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#endif
;
void ccTX(void);
void ccRX(void);
uint8_t ccStrobe(uint8_t);

uint8_t CC1101_sendbyte(uint8_t data);
void CC1101_writeReg(uint8_t addr, uint8_t data);
uint8_t CC1101_readReg(uint8_t addr);


#define CC1101_CFG_SIZE 0x29 //41


 const uint8_t CC1101_SOMFY_RTS_CFG[CC1101_CFG_SIZE] = {
// Somfy   IDX NAME      COMMENT
	0x0D,// 00 IOCFG2    GDO2 as serial output
	0x2E,// 01 IOCFG1    Tri-State
	0x2D,// 02 IOCFG0    GDO0 for input
	0x07,// 03 FIFOTHR  
	0xD3,// 04 SYNC1    
	0x91,// 05 SYNC0    
	0x3D,// 06 PKTLEN   
	0x04,// 07 PKTCTRL1 
	0x32,// 08 PKTCTRL0 
	0x00,// 09 ADDR     
	0x00,// 0A CHANNR   
	0x06,// 0B FSCTRL1   152kHz IF Frquency
	0x00,// 0C FSCTRL0  
	0x10,// 0D FREQ2     433.42 (10,AB,85: Somfy RTS Frequency)
	0xAB,// 0E FREQ1    
	0x85,// 0F FREQ0    
	0x55,// 10 MDMCFG4   bWidth 325kHz
	0x0A,// 11 MDMCFG3   Drate: 828 ((256+11)*2^5)*26000000/2^28
	0x30,// 12 MDMCFG2   Modulation: ASK
	0x23,// 13 MDMCFG1  
	0xb9,// 14 MDMCFG0   ChannelSpace: 350kHz
	0x00,// 15 DEVIATN  
	0x07,// 16 MCSM2    
	0x00,// 17 MCSM1    
	0x18,// 18 MCSM0     Calibration: RX/TX->IDLE
	0x14,// 19 FOCCFG   
	0x6C,// 1A BSCFG    
	0x07,// 1B AGCCTRL2  42 dB instead of 33dB
	0x00,// 1C AGCCTRL1 
	0x90,// 1D AGCCTRL0  4dB decision boundery
	0x87,// 1E WOREVT1  
	0x6B,// 1F WOREVT0  
	0xF8,// 20 WORCTRL  
	0x56,// 21 FREND1   
	0x11,// 22 FREND0    0x11 without Pa ramping
	0xE9,// 23 FSCAL3   
	0x2A,// 24 FSCAL2   
	0x00,// 25 FSCAL1   
	0x1F,// 26 FSCAL0   
	0x41,// 27 RCCTRL1  
	0x00,// 28 RCCTRL0  
};

// enum for TX power selection
enum CC1101_TxPower_e
{
	CC1101_TxPower_Minus30dBm   = 0,    // -30 dBm
    CC1101_TxPower_Minus20dBm	= 1,	// -20 dBm
	CC1101_TxPower_Minus15dBm	= 2,	// -15 dBm
	CC1101_TxPower_Minus10dBm	= 3,	// -10 dBm
	CC1101_TxPower_0dBm	        = 4,	// 0 dBm
	CC1101_TxPower_Plus5dBm 	= 5,	// 5 dBm
	CC1101_TxPower_Plus7dBm 	= 6,	// 7 dBm
	CC1101_TxPower_Plus10dBm 	= 7,	// 10 dBm 
};

typedef enum CC1101_TxPower_e CC1101_TxPower_e;

//TX power value

const uint8_t paTable[8] = {
  0x12, // -30 dBm
  0x0E, // -20 dBm
  0x1D, // -15 dBm
  0x34, // -10 dBm 
  0x60, // 0 dBm
  0x84, // 5 dBm
  0xC8, // 7 dBm
  0xC0, // 10 dBm 

};

// Configuration Registers
#define CC1101_IOCFG2           0x00    // GDO2 output pin configuration
#define CC1101_IOCFG1           0x01    // GDO1 output pin configuration
#define CC1101_IOCFG0           0x02    // GDO0 output pin configuration
#define CC1101_FIFOTHR          0x03    // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1            0x04    // Sync word, high byte
#define CC1101_SYNC0            0x05    // Sync word, low byte
#define CC1101_PKTLEN           0x06    // Packet length
#define CC1101_PKTCTRL1         0x07    // Packet automation control
#define CC1101_PKTCTRL0         0x08    // Packet automation control
#define CC1101_ADDR             0x09    // Device address
#define CC1101_CHANNR           0x0A    // Channel number
#define CC1101_FSCTRL1          0x0B    // Frequency synthesizer control
#define CC1101_FSCTRL0          0x0C    // Frequency synthesizer control
#define CC1101_FREQ2            0x0D    // Frequency control word, high byte
#define CC1101_FREQ1            0x0E    // Frequency control word, middle byte
#define CC1101_FREQ0            0x0F    // Frequency control word, low byte
#define CC1101_MDMCFG4          0x10    // Modem configuration
#define CC1101_MDMCFG3          0x11    // Modem configuration
#define CC1101_MDMCFG2          0x12    // Modem configuration
#define CC1101_MDMCFG1          0x13    // Modem configuration
#define CC1101_MDMCFG0          0x14    // Modem configuration
#define CC1101_DEVIATN          0x15    // Modem deviation setting
#define CC1101_MCSM2            0x16    // Main Radio Cntrl State Machine config
#define CC1101_MCSM1            0x17    // Main Radio Cntrl State Machine config
#define CC1101_MCSM0            0x18    // Main Radio Cntrl State Machine config
#define CC1101_FOCCFG           0x19    // Frequency Offset Compensation config
#define CC1101_BSCFG            0x1A    // Bit Synchronization configuration
#define CC1101_AGCCTRL2         0x1B    // AGC control
#define CC1101_AGCCTRL1         0x1C    // AGC control
#define CC1101_AGCCTRL0         0x1D    // AGC control
#define CC1101_WOREVT1          0x1E    // High byte Event 0 timeout
#define CC1101_WOREVT0          0x1F    // Low byte Event 0 timeout
#define CC1101_WORCTRL          0x20    // Wake On Radio control
#define CC1101_FREND1           0x21    // Front end RX configuration
#define CC1101_FREND0           0x22    // Front end TX configuration
#define CC1101_FSCAL3           0x23    // Frequency synthesizer calibration
#define CC1101_FSCAL2           0x24    // Frequency synthesizer calibration
#define CC1101_FSCAL1           0x25    // Frequency synthesizer calibration
#define CC1101_FSCAL0           0x26    // Frequency synthesizer calibration
#define CC1101_RCCTRL1          0x27    // RC oscillator configuration
#define CC1101_RCCTRL0          0x28    // RC oscillator configuration
#define CC1101_FSTEST           0x29    // Frequency synthesizer cal control
#define CC1101_PTEST            0x2A    // Production test
#define CC1101_AGCTEST          0x2B    // AGC test
#define CC1101_TEST2            0x2C    // Various test settings
#define CC1101_TEST1            0x2D    // Various test settings
#define CC1101_TEST0            0x2E    // Various test settings

// Status registers
#define CC1101_PARTNUM          0x30    // Part number
#define CC1101_VERSION          0x31    // Current version number
#define CC1101_FREQEST          0x32    // Frequency offset estimate
#define CC1101_LQI              0x33    // Demodulator estimate for link quality
#define CC1101_RSSI             0x34    // Received signal strength indication
#define CC1101_MARCSTATE        0x35    // Control state machine state
#define CC1101_WORTIME1         0x36    // High byte of WOR timer
#define CC1101_WORTIME0         0x37    // Low byte of WOR timer
#define CC1101_PKTSTATUS        0x38    // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC       0x39    // Current setting from PLL cal module
#define CC1101_TXBYTES          0x3A    // Underflow and # of bytes in TXFIFO
#define CC1101_RXBYTES          0x3B    // Overflow and # of bytes in RXFIFO

// Multi byte memory locations
#define CC1101_PATABLE          0x3E
#define CC1101_TXFIFO           0x3F
#define CC1101_RXFIFO           0x3F

// Definitions for burst/single access to registers
#define CC1101_WRITE_BURST      0x40
#define CC1101_READ_SINGLE      0x80
#define CC1101_READ_BURST       0xC0

// Strobe commands
#define CC1101_SRES             0x30        // Reset chip.
#define CC1101_SFSTXON          0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                            // If in RX/TX: Go to a wait state where only the synthesizer is
                                            // running (for quick RX / TX turnaround).
#define CC1101_SXOFF            0x32        // Turn off crystal oscillator.
#define CC1101_SCAL             0x33        // Calibrate frequency synthesizer and turn it off
                                            // (enables quick start).
#define CC1101_SRX              0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                            // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX              0x35        // In IDLE state: Enable TX. Perform calibration first if
                                            // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                            // Only go to TX if channel is clear.
#define CC1101_SIDLE            0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                            // Wake-On-Radio mode if applicable.
#define CC1101_SAFC             0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR             0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD             0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX             0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX             0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST          0x3C        // Reset real time clock.
#define CC1101_SNOP             0x3D        // No operation. May be used to pad strobe commands to two
                                            // bytes for simpler software.


//------------------------------------------------------------------------------
// Chip Status Byte
//------------------------------------------------------------------------------

// Bit fields in the chip status byte
#define CC1101_STATUS_CHIP_RDYn_BM             0x80
#define CC1101_STATUS_STATE_BM                 0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

// Chip states
#define CC1101_STATE_IDLE                      0x00
#define CC1101_STATE_RX                        0x10
#define CC1101_STATE_TX                        0x20
#define CC1101_STATE_FSTXON                    0x30
#define CC1101_STATE_CALIBRATE                 0x40
#define CC1101_STATE_SETTLING                  0x50
#define CC1101_STATE_RX_OVERFLOW               0x60
#define CC1101_STATE_TX_UNDERFLOW              0x70


//------------------------------------------------------------------------------
// Other register bit fields
//------------------------------------------------------------------------------
#define CC1101_LQI_CRC_OK_BM                   0x80
#define CC1101_LQI_EST_BM                      0x7F

#define MARCSTATE_SLEEP            0x00
#define MARCSTATE_IDLE             0x01
#define MARCSTATE_XOFF             0x02
#define MARCSTATE_VCOON_MC         0x03
#define MARCSTATE_REGON_MC         0x04
#define MARCSTATE_MANCAL           0x05
#define MARCSTATE_VCOON            0x06
#define MARCSTATE_REGON            0x07
#define MARCSTATE_STARTCAL         0x08
#define MARCSTATE_BWBOOST          0x09
#define MARCSTATE_FS_LOCK          0x0A
#define MARCSTATE_IFADCON          0x0B
#define MARCSTATE_ENDCAL           0x0C
#define MARCSTATE_RX               0x0D
#define MARCSTATE_RX_END           0x0E
#define MARCSTATE_RX_RST           0x0F
#define MARCSTATE_TXRX_SWITCH      0x10
#define MARCSTATE_RXFIFO_OVERFLOW  0x11
#define MARCSTATE_FSTXON           0x12
#define MARCSTATE_TX               0x13
#define MARCSTATE_TX_END           0x14
#define MARCSTATE_RXTX_SWITCH      0x15
#define MARCSTATE_TXFIFO_UNDERFLOW 0x16


//#define SET_BIT(PORT, BITNUM) ((PORT) |= (1<<(BITNUM)))
//#define CLEAR_BIT(PORT, BITNUM) ((PORT) &= ~(1<<(BITNUM)))

#define CC1101_DEASSERT  digitalWrite(CC1101_CS_PIN, HIGH);
#define CC1101_ASSERT    digitalWrite(CC1101_CS_PIN, LOW);

/******************************************************************************/
#endif
