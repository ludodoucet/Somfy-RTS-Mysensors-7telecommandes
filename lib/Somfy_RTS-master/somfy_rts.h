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
 *               - bugfix cast on adress thanks @Icare
 *               - fix issue arduino MEGA thanks @Icare
 *               - configuration input output for MEGA thanks @Icare
 *               - add enable pull up resistor for RX input 
 * Version 2.0 : Add somfy rts recive functions
 * Version 1.1 : bugfix data[6] 
 * Version 1.0 : Initial version   
 *
 */


#ifndef _Somfy_Rts_H
#define _Somfy_Rts_H


#if ARDUINO >= 100
	#include <Arduino.h> // Arduino 1.0
#else
	#include <WProgram.h> // Arduino 0022
#endif
#include "somfyconfig.h"
#include "cc1101.h"

typedef uint8_t somfy_rts_frame_t;

enum SomfyCmd
{
	SomfyCmd_None       = 0,
    SomfyCmd_My 		= 0x1,	//My		 Stop or move to favourite position
	SomfyCmd_Up 		= 0x2,	//Up		 Move up
	SomfyCmd_MyUp 		= 0x3,	//My + Up	 Set upper motor limit in initial programming mode
	SomfyCmd_Down 		= 0x4,	//Down		 Move down
	SomfyCmd_MyDown 	= 0x5,	//My + Down	 Set lower motor limit in initial programming mode
	SomfyCmd_UpDown 	= 0x6,	//Up + Down	 Change motor limit and initial programming mode
	SomfyCmd_Prog   	= 0x8,	//Prog		 Used for (de-)registering remotes, see below
	SomfyCmd_SunFlag 	= 0x9,	//Sun + Flag Enable sun and wind detector (SUN and FLAG symbol on the Telis Soliris RC)
	SomfyCmd_Flag	 	= 0xA	//Flag		 Disable sun detector (FLAG symbol on the Telis Soliris RC)


};
typedef enum SomfyCmd SomfyCmd;
typedef struct SomfyRXData SomfyRXData;
struct SomfyRXData
{
    SomfyCmd cmd;
    unsigned long address;
};



/* prototypes */
class somfy_rts {


	public:
  
		somfy_rts();
		~somfy_rts();      	
        void somfy_rts_func(SomfyCmd cmd, uint16_t rolling_code, uint32_t address);
        void somfy_rts_func(SomfyCmd cmd, uint8_t key, uint16_t rolling_code, uint32_t address);
        void beginRecive();
        static void Somfy_Recive_ISR();    // only for internal use   
        static SomfyRXData SomfyData; 
	    void beginCC1101(CC1101_TxPower_e CC1101_TxPower);
        void beginCC1101();
        void ReadAllRegisterCC1101(void);   // only for debug
    private:

		void send_somfy_rts_bitZero(void) ;
		void send_somfy_rts_bitOne(void) ;
		void send_somfy_rts_frame(somfy_rts_frame_t *frame, int8_t hwPulses) ;
		uint8_t somfy_rts_calc_checksum(somfy_rts_frame_t *frame) ;
		void somfy_rts_send(SomfyCmd cmd, uint16_t rolling_code, uint32_t address);
		void somfy_rts_send(SomfyCmd cmd, uint8_t key, uint16_t rolling_code, uint32_t address);
		volatile static bool somfy_rts_Reception_Activated;
        volatile static SomfyRXData LastSendedSomfyData; 
		volatile static short somfy_rts_status;				
        volatile static short somfy_rts_cpt_bits;
        volatile static short somfy_rts_cpt_synchro_hw;
        volatile static short somfy_rts_previous_bit;
        volatile static bool somfy_rts_waiting_half_symbol;
        volatile static short somfy_rts_payload[SOMFY_RTS_FRAME_SIZE];
        void initSeqCC1101 (CC1101_TxPower_e CC1101_TxPower);        
        volatile static bool somfy_rts_CC1101_Used;
        void somfy_rts_CC1101_tunein(CC1101_TxPower_e CC1101_TxPower);
};

#endif
