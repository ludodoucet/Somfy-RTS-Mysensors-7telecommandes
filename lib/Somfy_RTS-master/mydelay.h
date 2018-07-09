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
#ifndef _MYDELAY_H_
#define _MYDELAY_H_
#ifdef __AVR__
#include <util/delay_basic.h>
#endif
/* public prototypes */

#if defined (__arc__) 
    void delay_us(uint32_t count);
    void delay_ms(uint32_t count);
#else    
    void delay_us(uint16_t count);
    void delay_ms(uint16_t count);
#endif

#endif /* _MYDELAY_H_ */
