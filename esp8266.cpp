#ifdef ESP8266

// NOTE (BM):
// This file is a stripped down version of IRremoteESP8266.cpp, stripped of
// everything that is not specific for the ESP8266, and renamed to esp8266.cpp.
// It (now) contains esp8266-specific functions that fit to IRremote.

// Some of the comments below are not relevant any longer,
// but kept for legal reasons.

/***************************************************
 * IRremote for ESP8266
 *
 * Based on the IRremote library for Arduino by Ken Shirriff
 * Version 0.11 August, 2009
 * Copyright 2009 Ken Shirriff
 * For details, see
 *   http://arcfn.com/2009/08/multi-protocol-infrared-remote-library.html
 *
 * Modified by Paul Stoffregen <paul@pjrc.com> to support other boards and
 *   timers
 * Modified by Mitra Ardron <mitra@mitra.biz>
 * Added Sanyo and Mitsubishi controllers
 * Modified Sony to spot the repeat codes that some Sony's send
 *
 * Interrupt code based on NECIRrcv by Joe Knapp
 * http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1210243556
 * Also influenced by
 *   http://zovirl.com/2008/11/12/building-a-universal-remote-with-an-arduino/
 *
 * JVC and Panasonic protocol added by Kristian Lauszus (Thanks to zenwheel and
 *   other people at the original blog post)
 * LG added by Darryl Smith (based on the JVC protocol)
 * Whynter A/C ARC-110WD added by Francesco Meschia
 * Global Cache IR format sender added by Hisham Khalifa
 *   (http://www.hishamkhalifa.com)
 * Coolix A/C / heatpump added by bakrus
 * Denon: sendDenon, decodeDenon added by Massimiliano Pinto
 *   (from https://github.com/z3t0/Arduino-IRremote/blob/master/ir_Denon.cpp)
 * Kelvinator A/C and Sherwood added by crankyoldgit
 * Mitsubishi A/C added by crankyoldgit
 *     (derived from https://github.com/r45635/HVAC-IR-Control)
 *
 * Updated by markszabo (https://github.com/markszabo/IRremoteESP8266) for
 *   sending IR code on ESP8266
 * Updated by Sebastien Warin (http://sebastien.warin.fr) for receiving IR code
 *   on ESP8266
 *
 *  GPL license, all text above must be included in any redistribution
 ****************************************************/

#include "IRremote.h"
#include "IRremoteInt.h"

extern "C" {
	#include "user_interface.h"
	#include "gpio.h"
}

#ifdef USE_DEFAULT_ENABLE_IR_IN
#error Must undef USE_DEFAULT_ENABLE_IR_IN
#endif

// IRtimer ---------------------------------------------------------------------
// This class performs a simple time in useconds since instantiated.
// Handles when the system timer wraps around (once).

class IRtimer {
public:
	IRtimer();
	void reset();
	uint32_t elapsed();
private:
	uint32_t start;
};

IRtimer::IRtimer()
{
	reset();
}

void ICACHE_FLASH_ATTR IRtimer::reset()
{
	start = micros();
}

uint32_t ICACHE_FLASH_ATTR IRtimer::elapsed()
{
	uint32_t now = micros();
	if (start <= now) // Check if the system timer has wrapped.
		return(now - start); // No wrap.
	else
		return(0xFFFFFFFF - start + now); // Has wrapped.
}

static ETSTimer timer;

static void ICACHE_RAM_ATTR read_timeout(void *arg __attribute__((unused)))
{
	os_intr_lock();
	if (irparams.rawlen) {
		irparams.rcvstate = STATE_STOP;
	}
	os_intr_unlock();
}

static void ICACHE_RAM_ATTR gpio_intr()
{
	uint32_t now = system_get_time();
	uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	static uint32_t start = 0;

	os_timer_disarm(&timer);
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);

	if (irparams.rawlen >= RAWBUF) {
		irparams.overflow = true;
		irparams.rcvstate = STATE_STOP;
	}

	if (irparams.rcvstate == STATE_STOP) {
		return;
	}

	if (irparams.rcvstate == STATE_IDLE) {
		irparams.overflow = false;
		irparams.rcvstate = STATE_MARK;
		irparams.rawbuf[irparams.rawlen++] = 1;
	} else {
		if (now < start)
			irparams.rawbuf[irparams.rawlen++] = (0xFFFFFFFF - start + now) / USECPERTICK + 1;
		else
			irparams.rawbuf[irparams.rawlen++] = (now - start) / USECPERTICK + 1;
	}

	start = now;
#define ONCE 0
	os_timer_arm(&timer, 15, ONCE);
}

void ICACHE_FLASH_ATTR IRrecv::enableIRIn()
{
	// initialize state machine variables
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;

	// Initialize timer
	os_timer_disarm(&timer);
	os_timer_setfn(&timer, (os_timer_func_t *) read_timeout, NULL);

	// Attach Interrupt
	attachInterrupt(irparams.recvpin, gpio_intr, CHANGE);
}

// This could probably be useful, presently not enabled.
//void ICACHE_FLASH_ATTR IRrecv::disableIRIn()
//{
//	os_timer_disarm(&timer);
//	detachInterrupt(irparams.recvpin);
//}

#endif // ESP8266
