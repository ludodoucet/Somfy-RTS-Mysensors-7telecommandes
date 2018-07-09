/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2017 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

//This may be used to change user task stack size:
//#define CONT_STACKSIZE 4096
#include "Schedule.h"
extern "C" {
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "cont.h"
}
#include <core_version.h>

#define LOOP_TASK_PRIORITY 1
#define LOOP_QUEUE_SIZE    1

#define OPTIMISTIC_YIELD_TIME_US 16000

struct rst_info resetInfo;

extern "C" {
	extern const uint32_t __attribute__((section(".ver_number"))) core_version =
	    ARDUINO_ESP8266_GIT_VER;
	const char* core_release =
#ifdef ARDUINO_ESP8266_RELEASE
	    ARDUINO_ESP8266_RELEASE;
#else
	    NULL;
#endif
} // extern "C"

int atexit(void(*func)())
{
	(void)func;
	return 0;
}

extern "C" void ets_update_cpu_frequency(int freqmhz);
void initVariant() __attribute__((weak));
void initVariant()
{
}

void preloop_update_frequency() __attribute__((weak));
void preloop_update_frequency()
{
#if defined(F_CPU) && (F_CPU == 160000000L)
	REG_SET_BIT(0x3ff00014, BIT(0));
	ets_update_cpu_frequency(160);
#endif
}

extern void(*__init_array_start)(void);
extern void(*__init_array_end)(void);

cont_t g_cont __attribute__((aligned(16)));
static os_event_t g_loop_queue[LOOP_QUEUE_SIZE];

static uint32_t g_micros_at_task_start;

extern "C" void esp_yield()
{
	if (cont_can_yield(&g_cont)) {
		cont_yield(&g_cont);
	}
}

extern "C" void esp_schedule()
{
	ets_post(LOOP_TASK_PRIORITY, 0, 0);
}

extern "C" void __yield()
{
	if (cont_can_yield(&g_cont)) {
		esp_schedule();
		esp_yield();
	} else {
		panic();
	}
}

extern "C" void yield(void) __attribute__((weak, alias("__yield")));

extern "C" void optimistic_yield(uint32_t interval_us)
{
	if (cont_can_yield(&g_cont) &&
	        (system_get_time() - g_micros_at_task_start) > interval_us) {
		yield();
	}
}

static void loop_wrapper()
{
	static bool setup_done = false;
	preloop_update_frequency();
	if (!setup_done) {
		_begin();		// Startup MySensors library
		setup_done = true;
	}
	_process();			// Process incoming data
	loop();
	run_scheduled_functions();
	esp_schedule();
}

static void loop_task(os_event_t *events)
{
	(void)events;
	g_micros_at_task_start = system_get_time();
	cont_run(&g_cont, &loop_wrapper);
	if (cont_check(&g_cont) != 0) {
		panic();
	}
}

static void do_global_ctors(void)
{
	void(**p)(void) = &__init_array_end;
	while (p != &__init_array_start) {
		(*--p)();
	}
}

extern "C" void __gdb_init() {}
extern "C" void gdb_init(void) __attribute__((weak, alias("__gdb_init")));

extern "C" void __gdb_do_break() {}
extern "C" void gdb_do_break(void) __attribute__((weak, alias("__gdb_do_break")));

void init_done()
{
	system_set_os_print(1);
	gdb_init();
	do_global_ctors();
	esp_schedule();
}


extern "C" void user_init(void)
{
	struct rst_info *rtc_info_ptr = system_get_rst_info();
	memcpy((void *)&resetInfo, (void *)rtc_info_ptr, sizeof(resetInfo));

	uart_div_modify(0, UART_CLK_FREQ / (115200));

	init();

	initVariant();

	cont_init(&g_cont);

	ets_task(loop_task,
	         LOOP_TASK_PRIORITY, g_loop_queue,
	         LOOP_QUEUE_SIZE);

	system_init_done_cb(&init_done);
}
