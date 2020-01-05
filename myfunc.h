/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - all supporting functions, like pin setup, pin change interrupt handling, etc ------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _MYFUNC_h
#define _MYFUNC_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/*-- serial print functions -----------------------------------------------------------------------------------------------
* template and some functions for debugging over serial interface
* based on arduino serial class, so should work with all hardware served in arduino
* http://aeroquad.googlecode.com/svn/branches/pyjamasam/WIFIReceiver/Streaming.h
*/

class NullSerial : public Print {
public:
	virtual size_t write(uint8_t) { return (1); }
	void begin(int16_t) {}
};

NullSerial static Noserial;

#ifdef DEBUG
#define dbg_m Serial
#else
#define dbg_m Noserial
#endif

#define dbg Serial

template<class T> Print& operator <<(Print& obj, T arg) { obj.print(arg); return obj; }

const char num2char[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',  'A', 'B', 'C', 'D', 'E', 'F', };
struct _HEX {
	uint8_t* val;
	uint8_t len;
	_HEX(uint8_t v) : val(&v), len(1) {}
	_HEX(uint8_t* v, uint8_t l = 1) : val(v), len(l) {}
};
inline Print& operator <<(Print& obj, const _HEX& arg) {
	for (uint8_t i = 0; i < arg.len; i++) {
		if (i) obj.print(' ');
		obj.print(num2char[arg.val[i] >> 4]);
		obj.print(num2char[arg.val[i] & 0xF]);
	}
	return obj;
}

enum _eTIME { _TIME };
inline Print& operator <<(Print& obj, _eTIME arg) { obj.print('('); obj.print(millis()); obj.print(')'); return obj; }
//- -----------------------------------------------------------------------------------------------------------------------



//-- interrupt functions --------------------------------------------------------------------------------------------------
#define DEBOUNCE  5																										// debounce time for periodic check if an interrupt was raised
#define pc_interrupt_vectors 3																				// amount of pin change interrupt vectors
extern void(*pci_ptr)(uint8_t pin, uint8_t status, uint32_t time);
void register_PCINT(uint8_t pin_def);
uint8_t check_PCINT(uint8_t pin_def, uint8_t debounce);
void maintain_PCINT(uint8_t vec);
//- -----------------------------------------------------------------------------------------------------------------------

//-- encoder functions ----------------------------------------------------------------------------------------------------
class EncoderClass {
	uint8_t pinA, pinB;
	uint8_t pinx_store[4];
	uint8_t stat_store[4];
	uint8_t count;
	int8_t  value;

public:
	EncoderClass(uint8_t pinA, uint8_t pinB) {
		this->pinA = pinA;
		this->pinB = pinB;
		count = 0;
		register_PCINT(pinA);
		register_PCINT(pinB);
	}
	virtual ~EncoderClass() {}

	void irq(uint8_t pin, uint8_t status, uint32_t time) {
		// right - pin:15,0,41676, pin:14,0, 41688, pin:15,1,41839, pin:14,1,41841
		// left - pin:14,0,114813, pin:15,0,114824, pin:14,1,114838, pin:15,1,114840
		if (((status) && (count < 2)) || ((!status) && (count >= 2))) {
			count = 0;
			//dbg << F("error:") << pin << ',' << status << ',' << time << '\n';
			return;
		}
		
		pinx_store[count] = pin;
		stat_store[count] = status;
		count++;

		if (count >= 4) {
			count = value = 0;
			if ((pinx_store[0] == pinB) && (pinx_store[1] == pinA)) value = 1;
			if ((pinx_store[0] == pinA) && (pinx_store[1] == pinB)) value = -1;
			//dbg << F("enc:") << value << '\n';
		}
	}

	int8_t getValue() {
		int8_t ret;
		ret = value;
		value = 0;
		return ret;
	}
};
//- -----------------------------------------------------------------------------------------------------------------------

//-- button functions -----------------------------------------------------------------------------------------------------
class ButtonClass {
	#define LONGPRESS 3000
	uint8_t pin;
	uint32_t last_time;
	uint8_t value;

public:
	ButtonClass(uint8_t pin) {
		this->pin = pin;
		register_PCINT(pin);
		value = 0;
	}
	virtual ~ButtonClass() {}

	void irq(uint8_t status, uint32_t time) {
		if (time - last_time < DEBOUNCE) {
			last_time = time;
			return;
		}

		if (status) {									// not pressed
			if (time - last_time > LONGPRESS) value = 4;										// long released
			else value = 2;																									// short released

		}	else {											// pressed
			value = 1;																											// button pressed
		}
		last_time = time;
	}

	uint8_t getValue() {
		if (!value) return 0;																							// nothing to do

		if (value == 5) {
			if (millis() - last_time > LONGPRESS) value = 3;								// long pressed
			else return 0;																									// we are waiting for it
		}

		uint8_t ret = value;
		if (value == 1) value = 5;																				// we send status pressed only one time
		else value = 0;																										// status send, set value to 0
		return ret;
	}
};

//- -----------------------------------------------------------------------------------------------------------------------

//-- waittimer functions --------------------------------------------------------------------------------------------------
class TimerClass {
	uint32_t startTime;
	uint32_t checkTime;
public:
	TimerClass() { checkTime = 0; }
	~TimerClass() {}

	uint8_t wait_done(void) {
		return this->done();
	}

	uint8_t done(void) {
		if (!checkTime) return 2;																					// not armed, so nothing to do
		if ((millis() - startTime) < checkTime) return 0;									// not ready yet
		checkTime = 0;																										// if we are here, timeout was happened
		return 1;																													// return a 1 for done
	}

	void wait_set(uint32_t wait_millis) {
		this->set(wait_millis);
	}
	
	void set(uint32_t wait_millis) {
		startTime = millis();
		checkTime = wait_millis;
	}

	uint32_t remain(void) {
		if (!checkTime) return 0;
		return (checkTime - (millis() - startTime));
	}

	uint8_t completed(void) {
		if (!checkTime) return 0;																					// not armed, so return not active
		else if ((millis() - startTime) >= checkTime) return 1;						// timer done, but not progressed
		else return 2;																										// time not ready, need some additional time
	}
};
//- -----------------------------------------------------------------------------------------------------------------------


//-- eeprom functions -----------------------------------------------------------------------------------------------------
class EEpromClass {
public:
	EEpromClass() {}
	~EEpromClass() {}

	void get(uint16_t addr, uint8_t len, void* ptr) {
		eeprom_read_block((void*)ptr, (const void*)addr, len);						// AVR GCC standard function
	}

 void set(uint16_t addr, uint8_t len, void* ptr) {
		eeprom_update_block((const void*)ptr, (void*)addr, len);					// AVR GCC standard function
	}

	void clear(uint16_t addr, uint16_t len) {
		uint8_t tB = 0;
		for (uint16_t l = 0; l < len; l++) {															// step through the bytes of eeprom
			set(addr + l, 1, (void*)&tB);
		}
	}
};



#endif
