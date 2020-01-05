// buzzer.h

#ifndef _BUZZER_h
#define _BUZZER_h

#include "myfunc.h"

class BuzzerClass : public TimerClass {
	uint8_t pin;
	int8_t repeat;
	uint16_t ontime, offtime;
	//TimerClass timer;

public:
	BuzzerClass(uint8_t pin) {
		this->pin = pin;
	}
	~BuzzerClass() {}

	void init() {
		pinMode(pin, OUTPUT);
	}

	bool on(uint16_t onticks, uint16_t offticks, int8_t repeat) {
		if (on() == true) {
			ontime = onticks;
			offtime = offticks;
			this->repeat = repeat;
			if (ontime > 0) {
				wait_set(onticks);
			}
			return true;
		}
		return false;
		}

	bool on(uint16_t ticks) {
		return on(ticks, 0, 1);
	}

	bool on() {
		wait_set(0);
		digitalWrite(pin, HIGH);
		return true;
	}

	bool off(bool force) {
		if (force == true) {
			repeat = 0;
			ontime = 0;
		}
		digitalWrite(pin, LOW);
		return true;
	}

	bool off() {
		return off(false);
	}

	bool active() {
		return digitalRead(pin) == HIGH;
	}

	void trigger() {
		if (active()) {
			off();
			if (repeat != -1) repeat--;
			if ((repeat != 0) && ontime > 0) {
				wait_set(offtime);
			}
		}	else if ((repeat != 0) && ontime > 0) {
			on();
			wait_set(ontime);
		}
	}

	void poll() {
		if (wait_done() == 1) trigger();
	}

};


#endif

