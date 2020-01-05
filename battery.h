/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - Battery measurement class ---------------------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
*   infos on the battery values: http://wiki.mikrokopter.de/LiPo
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _BATTERY_h
#define _BATTERY_h

#include "myfunc.h"

#define battery_high 1260
#define battery_low	  900

static uint16_t battery_adj = 1000;

class BatteryBaseClass : public TimerClass{
	uint8_t  pin;
	uint8_t value;
	uint8_t alarm_value;
	uint16_t interval;

public:
	uint16_t* factor;																										// pointer to the factor variable to correct battery measurement

	BatteryBaseClass(uint8_t pin, uint16_t *factor = (uint16_t*)&battery_adj, uint8_t alarm = 30) {
		this->pin = pin;
		this->factor = factor;
		this->alarm_value = alarm;
		this->value = 100;
	}
	virtual ~BatteryBaseClass() {}


	void init() {
		pinMode(pin, INPUT);																							// measurment on an input pin only
		analogReference(INTERNAL);																				// battery reference to 1.1 Volt
		analogRead(pin);																									// read the pin, think it initialise the HW but not sure
		short_interval(false);
		wait_set(500);																										// next measurement after all othe HW is initialised
	}

	void poll() {
		if (wait_done()) {
			uint32_t milli_volt = analogRead(pin);													// read the io pin 3 times to get a stable value
			milli_volt += analogRead(pin);
			milli_volt += analogRead(pin);
			milli_volt /= 3;																								// divide by the amount of measures

			milli_volt *= 1385;																							// some math to get the milli volt value
			milli_volt /= *this->factor;																		// factor can be used to compensate the resistor deviation

			uint8_t temp = value;																						// calculate a percentage value
			value = map(milli_volt, battery_low, battery_high, 0, 100);
			if (value > 100) value = 100;
			
			if (value <= alarm_value) alarm();															// check if we need to call alarm

			wait_set(interval);																							// set next time to measure

			if ((value != temp)	&& (milli_volt > 500))														// some debug but only on change and if above  certain level
				dbg << F("milli_volt: ") << milli_volt << F(", ") << value << '%' << '\n';
		}
	}

	void short_interval(uint8_t enabled) {
		interval = (enabled == true) ? 500: 10000;
	}

	uint8_t level() {
		return this->value;
	}

	virtual void alarm() {
		//dbg << F("battery alarm\n");
	}
};

#endif

