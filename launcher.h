/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - Launcher class with ESC driver ----------------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _LAUNCHER_h
#define _LAUNCHER_h

#include <Servo.h>
#include "myfunc.h"


/**
* @brief contructor for the launcher class to accelerate the darts in a FDL-2
*
* @parameter (uint8_t) ESC pin, (uint16_t) min_speed, (uint16_t) max_speed
*/
class LauncherClass : public TimerClass {
	uint8_t mode;																												// 0 = stopped, 10 = stopping, 1 = standby (reduced speed), 11 = going to standby speed, 2 = fire speed, 12 / 22 = accelerating to fire speed
	uint8_t pin;
	Servo myServo;																											// create a servo object
	uint16_t min_speed, max_speed, set_speed;														// some speed vaiables

public:
	uint8_t ready;																											// signals readiness of launcher 
	uint8_t *fire_speed;																								// pointer to fire speed in % of max_speed
	uint16_t *speedup_time;																							// pointer which holds the time the motor needs to speedup
	uint8_t *standby_speed;																							// pointer to standby speed in % of max_speed
	uint16_t *standby_time;																							// pointer to standby time in ms

	LauncherClass(uint8_t ESC, uint16_t min_speed, uint16_t max_speed) : pin(ESC), min_speed(min_speed), max_speed(max_speed) {
		ready = 0;
		mode = 0;
	}

	void init() {																												// init the launcher, write start value into the ESC
		myServo.attach(pin, min_speed, max_speed);												// attaches the servo pin to the servo object
		myServo.writeMicroseconds(0);																			// and set it off
		//dbg << F("L::init, min_speed: ") << min_speed << F(", max_speed: ") << max_speed << ' ' << _TIME << '\n';
	}

	void start() {																											// init the start process of the launcher 

		set_speed = max_speed / 100 * *fire_speed;												// calculate the needed speed - max_speed is an absolute value and fire_speed a percentage value

		// state machine modes: 0 = stopped, 10 = stopping, 1 = standby (reduced speed),
		// 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed 
		uint16_t set_timer;																								// generate a variable to store the speedup time against different circumsdances
		if (mode == 0) set_timer = *speedup_time;													// coming from stopped, full time needed
		else if (mode == 2) set_timer = 0;																// we are already in fire mode, no additional time needed
		else set_timer = *speedup_time / 2;																// standby, decrease to standby or accelerating to fire, not the fulltime needed

		mode = 12;																												// set state machine to 'accelerating to fire speed'
		wait_set(set_timer);																							// set the timer accordingly
		myServo.writeMicroseconds(set_speed);															// and write the new speed into the esc
		//dbg << F("L::set start, speed: ") << *fire_speed << F(", set_speed: ") << set_speed << F(", speedup_time: ") << *speedup_time << F(", set_timer: ") << set_timer << ' ' << _TIME << '\n';
	}

	void stop() {																												// init the stop process via standby speed 
		// stop means, we are reducing the speed of the launcher to a standby level for a certain time
		// here we are setting a new status of the state machine

		if (set_speed) set_speed = max_speed / 100 * *standby_speed;			// calculate the standby speed 				
		// state machine modes: 0 = stopped, 10 = stopping, 1 = standby (reduced speed),
		// 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed
		uint16_t set_timer = *speedup_time / 2;														// we need some time to slow down

		ready = 0;																												// indicate the pusher that he cannot fire
		mode = 11;																												// we are going to standby speed
		wait_set(set_timer);																							// set the timer accordingly
		myServo.writeMicroseconds(set_speed);															// write the new speed into the esc
		//dbg << F("L::set stop, speed: ") << *standby_speed << F(", set_speed: ") << set_speed << F(", set_timer: ") << set_timer << ' ' << _TIME << '\n';
	}

	void poll() {																												// poll function to operate the pusher
		// state machine modes: 0 = stopped, 10 = stopping, 1 = standby (reduced speed),
		// 11 = going to standby speed, 2 = fire speed, 12 = accelerating to fire speed 

		if (!wait_done()) return;																					// waiting for a finished timer, leave


		if (mode == 12) {							// accelerating to fire														
			/* triggered in the start function, if we are here the start time has finished already */
			ready = 1;																											// signalize the pusher that he is allowed to fire
			mode = 2;																												//set status to 'fire speed'
			//dbg << F("L::accelerate done ") << _TIME << '\n';

		}	else if (mode == 11) {			// reducing speed to standby mode
			/* triggered in the stop function, if we are here the stop time has finsished */
			wait_set(*standby_time);																				// set the standby timer
			mode = 1;																												// set status 'standby' - we are on reduced speed
			//dbg << F("L::standby for ") << *standby_time << F("ms ") << _TIME << '\n';

		}	else if (mode == 1) {				// standby time is over
			/* triggered by the state machine itself, standby is over, we need to stop the motor */
			uint16_t set_timer = *speedup_time / 2;													// calculate the time for the stop process
			wait_set(set_timer);																						// set the timer accordingly
			myServo.writeMicroseconds(0);																		// set the esc to stop
			mode = 10;																											// set the status to stopping mode
			//dbg << F("L::stopping for ") << set_timer << F("ms ") << _TIME << '\n';

		} else if (mode == 10) {			// stopping mode
			/* triggered by the state machine itself, stopping time is over, we have a new status */
			mode = 0;																												// we are stopped
			//dbg << F("L::stopped!") << _TIME << '\n';
		}
	}
};

#endif

