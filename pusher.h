/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - Pusher class to operate the tb6612 motor driver -----------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#ifndef _PUSHER_h
#define _PUSHER_h

#include "myfunc.h"

/**
* @brief contructor for the pusher class to drive the pusher motor of a FDL-2
*
* @parameter (uint8_t) IN1 pin, (uint8_t) IN2 pin, (uint8_t) PWM pin, (uint8_t) STBY pin, (uint8_t) FRNT_SENS pin, (uint8_t) BACK_SENS pin
*/
class PusherClass : public TimerClass {
	uint8_t* enabled;																										// pointer to an enable variable - 1 means enabled (pusher shall start only while the launcher is at full speed)
	uint8_t pin_in1, pin_in2, pin_stb, pin_pwm;													// tb 6612 motor driver pins
	uint8_t pin_frnt, pin_back;																					// remember the front and back sensor pins

	uint8_t operate;																										// state machine, 0 inactive, 1 starting, 2 running, 3 returning, 4 breaking, 5 stopping
	uint8_t position;																										// 0 unknown, 10 unknown but motor started, 1 front sensor, 2 after frontsensor, 12 after frontsensor but slow speed, 3 back sensor, 4 after back sensor 
	uint8_t position_old;																								// to detect changes of the position
	uint8_t round;																											// pushed darts counter

public:
	uint8_t* mode;																											// pointer to how many darts to be launched by one start

	PusherClass(uint8_t IN1, uint8_t IN2, uint8_t PWM, uint8_t STBY, uint8_t FRNT_SENS, uint8_t BACK_SENS, uint8_t& launcher_enable)
		: pin_in1(IN1), pin_in2(IN2), pin_stb(STBY), pin_pwm(PWM), pin_frnt(FRNT_SENS), pin_back(BACK_SENS), enabled(&launcher_enable) {

		pinMode(pin_in1, OUTPUT);																						// the tb6612 chip has 2 inputs for driving the motor
		digitalWrite(pin_in1, LOW);																					// we need to set them as output and low at the arduino side

		pinMode(pin_in2, OUTPUT);
		digitalWrite(pin_in2, LOW);

		pinMode(pin_pwm, OUTPUT);																						// pwm drives the speed of the pusher motor
		set_speed(0);																												// we use the standard arduino function for it

		pinMode(pin_stb, OUTPUT);																						// not sure if the standby is needed, but 
		digitalWrite(pin_stb, HIGH);																				// it needs to be a high level for the tb6612 chip

		register_PCINT(pin_frnt);																						// the pusher has two sensors to detect the position
		register_PCINT(pin_back);																						// back and front. it is needed for driving the motor

		if (digitalRead(pin_back) == 0) position = 3;												// if the back sens is raised, we have a stable position

		operate = 3;																												// we start in returning mode			
	}
	~PusherClass() {}

	void set_speed(uint8_t speed) {																			// set speed and remembers it
		analogWrite(pin_pwm, speed);																			// standard arduino function for PWM
	}

	void start() {																											// start the pusher 
		operate = 1;																											// we need to set the operateing mode
		round = 0;																												// reset the round counter while we are started
		//dbg << F("P::set start ") << _TIME << '\n';	
	}

	void stop() {																												// init the stop process
		if (operate >= 3) return;																					// we are already in stopping mode
		if ((*mode) && (round < *mode)) return;														// don't now, while count is in use and not complete
		operate = 3;																											// enter go back mode
		//dbg << F("P::stop needed ") << _TIME << '\n';
	}

	void poll() {																												// poll function to operate the pusher
		// 0 inactive, 1 starting, 2 running, 3 returning, 4 breaking, 5 stopping

		if (operate == 1) {						// indicates that the pusher needs to be started

			if (*enabled != 1) return;																			// launcher is not up to speed, leave function

			set_speed(255);																									// set full speed
			digitalWrite(pin_in1, HIGH);																		// start the motor
			digitalWrite(pin_in2, LOW);
			operate = 2;																										// and indicate that we are in operate mode
			//dbg << F("P::started ") << _TIME << '\n';

		}	else if (operate == 2) {		// pusher runs, check count mode

			if ((*mode) && (round >= *mode)) {															// check if all bullets are fired
				stop();																												// slow down and start the stop operation
				//dbg << F("P::reached count: ") << round << ' ' << _TIME << '\n';
			}

		}	else if (operate == 3) {		// pusher is in return mode					
	 // several actions are started within the pcint function

			if (position == 0) {				// unknown position is indicated while just booted, so we start the motor to get a postion via the position sensors

				set_speed(160);																								// set a slow speed
				digitalWrite(pin_in1, HIGH);																	// start the motor to get a default position
				digitalWrite(pin_in2, LOW);
				digitalWrite(pin_stb, HIGH);																	// as we start the sketch with position = 0 and operate = 3 (return mode)
				position = 10;																								// set position to unknown but with motor started
				//dbg << F("P::no position, get one ") << _TIME << '\n';

			}	else if (position == 2) {	// we left the front sensor, the short break is done within the pcint function 

				if (!wait_done()) return;																			// pcint has a timer set to slowdown the motor
				set_speed(80);																								// set a slow speed
				digitalWrite(pin_in1, HIGH);																	// start the motor again
				digitalWrite(pin_in2, LOW);
				position = 12;																								// set a new position status
				//dbg << F("P::break done, motor on slow speed ") << _TIME << '\n';
			}

		}	else if (operate == 4) {		// pusher is in breaking mode
	 // wait for finishing the stopping time and check if we are at the right position 
	 // if we are too far, we see pos 4 in the pcint function and can return the motor there

			if (!wait_done()) return;																				// wait till the timer as finished

			digitalWrite(pin_in1, LOW);																			// release motor 
			digitalWrite(pin_in2, LOW);

			if (position == 3) {				// we are at the right position 
				operate = 5;																									// indicate finish
				//dbg << F("P::stopped at the right position ") << _TIME << '\n';
			}

		}	else if (operate == 5) {		// pusher is stopped
			operate = 0;																										// set operating mode to inactive
			round = 0;																											// and reset the round counter
			//dbg << F("P::finish stop, wait for action ") << _TIME << '\n';
		}

		if (position_old == position) return;															// leave while no change has happened								
		position_old = position;																					// to identify a change

		//if (position == 1) dbg << F("P::dart ") << round << ' ' << _TIME << '\n';	
	}

	void irq(uint8_t pin, uint8_t status, uint32_t time) {
		/* this is the pc interrupt function to detect the status of the front and back sensor
		** one turn of the pusher disc has several stati, here we are handling 1 front, 2 after front, 3 back and 4 after back sensor
		** on status 1 we count the pushed darts
		** on status 2 we start to slow down the motor if stop is required
		** on status 3 we stop the motor if stop is required
		** on status 4 we returning the motor if a stop is required because we have overrun the stop at the back sensor	*/

		if (pin == pin_back) {				// back sensor interrupt
			static uint32_t last_back_millis;																// variable to remember static on the time of the last event

			if (time - last_back_millis < 5) return;												// last event was <5 ms ago, seems to be a bounce
			last_back_millis = time;																				// remember the time for debounce

			//uint8_t stat = flag & digitalPinToBitMask(pin_back);					// filter if the button was pushed (0) or released (64)
			//dbg << "bb: " << _HEX(status) << ' ' << ((status) ? 'r' : 'p') << ' ' << _TIME << '\n';

			if (!status) {							// we are at the back sensor 
				position = 3;																									// set the position flag
				if (operate == 3) {																						// if we are in returning mode, we should have a slow motor and reached now the end position
					operate = 4;																								// set the new operate mode - breaking
					digitalWrite(pin_in1, HIGH);																// set breaking mode on motor
					digitalWrite(pin_in2, HIGH);
					wait_set(200);																							// and some time for slowing down, follow up is in the poll function
				}

			}	else {										// back sensor left
				position = 4;																									// remember the position
				if (operate == 4) {																						// seems we are slipped over the back sensor, so we return the motor
					operate = 3;																								// we are in returning mode
					set_speed(100);																							// set a slow speed
					digitalWrite(pin_in1, LOW);																	// start the motor in the oposite direction again 
					digitalWrite(pin_in2, HIGH);
				}
			}
		}


		if (pin == pin_frnt) {				// front senor interrupt
			static uint32_t last_frnt_millis;																// remember static on the time of the last event

			if (time - last_frnt_millis < 5) return;												// last event was <5 ms ago, seems to be a bounce
			last_frnt_millis = time;																				// remember the time for debounce

			//uint8_t stat = flag & digitalPinToBitMask(pin_frnt);					// filter if the button was pushed (0) or released (128)
			//dbg << "fb: " << _HEX(status) << ' ' << ((status) ? 'r' : 'p') << ' ' << _TIME << '\n';

			if (!status) {							// we are at the front sensor 
				position = 1;																									// remember the position
				round++;																											// increase the round counter (darts pushed)

			}	else {										// front sensor left
				position = 2;																									// set the new position
				if (operate == 3) {																						// if we are in returning mode, we are breaking the motor for some time and start it slower to return to the next position
					digitalWrite(pin_in1, HIGH);																// set breaking mode of tb6612 chip
					digitalWrite(pin_in2, HIGH);
					wait_set(100);																							// set some time, follow up is in main loop
				}
			}
		}
	}

};


#endif