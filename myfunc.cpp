/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - all supporting functions, like pin setup, pin change interrupt handling, etc ------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#include "myfunc.h"

/*-- interrupt functions --------------------------------------------------------------------------------------------------
* based on the same concept as the pin functions. everything pin related is defined in a pin struct, handover of the pin
* is done by forwarding a pointer to the struct. pin definition is done in HAL_<cpu>.h, functions are declared in HAL_<vendor>.h
*/
struct  s_pcint_vector {
	volatile uint8_t *PINREG;
	uint8_t curr;
	uint8_t chng;
	uint8_t prev;
	uint8_t mask;
	uint32_t time;
};
volatile s_pcint_vector pcint_vector[pc_interrupt_vectors];						// define a struct for pc int processing

/* function to register a pin interrupt */
void register_PCINT(uint8_t def_pin) {

	pinMode(def_pin, INPUT_PULLUP);																			// set the pin as input
	//set_pin_high(def_pin);																							// key is connected against ground, set it high to detect changes

	uint8_t vec = digitalPinToPCICRbit(def_pin);												// needed for interrupt handling and to sort out the port
	uint8_t port = digitalPinToPort(def_pin);														// need the pin port to get further information as port register
	if (port == NOT_A_PIN) return;																			// return while port was not found

	pcint_vector[vec].PINREG = portInputRegister(port);									// remember the input register
	pcint_vector[vec].mask |= digitalPinToBitMask(def_pin);							// set the pin bit in the bitmask

	*digitalPinToPCICR(def_pin) |= _BV(digitalPinToPCICRbit(def_pin));	// pci functions
	*digitalPinToPCMSK(def_pin) |= _BV(digitalPinToPCMSKbit(def_pin));	// make the pci active
	
	maintain_PCINT(vec);
	//dbg << "x-v:" << vec << ", m:" << pcint_vector[vec].mask << ", r:" << pcint_vector[vec].chng << ", pin:" << def_pin << '\n';
}

/* period check if a pin interrupt had happend */
uint8_t check_PCINT(uint8_t def_pin, uint8_t debounce) {
	// need to get vectore 0 - 3, depends on cpu
	uint8_t vec = digitalPinToPCICRbit(def_pin);												// needed for interrupt handling and to sort out the port
	uint8_t bit = digitalPinToBitMask(def_pin);													// get the specific bit for the asked pin

	uint8_t status = pcint_vector[vec].curr & bit ? 1 : 0;							// evaluate the pin status
	uint8_t prev = pcint_vector[vec].prev & bit ? 1 : 0;								// evaluate the previous pin status

	if (status == prev) return status;																	// check if something had changed since last time
	if (debounce && ((millis() - pcint_vector[vec].time) < DEBOUNCE)) return prev;	// seems there is a change, check if debounce is necassary and done

	pcint_vector[vec].prev ^= bit;																			// if we are here, there was a change and debounce check was passed, remember for next time

	if (status) return 3;																								// pin is 1, old was 0
	else return 2;																											// pin is 0, old was 1
}

void(*pci_ptr)(uint8_t pin, uint8_t status, uint32_t time);


/* internal function to handle pin change interrupts */
void maintain_PCINT(uint8_t vec) {

	pcint_vector[vec].curr = *pcint_vector[vec].PINREG  & pcint_vector[vec].mask;	// read the pin port and mask ot unneeded pins
	pcint_vector[vec].time = millis();																	// store the time, if we need to debounce it

	if (pcint_vector[vec].chng == pcint_vector[vec].curr) return;				// nothing to do while the same status as last time
	//dbg << "i-v:" << vec << ", m:" << pcint_vector[vec].mask << ", c:" << pcint_vector[vec].curr << ", n:" << pcint_vector[vec].chng << ", x:" << (pcint_vector[vec].curr ^ pcint_vector[vec].chng) << '\n';

	if (pci_ptr) {
		uint8_t pin_int = pcint_vector[vec].curr ^ pcint_vector[vec].chng;// evaluate the pin which raised the interrupt
		uint8_t status = pcint_vector[vec].curr & pin_int ? 1 : 0;				// evaluate the pin status

		// calc pin nummer from vector and pin bit
		// vec 0 = pin 8 to 13; vec 1 = pin 14 to 17; vec 2 = pin 0 to 7
		const uint8_t vect_pin[3] = { 8, 14, 0 };

		// pin_bit 1  = pin 0; pin_bit 2  = pin 1; pin_bit 4  = pin 2; pin_bit 8  = pin 3
		// pin_bit 16 = pin 4; pin_bit 32 = pin 5; pin_bit 64 = pin 6; and so on
		uint8_t ardu_pin = vect_pin[vec];																	// vector number sets the start position
		while (pin_int >>= 1) {																						// right shift till we have no bit anymore
			ardu_pin++;																											// increase the pin per turn
		}
		pci_ptr(ardu_pin, status, pcint_vector[vec].time);								// callback the interrupt function in user sketch
	}

	pcint_vector[vec].chng = pcint_vector[vec].curr;										// remember the current status to see the change next time
}


/* interrupt vectors to catch pin change interrupts */
#ifdef PCIE0
ISR(PCINT0_vect) {
	maintain_PCINT(0);
}
#endif

#ifdef PCIE1
ISR(PCINT1_vect) {
	maintain_PCINT(1);
}
#endif

#ifdef PCIE2
ISR(PCINT2_vect) {
	maintain_PCINT(2);
}
#endif

#ifdef PCIE3
ISR(PCINT3_vect) {
	maintain_PCINT(3);
}
#endif
//- -----------------------------------------------------------------------------------------------------------------------







