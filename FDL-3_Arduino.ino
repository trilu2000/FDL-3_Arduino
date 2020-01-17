/*- -----------------------------------------------------------------------------------------------------------------------
*  FDL-3 arduino implementation
*  2018-01-17 <trilu@gmx.de> Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
* - -----------------------------------------------------------------------------------------------------------------------
* - main sketch -----------------------------------------------------------------------------------------------------------
*   special thanks to Jesse Kovarovics http://www.projectfdl.com to make this happen
* - -----------------------------------------------------------------------------------------------------------------------
*/

#define DEBUG

#include <U8g2lib.h>
#include <Servo.h>
#include "battery.h"
#include "buzzer.h"
#include "launcher.h"
#include "pusher.h"
#include "myfunc.h"
#include "mylogo.h"


#define fdl3_Fire      4 // 5 at old, 4 on new hardware

#define fdl3_Battery   17
#define fdl3_Buzzer    5 // 5 on new hardware, not available on old hw
#define launcher_ESC   10

#define encoder_A      14
#define encoder_B      15
#define encoder_Click  16

#define fdl3_QClick1   2
#define fdl3_QClick2   3
#define fdl3_QClick3   8

#define pusher_PWM     11
#define pusher_Standby 9
#define pusher_Dir1    13
#define pusher_Dir2    12

#define pusher_FRNT    7
#define pusher_BACK    6

#define BEEP_INIT  20, 100, 2																					// on start of the device
#define BEEP_SHORT 20																									// single short beep as confirmation for key press 
#define BEEP_LONG 300																									// single long beep for eeprom save
#define BEEP_BATTERY  50, 100, 5																			// if battery goes low


// - everything settings related -------------------------------------
struct struct_profile_store {
	uint8_t  mode = 2;																									// how many darts per fire push
	uint8_t  speed = 80;																								// fire speed in % of max_speed
};

struct struct_setting_store {
	uint8_t  profile        : 2;																				// active profile
	uint8_t  buzzer_menu    : 1;																				// menu buzzer is enabled
	uint8_t  buzzer_battery : 1;																				// battery alarm buzzer is enabled per default
	uint8_t                 : 4;
	uint16_t speedup_time;																							// motor speedup time in millis
	uint8_t  standby_speed;																							// standby speed in percent of max_speed
	uint16_t standby_time;																							// standby time in millis
	uint16_t battery_adj;																								// adjustment factor for battery measurement
};

struct_profile_store* profile_ptr;
struct_profile_store  profile_store[4];																// one main set, plus 3 quick set buttons
struct_setting_store  setting_store = {0, 1, 0, 500, 50, 500, 1000};	// other settings like timing and buzzer...

EEpromClass eeprom;

// - everything encoder related --------------------------------------
EncoderClass encoder(encoder_A, encoder_B);
ButtonClass encoder_click(encoder_Click);
TimerClass encoder_timeout;

// - launcher to speed up the darts and ------------------------------
LauncherClass launcher(launcher_ESC, 1000, 2000);
/* pusher to shift the dart into the launcher */
PusherClass pusher(pusher_Dir1, pusher_Dir2, pusher_PWM, pusher_Standby, pusher_FRNT, pusher_BACK, launcher.ready);

// - display related -------------------------------------------------
// https://github.com/olikraus/u8g2/wiki/u8g2install
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
TimerClass display_timer;
uint8_t display_timeout;
uint8_t display_mode;
uint8_t menu_item, menu_select;

// - buzzer related --------------------------------------------------
BuzzerClass buzzer(fdl3_Buzzer);

// - battery related -------------------------------------------------
class BatteryClass : public BatteryBaseClass {
public:
	BatteryClass(uint8_t pin, uint16_t *factor, uint8_t alarm = 30) : BatteryBaseClass(pin, factor, alarm) {}
	void alarm() {
		if (!setting_store.buzzer_battery) return;
		buzzer.on(BEEP_BATTERY);
		dbg << F("battery alarm\n");
	}
};
BatteryClass battery(fdl3_Battery, &setting_store.battery_adj);

// - push and selection button related -------------------------------
ButtonClass qclick1(fdl3_QClick1);
ButtonClass qclick2(fdl3_QClick2);
ButtonClass qclick3(fdl3_QClick3);
ButtonClass fld3_fire(fdl3_Fire);

// -------------------------------------------------------------------





// - program section -------------------------------------------------

void setup() {
	dbg.begin(115200);
	dbg << F("\n\n\nFDL-3 Arduino v0.3\n\n");														// init serial interface and some debug

	u8g2.begin();																												// init the display and show some start message
	display_welcome();																									// show the welcome screen
	display_timer.set(5000);																						// schedule next regular update

	battery.init();																											// init the battery measurement
	launcher.init();																										// init the launcher
	pci_ptr = pci_ISR;																									// assign pin interrupt collector

	/* read defaults from eeprom in case that the magic number is ok */
	dbg << F("read settings from eeprom\n");

	uint16_t magic;
	eeprom.get(0, 2, &magic);
	if (magic != 0x1238) {																							// magic number doesn't fit, so a new config needs written
		magic = 0x1238;																										// set a magic number
		eeprom.set( 0, 2, &magic);																				// write the magic to the eeprom
		eeprom.set( 2, 8, &setting_store);																// write the others settings as default
		eeprom.set(10, 8, &profile_store);																// write the function settings
		dbg << F("magic doesn't fit, write defaults\n");
	}

	eeprom.get( 2, 8, &setting_store);																	// read the others settings 
	eeprom.get(10, 8, &profile_store);																	// read the function settings
	
	profile_ptr = &profile_store[setting_store.profile];								// assign profile pointer to current profile store
	pusher.mode = &profile_ptr->mode;																		// how many darts per fire push
	launcher.fire_speed = &profile_ptr->speed;													// fire speed in % of max_speed

	launcher.speedup_time = &setting_store.speedup_time;								// holds the time the motor needs to speedup
	launcher.standby_speed = &setting_store.standby_speed;							// standby speed in % of max_speed
	launcher.standby_time = &setting_store.standby_time;								// standby time in ms
	
	buzzer.init();
	buzzer.on(BEEP_INIT);

	dbg << F("init complete, mode: ") << *pusher.mode << F(", speed: ") << profile_ptr->speed << F(", speedup_time: ") << setting_store.speedup_time << F(", standby_speed: ") << setting_store.standby_speed << F(", standby_time: ") << setting_store.standby_time << F("\n\n");
}


void loop() {

	/* general poll function */
	pusher.poll();
	launcher.poll();
	buzzer.poll();
	battery.poll();


	/* poll the status display - updates are triggered by the encoder and quick buttons as well */
	if (display_timer.done()) {
		display_status();
		display_timer.set(10000);
	}

	/* poll the encoder regulary */
	int8_t enc_value = encoder.getValue();															// check if the encoder value had changed
	if (enc_value > 0) encoder_up(enc_value);
	if (enc_value < 0) encoder_down(enc_value);
	
	encoder_button(encoder_click.getValue());														// check the encoder button

	if (encoder_timeout.done()) {
		menu_item = 0;
		menu_select = 0;
	}


	/* poll the quick buttons */
	quick_click(1, qclick1.getValue());
	quick_click(2, qclick2.getValue());
	quick_click(3, qclick3.getValue());


	/* check fire button continously and drive pusher */
	uint8_t fire_button = fld3_fire.getValue();
	if (fire_button == 1) {
		pusher.start();
		launcher.start();
		dbg << F("M::Fire button pushed\n");

	} else if (fire_button == 2) {
		pusher.stop();
		launcher.stop();
		dbg << F("M::Fire button released\n");
	}

}


void encoder_up(int8_t x) {

	if (menu_select == 0) {
		if (menu_item >= 2) menu_item = 0;
		else menu_item++;
		//dbg << F("u: ") << menu_item << '\n';
	}

	if (menu_select == 1) {
		if (menu_item == 1) {
			profile_ptr->mode += 1;
			if (profile_ptr->mode > 3) profile_ptr->mode = 0;
		}
		if (menu_item == 2) {
			profile_ptr->speed += 5;
			if (profile_ptr->speed  > 100) profile_ptr->speed = 100;
		}
	}
	display_status();
	encoder_timeout.set(5000);
}


void encoder_down(int8_t x) {
	if (menu_select == 0) {
		if (menu_item == 0) menu_item = 2;
		else menu_item--;
		//dbg << F("d: ") << menu_item << '\n';
	}

	if (menu_select == 1) {
		if (menu_item == 1) {
			if (profile_ptr->mode == 0) profile_ptr->mode = 3;
			else profile_ptr->mode -= 1;
		}
		if (menu_item == 2) {
			profile_ptr->speed -= 5;
			if (profile_ptr->speed  < 50) profile_ptr->speed = 50;
		}
	}
	display_status();
	encoder_timeout.set(5000);
}


void encoder_button(int8_t status) {
	if (status != 1) return;																						// we use only a button press event currently
	if (setting_store.buzzer_menu) buzzer.on(100);											// some beep while button pressed

	if (!menu_item) {																										// no item is selected
		quick_click(0, status);																						// select a profile
		return;
	}
	menu_select++;																											// increase the select

	if (menu_select >= 2) {																							// menu select 2 means, we are in edit mode
		if (setting_store.buzzer_menu) buzzer.on(BEEP_LONG);							// some beep while settings are written
		eeprom.set(10, 8, &profile_store);																// write the settings
		menu_select = 0;																									// back for a new select
	}
	//dbg << F("p: ") << x << F(", ") << menu_select << '\n';

	display_status();
	encoder_timeout.set(5000);
}


void quick_click(uint8_t number, uint8_t status) {
	if (status != 1) return;

	setting_store.profile = number;																			// set the current profile number
	profile_ptr = &profile_store[setting_store.profile];									// assign profile pointer to current profile store
	pusher.mode = &profile_ptr->mode;																		// how many darts per fire push
	launcher.fire_speed = &profile_ptr->speed;													// fire speed in % of max_speed

	if (setting_store.buzzer_menu) buzzer.on(BEEP_SHORT);								// some beep while button pressed
	display_status();

	dbg << F("speed: ") << *launcher.fire_speed << F(", mode: ") << *pusher.mode << '\n';
}


void display_status() {

	u8g2.firstPage();																										// reset the buffer page counter													

	do {																																// step through the different pages
		draw_battery(battery.level());																		// write the battery level

		u8g2.setFont(u8g2_font_7x14B_tr);																	// we use a different font for the menu

		u8g2.setCursor(0, 26);																						// set the curser to line 35 of 64

		u8g2.print(status_line_item(1));																	// get the status of the line item
		u8g2.print(F("Mode:  "));
		if (profile_ptr->mode == 0) u8g2.print(F("unlimited"));
		if (profile_ptr->mode == 1) u8g2.print(F("single"));
		if (profile_ptr->mode == 2) u8g2.print(F("double"));
		if (profile_ptr->mode == 3) u8g2.print(F("tripple"));

		u8g2.setCursor(0, 46);

		u8g2.print(status_line_item(2));																	// get the status of the line item
		u8g2.print(F("Speed: "));											
		u8g2.print(profile_ptr->speed);
		u8g2.print("%");

		u8g2.setCursor(0, 63);
		u8g2.setFont(u8g2_font_6x12_tr);																	// get the font into the memory
		u8g2.print(F("Profile: "));
		u8g2.print(setting_store.profile);

	} while (u8g2.nextPage());																					// step throug the buffer pages till the end

	//dbg << F("status display update ") << _TIME << '\n';
}


char status_line_item(uint8_t item_nr) {
	if (item_nr == menu_item) {																					// seems the item is selected
		if (menu_select) {																								// seems it is commited to change the value
			return '#';
		} else {																													// not commited for a change
			return '>';
		}

	} else {																														// other item, return a blank
		return ' ';
	}

}


/* draws the welcome screen on the display, battery level follows later */
void display_welcome() {
	u8g2.clear();
	//u8g2.clearDisplay();																							// clear the display
	u8g2.firstPage();																										// start with the first buffer

	do {																																// loop throug all buffers
		u8g2.drawXBMP(0, 0, 128, 64, testbmp_bits);												// and draw the bitmap
	} while (u8g2.nextPage());																					// till the last page
	dbg << F("show welcome screen ") << _TIME << '\n';									// some debug
}


void draw_battery(uint8_t battery) {
	// displaying the battery level uses the first 8 lines of the display 
	u8g2.setDrawColor(1);																								// sets the color white for the frame and the box
	u8g2.drawFrame(0, 0, 100, 6);																				// draws the frame
	u8g2.drawBox(0, 0, battery, 6);																			// fills the frame

	u8g2.setFont(u8g2_font_6x12_tr);																		// get the font into the memory
	uint8_t xpos = 117;																									// battery info should be always at the same position, independend of the levvel
	if (battery >= 10) xpos = 111;																			// differnet pos while 2 digits
	if (battery >= 100) xpos = 105;																			// again with 3 digits
	u8g2.setCursor(xpos, 8);	
	u8g2.print(battery, DEC);																						// and writes the battery level
	u8g2.print("%");
}

void pci_ISR (uint8_t pin, uint8_t status, uint32_t time) {
	// encoder class - encoder left/right turn
	if ((pin == encoder_A) || (pin == encoder_B)) encoder.irq(pin, status, time);

	// button related, as encoder, fire trigger, or quick select buttons
	else if (pin == encoder_Click) encoder_click.irq(status, time);
	else if (pin == fdl3_Fire)     fld3_fire.irq(status, time);
	else if (pin == fdl3_QClick1)  qclick1.irq(status, time);
	else if (pin == fdl3_QClick2)  qclick2.irq(status, time);
	else if (pin == fdl3_QClick3)  qclick3.irq(status, time);

	// link to pusher - front and back pusher sensor
	else if (pin == pusher_FRNT)   pusher.irq(pin, status, time);
	else if (pin == pusher_BACK)   pusher.irq(pin, status, time);


	//dbg << F("pin:") << pin << ',' << status << ',' << time << '\n';
}




