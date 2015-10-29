/*****************************************************************************
 *   Assignment 2
 *
 *   Daryl Yong & Lee Kai Yi, EE2024 Lab B3
 *
 ******************************************************************************/
//normal to-dos
//CAN YOU INTERRUPT BASED ON MSTICKS FOR RGB BLINKY?
//EXPLAIN THRESHOLD TO ENTER SURVIVAL MODE

//rising edge for detection of light sensor < 3000lux
//write light_read in a better way
//queue system for counting lightning
//sw1 that allows the baseboard to reinitialize tilt value

//assignment hints:
//modify temp_read in a better way, rewrite it because you use a polling approach,
//can use interrupt approach to rewrite it (every few miliseconds check, then figure out duty cycle?)


//############# JUMPER CONFIGURATION ###############
//take out from j28 - enable sw4
//put into j36 - enable light sensor
//j13 from BLEN to PIO2_9 - use SW3 interrupt
//remove j23 jumper at PIO1_10 to green RGB - disable green RGB




//include brief summary of program functioning

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "stdio.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "temp.h"
#include "led7seg.h"
#include "light.h"


/* ################# ASSIGNMENT SPECIFICATIONS ################# */

#define SAMPLING_TIME 2000 // ms, intervals sensor senses in explorer mode
#define LIGHTNING_TIME_WINDOW 3000 // ms
#define LIGHTNING_MONITORING 3000 // lux
#define LIGHTNING_THRESHOLD 3000 // lux
#define TIME_UNIT 250 // ms

static uint32_t RESET_INITIAL = 1;
static uint32_t INITIAL_TIME = 0;
static uint32_t CURRENT_TIME = 0;
static uint32_t SEGMENT_DISPLAY = 0;

static uint8_t INDICATOR_EXPLORER = RGB_BLUE;
static uint8_t INDICATOR_SURVIVAL = RGB_RED;

static const char ENTER_SURVIVAL_MESSAGE[] = "Lightning Detected. Scheduled Telemetry is Temporarily Suspended.\n";
static const char EXIT_SURVIVAL_MESSAGE[] = "Lightning Has Subsided. Scheduled Telemetry Will Now Resume.\n";


/* ################# GLOBAL CONSTANTS ################# */
// define available operation modes
#define EXPLORER_MODE 0
#define SURVIVAL_MODE 1

//define lowest timeframe used to check in ms
#define TIMEFRAME 50
#define INTERUPTS_PER_SEC 20 //1000MS/50MS


/* ################# GLOBAL VARIABLES ################# */
static int OPERATION_MODE = EXPLORER_MODE; // default starting operation mode explorer

/* Accelerometer Variables */
static int32_t xoff = 0;
static int32_t yoff = 0;
static int32_t zoff = 0;

static int8_t x = 0;
static int8_t y = 0;
static int8_t z = 0;

// FIFO Array containing timestamp of last 9 lightning flashes
// first element is oldest value, last element is newest
static uint32_t recentFlashes[9] = {0,0,0,0,0,0,0,0,0};

// Timestamp checking beginning interrupt end interrupt of each lightning flash > LIGHTNING_THRESHOLD
// Lightning flash only counted if flashEnd-flashBeginning<500ms
static uint32_t flashBeginning = 0;
static uint32_t flashEnd = 0;

static int reset_led_countdown = 0; // set to '1' if >3000 lux interrupt during survival mode

/* ################# DEFINING AND WRAPPING OF TIMER ################### */

volatile uint32_t msTicks;

void SysTick_Handler(void){
	msTicks++;
}

uint32_t getMsTicks()
{
	return msTicks;
}

/* ################# DEFINING SWITCHING ON AND OFF RGB ################### */

void new_rgb_setLeds (uint8_t ledMask) // self defined function to blink RGB
{
	// turn on RGB_RED only if off, else turn off RGB_RED
    if ((ledMask & RGB_RED) != 0 && (GPIO_ReadValue(2) & 0x01) == 0) {
        GPIO_SetValue( 2, 1); // write a 1 instead of 0
    } else {
        GPIO_ClearValue( 2, 1);
    }
    // turn on RGB_BLUE only if off, else turn off RGB_BLUE
    if ((ledMask & RGB_BLUE) != 0 && (GPIO_ReadValue(0) & (1<<26)) == 0) {
        GPIO_SetValue( 0, (1<<26) );
    } else {
        GPIO_ClearValue( 0, (1<<26) );
    }
    // turn on RGB_GREEN only if off, else turn off RGB_GREEN
    if ((ledMask & RGB_GREEN) != 0 && (GPIO_ReadValue(2) & (1<<1)) == 0) {
        GPIO_SetValue( 2, (1<<1) );
    } else {
        GPIO_ClearValue( 2, (1<<1) );
    }
}

/* ################# DEFINING FUNCTION AND NOTES FOR SPEAKER ################### */

#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);

static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }
    }
    else {
    	Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

static uint32_t getNote(uint8_t ch)
{
    if (ch >= 'A' && ch <= 'G')
        return notes[ch - 'A'];
    if (ch >= 'a' && ch <= 'g')
        return notes[ch - 'a' + 7];
    return 0;
}

static uint32_t getDuration(uint8_t ch)
{
    if (ch < '0' || ch > '9')
        return 400;
    /* number of ms */
    return (ch - '0') * 200;
}

static uint32_t getPause(uint8_t ch)
{
    switch (ch) {
    case '+':
        return 0;
    case ',':
        return 5;
    case '.':
        return 20;
    case '_':
        return 30;
    default:
        return 5;
    }
}

static void playSong(uint8_t *song) {
    uint32_t note = 0;
    uint32_t dur  = 0;
    uint32_t pause = 0;

    /*
     * A song is a collection of tones where each tone is
     * a note, duration and pause, e.g.
     *
     * "E2,F4,"
     */

    while(*song != '\0') {
        note = getNote(*song++);
        if (*song == '\0')
            break;
        dur  = getDuration(*song++);
        if (*song == '\0')
            break;
        pause = getPause(*song++);

        playNote(note, dur);
        //delay32Ms(0, pause);
        Timer0_Wait(pause);

    }
}

static uint8_t * song = (uint8_t*)"C2.C2,D4,C4,F4,E8,";
        //(uint8_t*)"C2.C2,D4,C4,F4,E8,C2.C2,D4,C4,G4,F8,C2.C2,c4,A4,F4,E4,D4,A2.A2,H4,F4,G4,F8,";
        //"D4,B4,B4,A4,A4,G4,E4,D4.D2,E4,E4,A4,F4,D8.D4,d4,d4,c4,c4,B4,G4,E4.E2,F4,F4,A4,A4,G8,";

/* ################# INITIALIZING SSP, I2C, GPIO ################### */

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
	// initialize SW4 (default config)
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(1, 1<<31, 0);
}


/* ################# INITIALIZING PERIPHERALS ################### */

static int32_t readTempSensor(void) {
	int32_t temperature = temp_read();
	return temperature/10.0; // temperature in Celsius
}

static int32_t readLightSensor(void) {
	int32_t light_value = light_read();
	return light_value; // light intensity in lux
}

static int32_t * readAccelerometer(void) {
    /*
     * Assume base board in zero-g position when reading first value.
     */
	static int32_t xyz_values[3];

    acc_read(&x, &y, &z);
    x = x+xoff;
    y = y+yoff;
    z = z+zoff;

    xyz_values[0] = x;
    xyz_values[1] = y;
    xyz_values[2] = z;
    return xyz_values; // return array
}

void rgbBlinky (void) {
	uint32_t rgbColor;
	if (OPERATION_MODE == EXPLORER_MODE) {
		rgbColor = INDICATOR_EXPLORER; // blue
		new_rgb_setLeds(rgbColor);
	} else {
		rgbColor = INDICATOR_SURVIVAL; // red
		new_rgb_setLeds(rgbColor);
	}
}

/* ################# INITIALIZING HOPE ################### */

static void initializeHOPE(void) {
	SEGMENT_DISPLAY = '0';
	while (SEGMENT_DISPLAY != 'G') { // increment until segment displays 'F'
		led7seg_setChar(SEGMENT_DISPLAY, FALSE); // display new incremented value
		Timer0_Wait(1000); // 1s interval
		if (SEGMENT_DISPLAY == '6') {
			SEGMENT_DISPLAY = 'A'; // change from number to alphabet
		} else {
			SEGMENT_DISPLAY += 1; // increment
		}
	}
	led7seg_setChar(NULL, FALSE); // clear 7 segment display
}


/* ################# INITIALIZING INTERUPTS ################### */

void EINT3_IRQHandler(void) {
	if ((LPC_GPIOINT->IO2IntStatF >> 5) & 0x1) { // light sensor >3000 lux interrupt
		flashBeginning = getMsTicks();
		reset_led_countdown = 1;
		LPC_GPIOINT->IO2IntClr = (1<<5);
		light_clearIrqStatus();
	}

	if ((LPC_GPIOINT->IO2IntStatR >> 5) &0x1) { // light sensor <3000 lux interrupt
		flashEnd = getMsTicks();
		if (flashEnd - flashBeginning < 500) {
			uint32_t tempArray[9];
			memcpy(tempArray, recentFlashes, 9); // copy values into tempArray
			int i=0;
			while (i<9) {
				recentFlashes[i] = tempArray[i+1];
				i++;
			}
			recentFlashes[9] = flashEnd; // push new value in
		}
		LPC_GPIOINT->IO2IntClr = (1<<5);
		light_clearIrqStatus();
	}

	if ((LPC_GPIOINT->IO2IntStatF >> 10) & 0x1) { // SW3 interrupt
		playSong(song);
		LPC_GPIOINT->IO2IntClr = (1<<10);
	}
}

/* ################# COUNTERS FOR TASKS ################### */
static const int RGB_COUNTER_LIMIT = (1000+TIMEFRAME-1)/TIMEFRAME; // to obtain ceiling of division of 1000ms by TIMEFRAME
static int rgb_Counter = 0;

/* ################# TASK HANDLERS ################### */

static void explorerTasks(void){


	char Array[20]; //array to write

	//char output[] = "L%d_T%d_AX%d_AY%d_AZ%d\n";
	//int timeCounter = 0; //initialize time counter, increments every 500ms
	//static int TIME_TOGGLE_LED = 2; //1000ms/500ms=2
	//static int TIME_TOGGLE_INDICATOR = SAMPLING_TIME/500; //2000ms/500ms=4
/*
		if(timeCounter%TIME_TOGGLE_LED==0){
			toggleLEDS();
		}

		if(timeCounter%TIME_TOGGLE_INDICATOR==0){
			refreshIndicator();
		}

		while(CURRENT_TIME - INITIAL_TIME < 500){
			CURRENT_TIME = getMsTicks();
		}//time base of 0.5 seconds
		timeCounter++;
		INITIAL_TIME = CURRENT_TIME;
*/
		int32_t light_value = readLightSensor();
		int32_t temp_value = readTempSensor();
		int32_t *xyz_values;
		xyz_values = readAccelerometer();
		sprintf(Array, "L%d_T%d_AX%d_AY%d_AZ%d\n", light_value, temp_value,
				*(xyz_values), *(xyz_values+1), *(xyz_values+2));
		//printValues(light_value,temp_value,*xyz_values);
		oled_putString(0,20,(uint8_t*)Array,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		INITIAL_TIME = CURRENT_TIME;
		// TRANSMIT DATA TO HOME
		// CONDITION TO EXIT EXPLORER

}

/*
//function to print values to oled screen, in 3 different lines
void printValues(int32_t light_value, int32_t temp_value, int32_t *xyz_values){
	char lightArray[20];
	char tempArray[20];
	char xyzArray[20];
	sprintf(lightArray, "L%d", light_value);
	sprintf(tempArray, "T%d", temp_value);
	sprintf(xyzArray, "AX%d_AY%d_AZ%d", *(xyz_values), *(xyz_values+1), *(xyz_values+2));
	oled_putString(0,0,(uint8_t*)lightArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,20,(uint8_t*)tempArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,(uint8_t*)xyzArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}*/

static void survivalTasks(void) {
    uint16_t ledOn = 0xffff;
	INITIAL_TIME = getMsTicks();
	CURRENT_TIME = getMsTicks();

    while (OPERATION_MODE == SURVIVAL_MODE) {
        pca9532_setLeds(ledOn, 0xffff);
        int32_t light_value = readLightSensor();
        while (light_value < LIGHTNING_MONITORING) {
            while (CURRENT_TIME - INITIAL_TIME < TIME_UNIT) { // 250ms interval
            	CURRENT_TIME = getMsTicks();
            }
            ledOn = ledOn >> 1; // MSB = 0 -> turn off one LED
            pca9532_setLeds(ledOn, 0xffff);
            INITIAL_TIME = CURRENT_TIME;
            if (ledOn == 0) {
            	break; // break out of loop if all LEDs are off
            }
            light_value = readLightSensor(); // refresh light_value
        }
        ledOn = 0xffff; // failed condition, restart LED sequence
        reset_led_countdown = 0;
    }
    OPERATION_MODE = EXPLORER_MODE; // return to explorer mode
}

static void genericTasks(void){
	rgb_Counter++;
	if (rgb_Counter == RGB_COUNTER_LIMIT) {
		rgbBlinky();
		rgb_Counter = 0;
	}
}

int main (void) {

    init_i2c();
    init_ssp();
    init_GPIO();

    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
	led7seg_init();
    temp_init(&getMsTicks);

    // Setup Interrupt for Light Sensor
    light_enable();
    light_setRange(LIGHT_RANGE_4000); // sensing up to 3892 lux
    light_setHiThreshold(LIGHTNING_THRESHOLD); // condition for interrupt
    light_setIrqInCycles(LIGHT_CYCLE_1);
    light_clearIrqStatus();

    LPC_GPIOINT->IO2IntEnF |= 1 << 5; // light sensor >3000 lux interrupt (P2.5)
    LPC_GPIOINT->IO2IntEnR |= 1 << 5; // light sensor <3000 lux interrupt (P2.5)
    LPC_GPIOINT->IO2IntEnF |= 1 << 10; // SW3 (P2.10)
    NVIC_EnableIRQ(EINT3_IRQn);

    if(SysTick_Config(SystemCoreClock/1000)){
    	while(1); //capture error
    }

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z; // saving initial value of z axis of accelerometer

    /* ---- Speaker ------> */

    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn

    // Initialize OLED
    oled_clearScreen(OLED_COLOR_BLACK);

	initializeHOPE();

	INITIAL_TIME = getMsTicks();
	CURRENT_TIME = getMsTicks();

    while (1)
    {
    	//checking for the condition that the below execution exceeds our TIMEFRAME
    	if(CURRENT_TIME - INITIAL_TIME > TIMEFRAME){
    		while(1); //throw error, get stuck in this loop
    	}

    	//delay until the next TIMEFRAME INTERUPT
    	while (CURRENT_TIME - INITIAL_TIME < TIMEFRAME) { // 50MS INTERVAL
    		CURRENT_TIME = getMsTicks();
    	}
    	INITIAL_TIME = CURRENT_TIME;

    	genericTasks();
    	if(OPERATION_MODE == EXPLORER_MODE){
    		explorerTasks();
    	} else {
    		survivalTasks();
    	}
    }
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
