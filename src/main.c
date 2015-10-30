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
#include "lpc17xx_uart.h"
#include "stdio.h"
#include <inttypes.h>

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

#define INDICATOR_EXPLORER RGB_BLUE
#define INDICATOR_SURVIVAL RGB_RED

static uint32_t CURRENT_TIME = 0;
static uint32_t SEGMENT_DISPLAY = '0';

static const char ENTER_SURVIVAL_MESSAGE[] = "Lightning Detected. Scheduled Telemetry is Temporarily Suspended.\n";
static const char EXIT_SURVIVAL_MESSAGE[] = "Lightning Has Subsided. Scheduled Telemetry Will Now Resume.\n";

/* ################# GLOBAL CONSTANTS ################# */
// define available operation modes
#define EXPLORER_MODE 0
#define SURVIVAL_MODE 1

// map timed functions to timer number
#define PCA9532 1
#define RGB 2
#define SAMPLING 3

/* ################# GLOBAL VARIABLES ################# */
static int OPERATION_MODE = SURVIVAL_MODE; // default starting operation mode explorer

/* Accelerometer Variables */
static int32_t xoff = 0;
static int32_t yoff = 0;
static int32_t zoff = 0;

static int8_t x = 0;
static int8_t y = 0;
static int8_t z = 0;

static uint16_t ledOn = 0xffff; // pca9532 led bit pattern
static int RGB_ON = 0;

// FIFO Array containing timestamp of last 9 lightning flashes
// first element is newest value, last element is oldest
static uint32_t recentFlashes[9] = {0,0,0,0,0,0,0,0,0};
static int recentFlashesStackPointer = -1; // no values stored yet

// Timestamp checking beginning interrupt end interrupt of each lightning flash > LIGHTNING_THRESHOLD
// Lightning flash only counted if flashEnd-flashBeginning<500ms
static uint32_t flashBeginning = 0;
static uint32_t flashEnd = 0;
static int aboveThreshold = 0;

/* ################# INTERRUPT HANDLER FLAGS ################### */

static int RESET_LED_COUNTDOWN = 0; // flag set to '1' if >3000 lux interrupt during survival mode
static int LIGHTNING_THRESHOLD_FLAG = 0; //flag is raised when LIGHTNIGN THRESHOLD is raised in interupts
static int SW3_FLAG = 0;
static int PCA9532_LED_COUNTDOWN_FLAG = 0;
static int SAMPLING_FLAG = 0;
static int UPDATE7SEG_FLAG = 0;

/* ################# INTERRUPT PRIORITY SETTING ################### */

void init_Priority(void){

	uint32_t priority, PG = 5, PP, SP; // priority grouping, pre-empt priority, subpriority
	NVIC_SetPriorityGrouping(5);

	PP = 0, SP = 0;
	priority = NVIC_EncodePriority(PG,PP,SP);
	NVIC_SetPriority(SysTick_IRQn, priority);
	PP = 1, SP = 0;
	priority = NVIC_EncodePriority(PG,PP,SP);
	NVIC_SetPriority(EINT3_IRQn, priority); // light sensor and SW3
	// interrupt with smallest time interval is given higher priority
	PP = 2, SP = 0;
	priority = NVIC_EncodePriority(PG,PP,SP);
	NVIC_SetPriority(TIMER1_IRQn, priority); // pca9532 led (250ms)
	PP = 2, SP = 1;
	priority = NVIC_EncodePriority(PG,PP,SP);
	NVIC_SetPriority(TIMER2_IRQn, priority); // rgb (1s)
	PP = 2, SP = 2;
	priority = NVIC_EncodePriority(PG,PP,SP);
	NVIC_SetPriority(TIMER3_IRQn, priority); // sampling (2s)

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_ClearPendingIRQ(TIMER3_IRQn);

	NVIC_EnableIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
}

/* ################# DEFINING AND SYSTICK ################### */

volatile uint32_t msTicks;

void SysTick_Handler(void){
	msTicks++;
}

uint32_t getMsTicks()
{
	return msTicks;
}

/* ################# DEFINING SWITCHING ON AND OFF RGB ################### */

void new_rgb_setLeds (uint8_t ledMask) // self defined function to blink RGB depending on OPERATION_MODE
{
	if (RGB_ON == 1) {
		GPIO_ClearValue( 2, 1);
		GPIO_ClearValue( 0, (1<<26) );
	} else {
		if ((ledMask & RGB_RED) != 0) {
			GPIO_SetValue( 2, 1); // write a 1 instead of
		}
		if ((ledMask & RGB_BLUE) != 0) {
			GPIO_SetValue( 0, (1<<26) );
		}
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

	// init rgb to output
    GPIO_SetDir( 2, 1, 1 );
    GPIO_SetDir( 0, (1<<26), 1 );
    GPIO_SetDir( 2, (1<<1), 1 );
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
	if (OPERATION_MODE == EXPLORER_MODE) {
		new_rgb_setLeds(INDICATOR_EXPLORER); // blue
	} else {
		new_rgb_setLeds(INDICATOR_SURVIVAL); // red
	}
}

void init_Speaker(void) {
    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27); //LM4811-clk
    GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
    GPIO_ClearValue(2, 1<<13); //LM4811-shutdn
}

/* ################# INITIALIZING TIMER ################### */

// timerNumber is in range 1-3
static void enableTimer(int timerNumber, uint32_t time) {
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

	// Initialize timer 0, prescale count time of 1ms
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1000;
	// use channel 0, MR0
	TIM_MatchConfigStruct.MatchChannel = 0;
	// Enable interrupt when MR0 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR0: TIMER will reset if interrupt triggered
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Continue running after interrupt has occurred
	TIM_MatchConfigStruct.StopOnMatch = FALSE;
	//do no thing for external output
	TIM_MatchConfigStruct.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	// Set Match value, count value is time (timer * 1000uS =timer mS )
	TIM_MatchConfigStruct.MatchValue = time;

	LPC_TIM_TypeDef *TIMx; // select Timer0, Timer1 or Timer2
	if (timerNumber == 1) {
		TIMx = LPC_TIM1;
	} else if (timerNumber == 2) {
		TIMx = LPC_TIM2;
	} else if (timerNumber == 3) {
		TIMx = LPC_TIM3;
	}

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(TIMx,TIM_TIMER_MODE,&TIM_ConfigStruct);
	TIM_ConfigMatch(TIMx,&TIM_MatchConfigStruct);
	TIM_Cmd(TIMx, ENABLE);
}

static void disableTimer(int timerNumber) {
	LPC_TIM_TypeDef *TIMx;
	if (timerNumber == 1) {
		TIMx = LPC_TIM1;
	} else if (timerNumber == 2) {
		TIMx = LPC_TIM2;
	} else if (timerNumber == 3) {
		TIMx = LPC_TIM3;
	}
	TIM_DeInit(TIMx);
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
	if(readLightSensor() > LIGHTNING_THRESHOLD) {
		aboveThreshold = 1;
	}
}

/* ################# INITIALIZING INTERUPTS ################### */

void TIMER1_IRQHandler(void){
	PCA9532_LED_COUNTDOWN_FLAG = 1;
	TIM_ClearIntPending(LPC_TIM1,0);
}

void TIMER2_IRQHandler(void){
	// toggle on-off
	if (RGB_ON == 0) {
		RGB_ON = 1;
	} else {
		RGB_ON = 0;
	}
	rgbBlinky();
	TIM_ClearIntPending(LPC_TIM2,0);
}

void TIMER3_IRQHandler(void){
	SAMPLING_FLAG = 1;
	TIM_ClearIntPending(LPC_TIM3,0);
}

void EINT3_IRQHandler(void) {
	if ((LPC_GPIOINT->IO2IntStatF >> 5) & 0x1) { // light sensor cross 3000 lux threshold interrupt
		if (aboveThreshold) {
			flashEnd = getMsTicks();
			if (flashEnd - flashBeginning < 500) {
				LIGHTNING_THRESHOLD_FLAG = 1;
				UPDATE7SEG_FLAG = 1; // genericTasks() will increment 7 segment display
				if (SEGMENT_DISPLAY != '9') { // limit 7 segment display to '9'
					SEGMENT_DISPLAY += 1;
				}
			}
			aboveThreshold = 0;
		   // uint16_t a=flashEnd;
		   // printf("flash End: %" PRIu32 "\n",a);
		} else {
			flashBeginning = getMsTicks();
			if (OPERATION_MODE == SURVIVAL_MODE) {
				RESET_LED_COUNTDOWN = 1;
			}
			aboveThreshold = 1;
		    //uint16_t a=flashBeginning;
		    //printf("flash begin: %" PRIu32 "\n",a);
		}
		LPC_GPIOINT->IO2IntClr = (1<<5);
		light_clearIrqStatus();
	}

	if ((LPC_GPIOINT->IO2IntStatF >> 10) & 0x1) { // SW3 interrupt
		SW3_FLAG = 1;
		LPC_GPIOINT->IO2IntClr = (1<<10);
	}
}

void init_Interrupts(void) {
    // Setup Interrupt for Light Sensor
    light_enable();
    light_setRange(LIGHT_RANGE_4000); // sensing up to 3892 lux
    light_setHiThreshold(LIGHTNING_THRESHOLD); // condition for interrupt
    light_setIrqInCycles(LIGHT_CYCLE_1);
    light_clearIrqStatus();

    LPC_GPIOINT->IO2IntEnF |= 1 << 5; // light sensor 3000 lux interrupt (P2.5)
    LPC_GPIOINT->IO2IntEnF |= 1 << 10; // SW3 (P2.10)
}

static char Array[20]; //array to write

/* ################# TASK HANDLERS ################### */

static void explorerTasks(void){
	if (SAMPLING_FLAG == 1) {
		SAMPLING_FLAG = 0;
		int32_t light_value = readLightSensor();
		int32_t temp_value = readTempSensor();
		int32_t *xyz_values;
		xyz_values = readAccelerometer();
		printValues(light_value, temp_value, xyz_values);
		// TRANSMIT DATA TO HOME
	}
}

//function to print values to oled screen, in 3 different lines
void printValues(int32_t light_value, int32_t temp_value, int32_t *xyz_values){
	char lightArray[20];
	char tempArray[20];
	char xyzArray[20];
	sprintf(lightArray, "L%d", light_value);
	sprintf(tempArray, "T%d", temp_value);
	sprintf(xyzArray, "AX%d_AY%d_AZ%d", (int) *(xyz_values), (int) *(xyz_values+1),(int) *(xyz_values+2));
	oled_putString(0,0,(uint8_t*)lightArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,20,(uint8_t*)tempArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,(uint8_t*)xyzArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

static void survivalTasks(void) {

	if (RESET_LED_COUNTDOWN == 0) {
		if (PCA9532_LED_COUNTDOWN_FLAG == 1) {
			PCA9532_LED_COUNTDOWN_FLAG = 0;
			ledOn = ledOn >> 1;
		}
	} else {
		RESET_LED_COUNTDOWN = 0;
		ledOn = 0xffff;
	}

	pca9532_setLeds(ledOn, 0xffff);

	if (ledOn == 0) { // no lightning in past 4s
		OPERATION_MODE = EXPLORER_MODE;
	}
}

static void genericTasks(void){

	//DOES NOT WORK, ASSUMES LIGHTNING THRESHOLD CAN ONLY BE RAISED ONCE EVERY 50MS
	//NEED TO CHANGE TO A DIFFERENT IMPLEMENTATION! (STORE LIGHTNING THRESHOLD IN ARRAY,
	//COUNT DOWN LIGHTNING THRESHOLD FLAG?
	if(LIGHTNING_THRESHOLD_FLAG == 1){
		LIGHTNING_THRESHOLD_FLAG=0; //RESET LIGHTNING THRESHOLD FLAG
		int i=7;
		while(i>=0){
			recentFlashes[i+1]=recentFlashes[i];
			i--;
		}
		recentFlashes[0] = flashEnd; // push new value in
		recentFlashesStackPointer++;
	}

	//check if count of lightning flashes is correct
	CURRENT_TIME = getMsTicks();
	if (CURRENT_TIME > recentFlashes[recentFlashesStackPointer] + 3000) {
		recentFlashesStackPointer--;
		if (SEGMENT_DISPLAY != '0') { // minimum display '0'
			SEGMENT_DISPLAY -= 1;
		}
		UPDATE7SEG_FLAG = 1;
	}

	if (UPDATE7SEG_FLAG == 1) {
		UPDATE7SEG_FLAG = 0;
		led7seg_setChar(SEGMENT_DISPLAY, FALSE);
	}

	if ((recentFlashesStackPointer >= 3) && (OPERATION_MODE == EXPLORER_MODE)) {
		OPERATION_MODE = SURVIVAL_MODE;
        oled_clearScreen(OLED_COLOR_BLACK);
	}
	if (SW3_FLAG == 1) {
		SW3_FLAG = 0;
	}
}

int main (void) {

    init_i2c();
    init_ssp();
    init_GPIO();
    init_Priority();

    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
	led7seg_init();
    temp_init(&getMsTicks);

    if(SysTick_Config(SystemCoreClock/1000)){
    	while(1); //capture error
    }

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0;
    yoff = 0;
    zoff = 0-z;

    // Initialize OLED
    oled_clearScreen(OLED_COLOR_BLACK);

	//initializeHOPE();
    init_Interrupts();
	enableTimer(RGB, 1000); // RGB will blink throughout operation
	enableTimer(SAMPLING, 2000); //enable sampling timer when first going into loop
	enableTimer(PCA9532, 250);

    while (1)
    {
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
