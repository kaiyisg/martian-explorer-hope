/*****************************************************************************
 *   Assignment 2
 *
 *   Daryl Yong & Lee Kai Yi, EE2024 Lab B3
 *
 *   HOPE is a project based on the LPC17Xpresso baseboard and ARM-styled
 *   architecture LPC1769 microcontrollers, and models the logic behind an
 *   autonomous rover on mars, with capabilities of monitoring and
 *   broadcasting information on its surroundings, and self preservation
 *   during dangerous situations.
 *
 *   HOPE’s key functions involve sending data captured from it’s
 *   accelerometer, light sensor, and temperature sensor, to a transmission
 *   relay, HOME, computer terminal connected to it through UART. Several of
 *   HOPE's key configurations can also be externally controlled by a GUI
 *   displayed on the OLED display, controlled by the joystick.
 *
 ******************************************************************************/


//############# JUMPER CONFIGURATION ###############
//take out from j28 - enable sw4
//put into j36 - enable light sensor
//j13 from BLEN to PIO2_9 - use SW3 interrupt
//remove j23 jumper at PIO1_10 to green RGB - disable green RGB
//remove j20 top two jumpers for UART
//put jumper in j58
//removed jumper B on J7

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
#include "rotary.h"


/* ############################################################# */
/* ################# ASSIGNMENT SPECIFICATIONS ################# */
/* ############################################################# */

#define SAMPLING_TIME 2000 // ms, intervals sensor senses in explorer mode
static uint32_t LIGHTNING_TIME_WINDOW = 3000; // ms, can be modified by user
static uint32_t USER_LIGHTNING_TIME_WINDOW;
#define LIGHTNING_MONITORING 3000 // lux
#define LIGHTNING_THRESHOLD 3000 // lux
#define TIME_UNIT 250 // ms
#define RANGE_K2 3892 // lux

static uint32_t CURRENT_TIME = 0;
static uint32_t SEGMENT_DISPLAY = '0';

static const char ENTER_SURVIVAL_MESSAGE[] =
		"Lightning Detected. Scheduled Telemetry is Temporarily Suspended.\r\n";
static const char EXIT_SURVIVAL_MESSAGE[] =
		"Lightning Has Subsided. Scheduled Telemetry Will Now Resume.\r\n";

/* ############################################################# */
/* ##################### GLOBAL CONSTANTS ###################### */
/* ############################################################# */

// define available operation modes
#define EXPLORER_MODE 0
#define SURVIVAL_MODE 1

// map timed functions to timer number
#define PCA9532 1
#define RGB 2
#define SAMPLING 3

/* ############################################################# */
/* ##################### GLOBAL VARIABLES ###################### */
/* ############################################################# */

static int OPERATION_MODE = EXPLORER_MODE; // default starting operation mode explorer

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

// keeps track of how many flashes in past LIGHTNING_TIME_WINDOW
static int recentFlashesStackPointer = -1; // -1 since there are no values in array initially
// size of recentFlashes array (currently set at 9)
static int recentFlashesSize = sizeof(recentFlashes)/sizeof(recentFlashes[0]);

// Timestamp checking beginning and end interrupt of each
// lightning flash that is > LIGHTNING_THRESHOLD
// Lightning flash only counted if flashEnd-flashBeginning<500ms
static uint32_t flashBeginning = 0;
static uint32_t flashEnd = 0;
static int aboveThreshold = 0;

//variable to control noise rejection for lightning detection
//controls the number of flashes required to enter survivor mode
static int flashesToEnterSurvival = 3;
static int flashesToEnterSurvivalDisplay;

//track rotary switch rotation: negative value is counter-clockwise
static int clockwise_rotation = 0;

//variables for OLED display
#define AVRG_READING_SELECTED 1
#define CONTROL_FLASHES_SELECTED 2
#define TIME_TO_SURVIVAL_SELECTED 3
static int explorerMainDisplaySelection = AVRG_READING_SELECTED; //default setting

#define MAIN_SCREEN 0
#define AVRG_READING_SCREEN 1
#define CONTROL_FLASHES_SCREEN 2
#define TIME_TO_SURVIVAL_SCREEN 3
static int explorerScreen = 0;

//values to track diagnostics and readings
static int32_t avrg_light_value;
static int32_t highest_light_value;
static int32_t lowest_light_value;
static int32_t avrg_temp_value;
static int32_t highest_temp_value;
static int32_t lowest_temp_value;
static int32_t avrg_xyz_values[3];
static int32_t highest_xyz_values[3];
static int32_t lowest_xyz_values[3];
static int diagnostic_runs_count = 0;
static int explorer_diagnostic_display_page = 1;

/* ############################################################# */
/* ################## DEFINING AND SYSTICK ##################### */
/* ############################################################# */

volatile uint32_t msTicks;

void SysTick_Handler(void){
	msTicks++;
}

uint32_t getMsTicks()
{
	return msTicks;
}

/* ############################################################# */
/* ################### INITIALIZING TIMER ###################### */
/* ############################################################# */

// timerNumber 1,2,3 uses Timer1,Timer2,Timer3. Timer0 is used for initializeHOPE() sequence
static void enableTimer(int timerNumber, uint32_t time){
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

	LPC_TIM_TypeDef *TIMx; // select Timer1, Timer2 or Timer3
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

/* ############################################################# */
/* ############## INITIALIZING SSP, I2C, GPIO ################## */
/* ############################################################# */

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
	// initialize SW4
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	// initialize SW3
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);

	// initialize rotary switch
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 24;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 25;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1<<24, 0);
	GPIO_SetDir(0, 1<<25, 0);

	//initialize joystick
	PinCfg.Pinnum = 17;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 15;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 16;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 4;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(0, 1<<17, 0); // center
	GPIO_SetDir(0, 1<<15, 0); // left
	GPIO_SetDir(0, 1<<16, 0); // right
	GPIO_SetDir(2, 1<<3, 0); // up
	GPIO_SetDir(2, 1<<4, 0); // down

	// init red and blue rgb to output
    GPIO_SetDir( 2, 1, 1 );
    GPIO_SetDir( 0, (1<<26), 1 );
}

void pinsel_uart3(void){
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity =UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	pinsel_uart3();

	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3,ENABLE);
}

/* ############################################################# */
/* ################## INITIALIZING INTERRUPTS ################## */
/* ############################################################# */

//Interupt handler flags
// resets and stops led countdown in survival mode until < LIGHTNING_MONITORING
static int STOP_LED_COUNTDOWN = 0;
static int SW3_FLAG = 0;
static int SAMPLING_FLAG = 0;
static int NEW_LIGHTNING_FLAG = 0;
static int UPDATE7SEG_FLAG = 0;
static int JOYSTICK_UP_FLAG = 0;
static int JOYSTICK_DOWN_FLAG = 0;
static int JOYSTICK_PRESS_FLAG = 0;
static int JOYSTICK_LEFT_FLAG = 0;
static int JOYSTICK_RIGHT_FLAG = 0;
static int ROTARY_RIGHT_FLAG = 0;
static int ROTARY_LEFT_FLAG = 0;

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
	// clear pending status before enabling
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_ClearPendingIRQ(TIMER3_IRQn);

	NVIC_EnableIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
}

void init_Interrupts(void){

    // Setup Interrupt for Light Sensor
    light_setRange(LIGHT_RANGE_4000); // sensing up to 3892 lux
    light_setIrqInCycles(LIGHT_CYCLE_1);
    light_clearIrqStatus();
    light_enable();

    // Determine initial light conditions
    uint32_t initial_light_value = light_read();
    if (initial_light_value > LIGHTNING_THRESHOLD) {
    	// Initialize interrupt to trigger on falling below threshold
    	aboveThreshold = 1;
    	light_setLoThreshold(LIGHTNING_THRESHOLD);
    	light_setHiThreshold(RANGE_K2-1); // disable high threshold
    } else {
        // Initialize interrupt to trigger on exceeding threshold
    	aboveThreshold = 0;
        light_setHiThreshold(LIGHTNING_THRESHOLD);
        light_setLoThreshold(0); // disable low threshold
    }

    // clear interrupts before enabling
    LPC_GPIOINT->IO2IntClr = 1 << 5;
    LPC_GPIOINT->IO2IntClr = 1 << 10;
    LPC_GPIOINT->IO0IntClr = 1 << 24;
    LPC_GPIOINT->IO0IntClr = 1 << 25;
    LPC_GPIOINT->IO0IntClr = 1 << 17;
    LPC_GPIOINT->IO0IntClr = 1 << 15;
    LPC_GPIOINT->IO0IntClr = 1 << 16;
    LPC_GPIOINT->IO2IntClr = 1 << 3;
    LPC_GPIOINT->IO2IntClr = 1 << 4;

    LPC_GPIOINT->IO2IntEnF |= 1 << 5; // light sensor 3000 lux interrupt (P2.5)
    LPC_GPIOINT->IO2IntEnF |= 1 << 10; // SW3 (P2.10)
    LPC_GPIOINT->IO0IntEnF |= 1 << 24; // rotary switch right
    LPC_GPIOINT->IO0IntEnF |= 1 << 25; // rotary switch left
    // joystick
    LPC_GPIOINT->IO0IntEnF |= 1 << 17; // center
    LPC_GPIOINT->IO0IntEnF |= 1 << 15; // down
    LPC_GPIOINT->IO0IntEnF |= 1 << 16; // right
    LPC_GPIOINT->IO2IntEnF |= 1 << 3; // up
    LPC_GPIOINT->IO2IntEnF |= 1 << 4; // left

	enableTimer(RGB, 1000); // RGB will blink throughout operation at 1000ms interval
	enableTimer(SAMPLING, 2000);
	enableTimer(PCA9532, 250);
}

void lightning_Interrupt_Handler(void){
	if (aboveThreshold) { // Interrupt indicates light reading went below threshold
		aboveThreshold = 0;
		// Sense for threshold exceed again
		light_setHiThreshold(LIGHTNING_THRESHOLD);
		light_setLoThreshold(0); // disable low threshold

		if (OPERATION_MODE == SURVIVAL_MODE) {
			STOP_LED_COUNTDOWN = 0; // continue with led countdown sequence
		}
		flashEnd = getMsTicks();
		if (flashEnd - flashBeginning < 500) {
			NEW_LIGHTNING_FLAG = 1; // push new value to recentFlashes[]
		}

	} else { // Interrupt indicates light reading exceeded threshold
		aboveThreshold = 1;
		// Sense for fall below threshold
		light_setLoThreshold(LIGHTNING_THRESHOLD);
		light_setHiThreshold(RANGE_K2-1); // disable high threshold

		flashBeginning = getMsTicks();
		if (OPERATION_MODE == SURVIVAL_MODE) {
			STOP_LED_COUNTDOWN = 1;
			ledOn = 0xffff; // reset countdown sequence
			pca9532_setLeds(ledOn, 0xffff);
		}
	}
}

void TIMER1_IRQHandler(void){ // PCA9532 Timer
	if (OPERATION_MODE == SURVIVAL_MODE && STOP_LED_COUNTDOWN == 0) {
		ledOn = ledOn >> 1; // turn off one led by right shifting bit pattern
		pca9532_setLeds(ledOn, 0xffff);
	}
	TIM_ClearIntPending(LPC_TIM1,0);
}

void TIMER2_IRQHandler(void){ // RGB timer
	rgbBlinky();
	TIM_ClearIntPending(LPC_TIM2,0);
}

void TIMER3_IRQHandler(void){ // Sampling Timer
	SAMPLING_FLAG = 1;
	TIM_ClearIntPending(LPC_TIM3,0);
}

void EINT3_IRQHandler(void){
	if ((LPC_GPIOINT->IO2IntStatF >> 5) & 0x1) { // Light Sensor Interrupt
		LPC_GPIOINT->IO2IntClr = (1<<5);
		light_clearIrqStatus();
		lightning_Interrupt_Handler();
	}

	if ((LPC_GPIOINT->IO2IntStatF >> 10) & 0x1) { // SW3 interrupt
		LPC_GPIOINT->IO2IntClr = (1<<10);
		SW3_FLAG = 1;
	}

	if ((LPC_GPIOINT->IO0IntStatF >> 24) & 0x3) { // either P0.24 or P0.25 triggered (rotary switch)
		LPC_GPIOINT->IO0IntClr = 0x3 << 24; // clear bits 24 and 25
		uint8_t rotaryState;
		rotaryState = rotary_read();
		if (rotaryState == ROTARY_RIGHT) {
			ROTARY_RIGHT_FLAG = 1;
			clockwise_rotation+=1;
		} else if (rotaryState == ROTARY_LEFT) {
			ROTARY_LEFT_FLAG = 1;
			clockwise_rotation-=1;
		}
	}

	//joystick up
	if((LPC_GPIOINT->IO2IntStatF >> 3) & 0x1){
		LPC_GPIOINT->IO2IntClr = 0x1 << 3;
		JOYSTICK_UP_FLAG = 1;
	}
	//joystick center
	if((LPC_GPIOINT->IO0IntStatF >> 17) & 0x1){
		LPC_GPIOINT->IO0IntClr = 0x1 << 17;
		JOYSTICK_PRESS_FLAG = 1;
	}
	//joystick down
	if((LPC_GPIOINT->IO0IntStatF >> 15) & 0x1){
		LPC_GPIOINT->IO0IntClr = 0x1 << 15;
		JOYSTICK_DOWN_FLAG = 1;
	}
	//joystick left
	if((LPC_GPIOINT->IO2IntStatF >> 4) & 0x1){
		LPC_GPIOINT->IO2IntClr = 0x1 << 4;
		JOYSTICK_LEFT_FLAG = 1;
	}
	//joystick right
	if((LPC_GPIOINT->IO0IntStatF >> 16) & 0x1){
		LPC_GPIOINT->IO0IntClr = 0x1 << 16;
		JOYSTICK_RIGHT_FLAG = 1;
	}
}

/* ############################################################# */
/* ############ HELPER FUNCTIONS FOR PERIPHERALS ############### */
/* ############################################################# */

static int32_t readTempSensor(void){
	int32_t temperature = temp_read();
	return temperature/10.0; // temperature in Celsius
}

static int32_t readLightSensor(void){
	int32_t light_value = light_read();
	return light_value; // light intensity in lux
}

static int32_t * readAccelerometer(void){
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

// function to print sensor readings to oled screen, in 3 different lines
void printValues(int32_t light_value, int32_t temp_value, int32_t *xyz_values){
	char lightTempArray[20];
	char xyzArray[20];
	sprintf(lightTempArray, "L%d_T%d", (int)light_value, (int)temp_value);
	sprintf(xyzArray, "AX%d_AY%d_AZ%d", (int)*(xyz_values), (int)*(xyz_values+1),(int)*(xyz_values+2));

	if(OPERATION_MODE == EXPLORER_MODE){
		oled_putString(0,0,(uint8_t*)lightTempArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,10,(uint8_t*)xyzArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	} else {
		oled_putString(0,10,(uint8_t*)lightTempArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,20,(uint8_t*)xyzArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}
}

void rgbBlinky (void){
	if (RGB_ON == 1) {
		RGB_ON = 0; // toggle off
		GPIO_ClearValue( 2, 1);
		GPIO_ClearValue( 0, (1<<26) );
	} else {
		RGB_ON = 1; // toggle on
		if (OPERATION_MODE == SURVIVAL_MODE) {
			GPIO_SetValue( 2, 1); // red
		}
		if (OPERATION_MODE == EXPLORER_MODE) {
			GPIO_SetValue( 0, (1<<26)); // blue
		}
	}
}

void resetHOPE(void){ // reset all global variables and peripherals to initial values

	OPERATION_MODE = EXPLORER_MODE;
	init_Interrupts();

	/* Accelerometer Variables */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;

	ledOn = 0xffff; // pca9532 led bit pattern
	RGB_ON = 0;

	static uint32_t tempArray[9] = {0,0,0,0,0,0,0,0,0};
	memcpy(recentFlashes, tempArray, recentFlashesSize);
	recentFlashesStackPointer = -1;

	flashBeginning = 0;
	flashEnd = 0;
	flashesToEnterSurvival = 3;
	clockwise_rotation = 0;
	LIGHTNING_TIME_WINDOW = 3000;

	/* RESET FLAGS */
	STOP_LED_COUNTDOWN = 0;
	SW3_FLAG = 0;
	SAMPLING_FLAG = 0;
	NEW_LIGHTNING_FLAG = 0;
	UPDATE7SEG_FLAG = 0;
	JOYSTICK_UP_FLAG = 0;
	JOYSTICK_DOWN_FLAG = 0;
	JOYSTICK_PRESS_FLAG = 0;
	JOYSTICK_LEFT_FLAG = 0;
	JOYSTICK_RIGHT_FLAG = 0;
	ROTARY_RIGHT_FLAG = 0;
	ROTARY_LEFT_FLAG = 0;
	SEGMENT_DISPLAY = '0';

	/* RESET PERIPHERALS */
	pca9532_setLeds(0x0000, 0xffff);
	led7seg_setChar('\0', FALSE);
	oled_clearScreen(OLED_COLOR_BLACK);
	explorerMainDisplayInit();
	explorerMainDisplaySelection = AVRG_READING_SELECTED;
	diagnostic_runs_count = 0;
	explorer_diagnostic_display_page = 1;
	light_setRange(LIGHT_RANGE_4000);
}

/* ############################################################# */
/* ################ DEFINING OLED DISPLAY GUI ################## */
/* ############################################################# */

void introDisplay(int hopePosition){

	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(0,0,"====H.O.P.E.====",OLED_COLOR_BLACK,OLED_COLOR_WHITE);
	oled_putString(0,35,"      MARS    ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_circle(45, 35, 15, OLED_COLOR_WHITE);
	// animation causing square to orbit around circle
	switch(hopePosition){
	case 0:
		oled_rect(40,10,50,20,OLED_COLOR_WHITE);
		break;
	case 1:
		oled_rect(23,20,33,30,OLED_COLOR_WHITE);
		break;
	case 2:
		oled_rect(23,38,33,48,OLED_COLOR_WHITE);
		break;
	case 3:
		oled_rect(33,50,43,60,OLED_COLOR_WHITE);
		break;
	case 4:
		oled_rect(47,50,57,60,OLED_COLOR_WHITE);
		break;
	case 5:
		oled_rect(57,38,67,48,OLED_COLOR_WHITE);
		break;
	case 6:
		oled_rect(57,20,67,30,OLED_COLOR_WHITE);
		break;
	default:
		oled_rect(40,10,50,20,OLED_COLOR_WHITE);
		break;
	}
}

void explorerMainDisplayInit(void){
	JOYSTICK_UP_FLAG = 0;
	JOYSTICK_DOWN_FLAG = 0;
	JOYSTICK_PRESS_FLAG = 0;
	JOYSTICK_LEFT_FLAG = 0;
	JOYSTICK_RIGHT_FLAG = 0;
	ROTARY_RIGHT_FLAG = 0;
	ROTARY_LEFT_FLAG = 0;

	explorerDisplayClearScreen();
	char array[20];
	sprintf(array, "F to SVR: %d", (int)flashesToEnterSurvival);
	oled_putString(0,20,array,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,30,"> AVRG Reading",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,"  Control Flashes",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,50,"  Time to SVR",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	explorerScreen = MAIN_SCREEN;
}

//function to refresh sensor values on main display in explorer mode
void explorerMainDisplayRefresh(int32_t light_value, int32_t temp_value, int32_t *xyz_values){
	char lightTempArray[20];
	char xyzArray[20];
	sprintf(lightTempArray, "L%d_T%d", (int)light_value, (int)temp_value);
	sprintf(xyzArray, "AX%d_AY%d_AZ%d", (int)*(xyz_values), (int)*(xyz_values+1),(int)*(xyz_values+2));

	oled_putString(0,0,(uint8_t*)lightTempArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,10,(uint8_t*)xyzArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

void explorerDisplayClearScreen(void){
	oled_putString(0,20,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,30,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,50,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

void explorerDiagnosticDisplay(int page){
	char avrgArray[20];
	char maxArray[20];
	char minArray[20];
	if(page==1){
		sprintf(avrgArray, "AVG-L%d", (int)avrg_light_value);
		sprintf(maxArray, "MAX-L%d", (int)highest_light_value);
		sprintf(minArray, "MIN-L%d", (int)lowest_light_value);
		explorerDisplayClearScreen();
		oled_putString(0,20,"Light Readings",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,30,(uint8_t*)avrgArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,40,(uint8_t*)maxArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,50,(uint8_t*)minArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}else if(page==2){
		sprintf(avrgArray, "AVG-T%d", (int)avrg_temp_value);
		sprintf(maxArray, "MAX-T%d", (int)highest_temp_value);
		sprintf(minArray, "MIN-T%d", (int)lowest_temp_value);
		explorerDisplayClearScreen();
		oled_putString(0,20,"Temp Readings",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,30,(uint8_t*)avrgArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,40,(uint8_t*)maxArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,50,(uint8_t*)minArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}else if(page==3){
		sprintf(avrgArray, "AVG/X%dY%d/Z%d", (int)*(avrg_xyz_values),
				(int)*(avrg_xyz_values+1),(int)*(avrg_xyz_values+2));
		sprintf(maxArray, "MAX/X%dY%dZ%d", (int)*(highest_xyz_values),
				(int)*(highest_xyz_values+1),(int)*(highest_xyz_values+2));
		sprintf(minArray, "MIN/X%dY%dZ%d", (int)*(lowest_xyz_values),
				(int)*(lowest_xyz_values+1),(int)*(lowest_xyz_values+2));
		explorerDisplayClearScreen();
		oled_putString(0,20,"XYZ Readings",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,30,(uint8_t*)avrgArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,40,(uint8_t*)maxArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		oled_putString(0,50,(uint8_t*)minArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	}
}

void explorerControlFlashesDisplay(void){
	explorerDisplayClearScreen();
	char array[20];
	sprintf(array, "mode to: %d", (int)flashesToEnterSurvivalDisplay);
	oled_putString(0,30,"Change #flashes",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,"enter survival",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,50,(uint8_t*)array,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

void explorerTimeToSurvivalDisplay(void){
	explorerDisplayClearScreen();
	char array[20];
	sprintf(array, "%d ms", (int)USER_LIGHTNING_TIME_WINDOW);
	oled_putString(0,30,"Change time to",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,"survival mode:",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,50,(uint8_t*)array,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

void explorerMainDisplayControl (void){

	//joystick up and down only used in main screen and flash control screen
	if(explorerScreen != MAIN_SCREEN && explorerScreen != CONTROL_FLASHES_SCREEN){
		JOYSTICK_DOWN_FLAG = 0;
		JOYSTICK_UP_FLAG = 0;
	}

	if(explorerScreen == MAIN_SCREEN){

		if(JOYSTICK_DOWN_FLAG==1){
			JOYSTICK_DOWN_FLAG=0;
			if(explorerMainDisplaySelection==AVRG_READING_SELECTED){
				explorerMainDisplaySelection=CONTROL_FLASHES_SELECTED;
			}else if(explorerMainDisplaySelection==CONTROL_FLASHES_SELECTED){
				explorerMainDisplaySelection=TIME_TO_SURVIVAL_SELECTED;
			}else if(explorerMainDisplaySelection==TIME_TO_SURVIVAL_SELECTED){
				explorerMainDisplaySelection=AVRG_READING_SELECTED;
			}
		}

		if(JOYSTICK_UP_FLAG==1){
			JOYSTICK_UP_FLAG=0;
			if(explorerMainDisplaySelection==AVRG_READING_SELECTED){
				explorerMainDisplaySelection=TIME_TO_SURVIVAL_SELECTED;
			}else if(explorerMainDisplaySelection==CONTROL_FLASHES_SELECTED){
				explorerMainDisplaySelection=AVRG_READING_SELECTED;
			}else if(explorerMainDisplaySelection==TIME_TO_SURVIVAL_SELECTED){
				explorerMainDisplaySelection=CONTROL_FLASHES_SELECTED;
			}
		}

		switch(explorerMainDisplaySelection){

		case AVRG_READING_SELECTED:
			oled_putString(0,30,"> AVRG Reading",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,40,"  Control Flashes",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,50,"  Time to SVR",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			break;
		case CONTROL_FLASHES_SELECTED:
			oled_putString(0,30,"  AVRG Reading",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,40,"> Control Flashes",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,50,"  Time to SVR",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			break;
		case TIME_TO_SURVIVAL_SELECTED:
			oled_putString(0,30,"  AVRG Reading",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,40,"  Control Flashes",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,50,"> Time to SVR",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			break;
		default:
			oled_putString(0,30,"> AVRG Reading",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,40,"  Control Flashes",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			oled_putString(0,50,"  Time to SVR",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
			break;
		}
	}

	// use joystick up and down to adjust number of flashes required to enter survival mode
	else if (explorerScreen == CONTROL_FLASHES_SCREEN){
		if(JOYSTICK_DOWN_FLAG==1){
			JOYSTICK_DOWN_FLAG=0;
			explorerDisplayClearScreen();
			if(flashesToEnterSurvivalDisplay>1)flashesToEnterSurvivalDisplay--;
			explorerControlFlashesDisplay();
		}else if(JOYSTICK_UP_FLAG==1){
			JOYSTICK_UP_FLAG=0;
			explorerDisplayClearScreen();
			if(flashesToEnterSurvivalDisplay<9)flashesToEnterSurvivalDisplay++;
			explorerControlFlashesDisplay();
		}
	}

	// joystick press or right confirms selection
	if(JOYSTICK_PRESS_FLAG==1 || JOYSTICK_RIGHT_FLAG==1){
		JOYSTICK_PRESS_FLAG=0;
		JOYSTICK_RIGHT_FLAG=0;

		switch(explorerScreen){

		case MAIN_SCREEN:

			//select average reading on main screen
			if(explorerMainDisplaySelection==AVRG_READING_SELECTED){
				explorerScreen=AVRG_READING_SCREEN;
				explorerDiagnosticDisplay(explorer_diagnostic_display_page);
				explorer_diagnostic_display_page++;

			//select control flashes display
			}else if(explorerMainDisplaySelection==CONTROL_FLASHES_SELECTED){
				explorerScreen=CONTROL_FLASHES_SCREEN;
				flashesToEnterSurvivalDisplay=flashesToEnterSurvival;
				explorerControlFlashesDisplay();

			}else if(explorerMainDisplaySelection==TIME_TO_SURVIVAL_SELECTED){
				explorerScreen=TIME_TO_SURVIVAL_SCREEN;
				USER_LIGHTNING_TIME_WINDOW=LIGHTNING_TIME_WINDOW;
				explorerTimeToSurvivalDisplay();
			}
			break;

		//loop through arvg reading screen till end, return to main screen
		case AVRG_READING_SCREEN:
			if(explorer_diagnostic_display_page>3){
				explorer_diagnostic_display_page=1;
				explorerMainDisplaySelection=AVRG_READING_SELECTED;
				explorerScreen = MAIN_SCREEN;
				explorerDisplayClearScreen();
				explorerMainDisplayInit();
			}else{
				explorerDiagnosticDisplay(explorer_diagnostic_display_page);
				explorer_diagnostic_display_page++;
			}
			break;

		//go back to main screen on click
		case CONTROL_FLASHES_SCREEN:
			explorerMainDisplaySelection=CONTROL_FLASHES_SELECTED;
			explorerScreen = MAIN_SCREEN;
			flashesToEnterSurvival = flashesToEnterSurvivalDisplay;
			explorerDisplayClearScreen();
			explorerMainDisplayInit();
			break;

		case TIME_TO_SURVIVAL_SCREEN:
			explorerMainDisplaySelection=TIME_TO_SURVIVAL_SELECTED;
			explorerScreen = MAIN_SCREEN;
			LIGHTNING_TIME_WINDOW = USER_LIGHTNING_TIME_WINDOW;
			explorerDisplayClearScreen();
			explorerMainDisplayInit();
			break;
		}
	}

	// joystick left cancels current action and returns to MAIN_SCREEN
	if(JOYSTICK_LEFT_FLAG == 1){
		JOYSTICK_LEFT_FLAG = 0;

		switch(explorerScreen){

		case MAIN_SCREEN: // do nothing
			break;

		case AVRG_READING_SCREEN:
			explorer_diagnostic_display_page=1;
			explorerMainDisplaySelection=AVRG_READING_SELECTED;
			explorerScreen = MAIN_SCREEN;
			explorerMainDisplayInit();
			break;

		case CONTROL_FLASHES_SCREEN:
			explorerMainDisplaySelection=CONTROL_FLASHES_SELECTED;
			explorerScreen = MAIN_SCREEN;
			explorerMainDisplayInit();
			break;

		case TIME_TO_SURVIVAL_SCREEN:
			explorerMainDisplaySelection=TIME_TO_SURVIVAL_SELECTED;
			explorerScreen = MAIN_SCREEN;
			explorerMainDisplayInit();
		}
	}

	// use rotary right and left to adjust time window for lightning flashes
	if(ROTARY_RIGHT_FLAG==1 || ROTARY_LEFT_FLAG==1){
		ROTARY_RIGHT_FLAG = 0;
		ROTARY_LEFT_FLAG = 0;
		if(explorerScreen==TIME_TO_SURVIVAL_SCREEN){
			USER_LIGHTNING_TIME_WINDOW = USER_LIGHTNING_TIME_WINDOW + clockwise_rotation*10;
			clockwise_rotation = 0;
			explorerTimeToSurvivalDisplay();
		}
	}
}

void survivalDisplay(void){
	char lightTempArray[20];
	char xyzArray[20];
	sprintf(lightTempArray, "LS_TS");
	sprintf(xyzArray, "AXS_AYS_AZS");
	oled_putString(0,0,"====H.O.P.E.====",OLED_COLOR_BLACK,OLED_COLOR_WHITE);
	oled_putString(0,10,(uint8_t*)lightTempArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,20,(uint8_t*)xyzArray,OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,30,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,40,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0,50,"                  ",OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

/* ############################################################# */
/* #################### INITIALIZING HOPE ###################### */
/* ############################################################# */

static void initializeHOPE(void){

	int oled_segment_display =0;
	SEGMENT_DISPLAY = '0';

	while (SEGMENT_DISPLAY != 'G') { // increment until segment displays 'F'
		led7seg_setChar(SEGMENT_DISPLAY, FALSE); // display new incremented value

		Timer0_Wait(1000); // 1s interval
		if (SEGMENT_DISPLAY == '6') {
			SEGMENT_DISPLAY = 'A'; // change from number to alphabet
		} else {
			SEGMENT_DISPLAY += 1; // increment
		}
		if (SEGMENT_DISPLAY >= 'A') {
			oled_segment_display++;
			introDisplay(oled_segment_display);
		} else {
			oled_clearScreen(OLED_COLOR_BLACK);
		}
	}
	SEGMENT_DISPLAY = '\0';
	led7seg_setChar(SEGMENT_DISPLAY, FALSE); // clear 7 segment display
	oled_clearScreen(OLED_COLOR_BLACK); //clear OLED screen

}

/* ############################################################# */
/* ################ SENSOR READINGS DIAGNOSTICS ################ */
/* ############################################################# */

void calcNewAvrgVal(int32_t *current_average, int32_t new_value, int diagnostic_runs_count){
	*current_average = ((*current_average)*diagnostic_runs_count + new_value)/
			(diagnostic_runs_count + 1);
}

void calcNewHighestVal(int32_t *current_highest, int32_t new_value){
	if (*current_highest < new_value){
		*current_highest = new_value;
	}
}

void calcNewLowestVal(int32_t *current_lowest, int32_t new_value){
	if (*current_lowest > new_value) {
		*current_lowest = new_value;
	}
}

void explorerDiagnosticLogic(int32_t light_value, int32_t temp_value, int32_t *xyz_values){
	//calculate all-time average values for this run
	if(diagnostic_runs_count==0){
		avrg_light_value = light_value;
		highest_light_value = light_value;
		lowest_light_value = light_value;
		avrg_temp_value = temp_value;
		highest_temp_value = temp_value;
		lowest_temp_value = temp_value;
		*avrg_xyz_values = *xyz_values;
		*highest_xyz_values = *xyz_values;
		*lowest_xyz_values = *xyz_values;
		*(avrg_xyz_values+1) = *(xyz_values+1);
		*(highest_xyz_values+1) = *(xyz_values+1);
		*(lowest_xyz_values+1) = *(xyz_values+1);
		*(avrg_xyz_values+2) = *(xyz_values+2);
		*(highest_xyz_values+1) = *(xyz_values+1);
		*(lowest_xyz_values+1) = *(xyz_values+1);
	}else{
		calcNewAvrgVal(&avrg_light_value,light_value,diagnostic_runs_count);
		calcNewHighestVal(&highest_light_value,light_value);
		calcNewLowestVal(&lowest_light_value,light_value);
		calcNewAvrgVal(&avrg_temp_value,temp_value,diagnostic_runs_count);
		calcNewHighestVal(&highest_temp_value,temp_value);
		calcNewLowestVal(&lowest_temp_value,temp_value);
		// x values
		calcNewAvrgVal(avrg_xyz_values,*xyz_values,diagnostic_runs_count);
		calcNewHighestVal(highest_xyz_values,*xyz_values);
		calcNewLowestVal(lowest_xyz_values,*xyz_values);
		// y values
		calcNewAvrgVal(avrg_xyz_values+1,*(xyz_values+1),diagnostic_runs_count);
		calcNewHighestVal(highest_xyz_values+1,*(xyz_values+1));
		calcNewLowestVal(lowest_xyz_values+1,*(xyz_values+1));
		// z values
		calcNewAvrgVal(avrg_xyz_values+2,*(xyz_values+2),diagnostic_runs_count);
		calcNewHighestVal(highest_xyz_values+2,*(xyz_values+2));
		calcNewLowestVal(lowest_xyz_values+2,*(xyz_values+2));
	}
	diagnostic_runs_count++;
}

/* ############################################################# */
/* ##################### TASK HANDLERS ######################### */
/* ############################################################# */

static void explorerTasks(void){

	if (SAMPLING_FLAG == 1) {
		SAMPLING_FLAG = 0;
		int32_t light_value = readLightSensor();
		int32_t temp_value = readTempSensor();
		int32_t *xyz_values;
		xyz_values = readAccelerometer();
		explorerMainDisplayRefresh(light_value, temp_value, xyz_values);
		char msg[50];
		snprintf(msg,sizeof(msg),"L%d_T%d_AX%d_AY%d_AZ%d\r\n",(int)light_value,(int)temp_value,
				(int)*(xyz_values),(int)*(xyz_values+1),(int)*(xyz_values+2));
		UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);
		explorerDiagnosticLogic(light_value, temp_value, xyz_values);
	}
	explorerMainDisplayControl();

	// CONDITION FOR SWITCHING TO SURVIVAL MODE
	if (recentFlashesStackPointer >= flashesToEnterSurvival-1) {
		OPERATION_MODE = SURVIVAL_MODE;
		oled_clearScreen(OLED_COLOR_BLACK);
		survivalDisplay();
    	UART_Send(LPC_UART3, (uint8_t *)ENTER_SURVIVAL_MESSAGE , strlen(ENTER_SURVIVAL_MESSAGE), BLOCKING);
	}
}

static void survivalTasks(void){

	if (ledOn == 0) { // no lightning in previous 4s, safe to switch to EXPLORER_MODE
		OPERATION_MODE = EXPLORER_MODE;
		oled_clearScreen(OLED_COLOR_BLACK);
		explorerMainDisplayInit(); //init explorer display
    	UART_Send(LPC_UART3, (uint8_t *)EXIT_SURVIVAL_MESSAGE , strlen(EXIT_SURVIVAL_MESSAGE), BLOCKING);
	}
}

static void genericTasks(void){

	//check if 7 segment count of lightning flashes is up-to-date
	CURRENT_TIME = getMsTicks();
	if ((CURRENT_TIME > recentFlashes[recentFlashesStackPointer] + LIGHTNING_TIME_WINDOW) &&
			recentFlashesStackPointer >= 0){ // stack pointer pointing to a valid value that has expired
		recentFlashesStackPointer--;
		if (SEGMENT_DISPLAY != '1') { // minimum display '1'
			SEGMENT_DISPLAY -= 1;
		} else {
			SEGMENT_DISPLAY = '\0'; // turn off display
		}
		UPDATE7SEG_FLAG = 1;
	}

	if (NEW_LIGHTNING_FLAG == 1) {
		NEW_LIGHTNING_FLAG = 0;
		// Push new value of flashBeginning into recentFlashes
		int i=7;
		while(i>=0){
			recentFlashes[i+1]=recentFlashes[i]; //shift values to the right
			i--;
		}
		recentFlashes[0] = flashBeginning; // push new value into first element of array
		if (recentFlashesStackPointer != recentFlashesSize-1) {
			recentFlashesStackPointer++; // increment only if pointer does not point outside the array
		}

		if (SEGMENT_DISPLAY == '\0') { // turn off 7 segment display
			SEGMENT_DISPLAY = '1';
			UPDATE7SEG_FLAG = 1;
		} else if (SEGMENT_DISPLAY != '9') { // limit 7 segment display to '9'
			SEGMENT_DISPLAY += 1;
			UPDATE7SEG_FLAG = 1;
		}
	}

	if (UPDATE7SEG_FLAG == 1) {
		UPDATE7SEG_FLAG = 0;
		led7seg_setChar(SEGMENT_DISPLAY, FALSE);
	}

	if (SW3_FLAG == 1) {
		SW3_FLAG = 0;
		int32_t light_value = readLightSensor();
		int32_t temp_value = readTempSensor();
		int32_t *xyz_values;
		xyz_values = readAccelerometer();
		printValues(light_value, temp_value, xyz_values); // print current sensor readings
		char msg[50];
		snprintf(msg,sizeof(msg),"L%d_T%d_AX%d_AY%d_AZ%d\r\n",(int)light_value,(int)temp_value,
				(int)*(xyz_values),(int)*(xyz_values+1),(int)*(xyz_values+2));
		UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING); // send sensor readings to HOME
	}
}

/* ############################################################# */
/* #################### MAIN FUNCTION ########################## */
/* ############################################################# */

int main(void){

    init_i2c();
    init_ssp();
    init_GPIO();
    init_uart();
    init_Priority();

    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
	led7seg_init();
    temp_init(&getMsTicks);
    rotary_init();

    if(SysTick_Config(SystemCoreClock/1000)){
    	while(1); //capture error
    }

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;

    // Initialize OLED
    oled_clearScreen(OLED_COLOR_BLACK);
	initializeHOPE();

    //variable to control detection of flashes to enter survivor
    flashesToEnterSurvivalDisplay = flashesToEnterSurvival;
    USER_LIGHTNING_TIME_WINDOW = LIGHTNING_TIME_WINDOW;
    init_Interrupts();
    explorerMainDisplayInit();
    light_setRange(LIGHT_RANGE_4000);

    uint8_t resetButtonSW4 = 1;

    while (1)
    {
    	resetButtonSW4 = (GPIO_ReadValue(1) >> 31) & 0x01; // SW4 polling mode
    	if (resetButtonSW4 == 0) {
    		resetHOPE();
    	}

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
