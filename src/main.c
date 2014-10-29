#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

/*
 * Include Systick Functions
 */

//#include "LPC17xx.h"
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "rotary.h"
#include "eeprom.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "uart2.h"
#include "light.h"
#include "temp.h"

/*
 * Program Constants
 */


#define ACCELEROMETER_BUFFER 10

const uint8_t ACC_ST_BRIGHT = 50;
const uint8_t ACC_ST_DIM_RELAY = 100;
const uint32_t LS_TS_ST_BRIGHT = 1000;
const uint32_t LS_TS_ST_DIM_RELAY = 3000;

const uint32_t reportingTime = 5;
const uint32_t distressTime = 1;

const uint32_t LIGHT_THRESHOLD = 800;
const uint32_t TEMP_WARN = 260;

const uint32_t note = 1703;

const char *WELCOME_MESSAGE = "BOOTING\n";
const char *DEBUG_SYSTICK = "SYSTICK INIT ERROR\n";

/*
 * Program Variables
 */

static uint32_t temperature = 0;
static uint32_t lightIntensity = 0;
static uint32_t msTicks = 0;
volatile static uint8_t ALARM_OFF = 0;
static uint8_t temperatureLock = 0;
static uint32_t ch = 48;
static uint32_t ledtime = 1000;

//typedef and structures
typedef enum {BRIGHT, DIM, RELAY} timingState_t;

struct timingState {
	timingState_t state;
	uint8_t accelerationTime;
	uint32_t tempLightTime;
};

static struct timingState timeState;

/*
 * Circular Buffer
 */

struct accelerationCB {
	int pointer;
	int8_t x[ACCELEROMETER_BUFFER];
	int8_t y[ACCELEROMETER_BUFFER];
	int8_t z[ACCELEROMETER_BUFFER];
	int8_t xoff;
	int8_t yoff;
	int8_t zoff;
	int xvar;
	int yvar;
	int zvar;
};

static struct accelerationCB accCB;

/**
 * Initializes Accelerometer and variance calculations
 */

void initAccCB(void){
	accCB.pointer = 0;
	acc_read(&accCB.xoff, &accCB.yoff, &accCB.zoff);
	accCB.xoff = 0 - accCB.xoff;
	accCB.yoff = 0 - accCB.yoff;
	accCB.zoff = 0 - accCB.zoff;
	accCB.xvar = 0;
	accCB.yvar = 0;
	accCB.zvar = 0;
}

void nextElements(void){
	accCB.pointer = (accCB.pointer + 1) % ACCELEROMETER_BUFFER;
}

void accelerationHandler(void){
	nextElements();
	acc_read(&accCB.x[accCB.pointer], &accCB.y[accCB.pointer], &accCB.z[accCB.pointer]);
}

int calcAccVar(void){

	int sumz = 0;
	int meanz = 0;
	int sumSQz = 0;
	int index;

	for (index = 0; index < ACCELEROMETER_BUFFER; index++){
		accCB.z[index] = accCB.z[index] - accCB.zoff;
		sumz += accCB.z[index];
	}

	meanz = sumz / ACCELEROMETER_BUFFER;

	for (index = 0; index < ACCELEROMETER_BUFFER; index++){
		sumSQz += (accCB.z[index] - meanz)*(accCB.z[index] - meanz);
	}

	accCB.zvar = sumSQz/ACCELEROMETER_BUFFER;

	return accCB.zvar;
}

void checkTempLight(void){
	temperatureLock = 1;
	temperature = temp_read();
	temperatureLock = 0;
	lightIntensity = light_read();

	if (timeState.state == RELAY){
		return;
	}

	if (lightIntensity < LIGHT_THRESHOLD){
		timeState.state = DIM;
		timeState.accelerationTime = ACC_ST_DIM_RELAY;
		timeState.tempLightTime = LS_TS_ST_DIM_RELAY;
	} else {
		timeState.state = BRIGHT;
		timeState.accelerationTime = ACC_ST_BRIGHT;
		timeState.tempLightTime = LS_TS_ST_BRIGHT;
	}

	printf("Temp: %d | Light: %d\n", temperature, lightIntensity);
	printf("AccTime : %d | TempLightTime : %d\n", timeState.accelerationTime, timeState.tempLightTime);
}

static void led7seg_update(void){
	if (ch < 57)
	ch++;
			else
				ch = 48;
			led7seg_setChar(ch, 0);
}

void SysTick_Handler(void) {
	/*
	 * Parallel control loop
	 *
	 * Sample accelerometer
	 *
	 */
	msTicks++;
	if (msTicks % timeState.accelerationTime == 0 && temperatureLock == 0){
		accelerationHandler();
	}
	if (msTicks % ledtime == 0){
		led7seg_update();
	}

}

void initSystick() {
	/**
	 * Configure Systick
	 */

	if (SysTick_Config(SystemCoreClock / 1000)) {
		printf(DEBUG_SYSTICK);
		while (1)
			;

	}
}

uint32_t getTicks(void){
    return msTicks;
}

static void sysTick_delay(uint32_t delayTicks){
uint32_t currentTicks;
currentTicks = msTicks;
while ((msTicks - currentTicks) < delayTicks);
}

/*
 * Timing States
 */



void initTimingState(void){
	timeState.state = BRIGHT;
	timeState.accelerationTime = ACC_ST_BRIGHT;
	timeState.tempLightTime = LS_TS_ST_BRIGHT;
}

/**
 * Synchronous Serial Port (SSP) is a controller that supports the Serial Peripheral Interface (SPI), 4-wire Synchronous Serial Interface (SSI), and Microwire serial buses
 */

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

/**
 * Inits I2C
 */

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
	/*
	 * Initialize button/s
	 *
	 * SW3 TOGGLE
	 * SW4 ALARM OFF
	 * BUZZER - ALARM
	 *
	 */

	PINSEL_CFG_Type PinCfg;
	//initialise SW3
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1 << 10, 0);

	//initialise SW4
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 31, 0);
	GPIO_ClearValue(1, 1 << 31);

}

void buzzer_init(){
    GPIO_SetDir(2, 1<<0, 1);
    GPIO_SetDir(2, 1<<1, 1);

    GPIO_SetDir(0, 1<<27, 1);
    GPIO_SetDir(0, 1<<28, 1);
    GPIO_SetDir(2, 1<<13, 1);
    GPIO_SetDir(0, 1<<26, 1);

    GPIO_ClearValue(0, 1<<27);
    GPIO_ClearValue(0, 1<<28);
    GPIO_ClearValue(2, 1<<13);
}

void activateBuzzer(void) {

	ALARM_OFF = 0;
	led7seg_setChar('1',0);
	while (!ALARM_OFF) {

		GPIO_SetValue(0, 1<<26);
		Timer0_us_Wait(note / 2);

		GPIO_ClearValue(0, 1<<26);
		Timer0_us_Wait(note / 2);

	}
	led7seg_setChar('2',0);
}

void enableSW3int(void){

	PINSEL_CFG_Type PinCfg;

	GPIO_SetDir(2,1<<10,0);

	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PINSEL_ConfigPin(&PinCfg);

	LPC_GPIOINT->IO2IntEnF |= 1<<10;

	NVIC_EnableIRQ(EINT0_IRQn);

	//LPC_SC->EXTINT = 1;
	//NVIC_ClearPendingIRQ(EINT3_IRQn);
	//NVIC_SetPriorityGrouping(4);
	//NVIC_SetPriority(EINT3_IRQn, NVIC_EncodePriority(4,2,0));
	NVIC_EnableIRQ(EINT3_IRQn);


}

void EINT0_IRQHandler(void){
	LPC_SC->EXTINT = 1;				//Clear EINT0 interrupt
	LPC_GPIOINT->IO2IntClr = 1 << 10;
	NVIC_ClearPendingIRQ(EINT0_IRQn);
}


int main (void) {

	/**
	 * Initialisation Steps
	 */
	__disable_irq();
	printf(WELCOME_MESSAGE);

	init_i2c();
    init_ssp();
    init_GPIO();

    led7seg_init();
    led7seg_setChar(' ',0);
    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
    /*
     * Red LED of RGB light due to conflict with OLED
     */
    GPIO_SetDir(2,1,1);
    temp_init(&getTicks);
    eeprom_init();
    light_enable();
    buzzer_init();
    initAccCB();
    initSystick();

    enableSW3int();
    __enable_irq();

    uint8_t btn1 = 1;

    while(TRUE){
    checkTempLight();
   btn1 = (GPIO_ReadValue(1)) >> 31 & 0x01;
   if (btn1 == 0){
    	ALARM_OFF = 1 ;
    }
    if (temperature >= TEMP_WARN && ALARM_OFF == 0){
    	activateBuzzer();
    }

    	/*
    	 * Acc test ? Passed
    	 */


    	//printf("X: %d | Y: %d | Z: %d | pointer: %d\n", accCB.x[accCB.pointer], accCB.y[accCB.pointer], accCB.z[accCB.pointer], accCB.pointer);
    	//calcAccVar();
    	//printf("Xvar: %d | Yvar: %d | Zvar: %d\n", accCB.xvar, accCB.yvar, accCB.zvar);
    	//calcAccVar();
    	//accelerationHandler();
    	//printf("Zvar: %d | Z: %d | pointer %d\n", accCB.zvar, accCB.z[accCB.pointer], accCB.pointer);


    	//activateBuzzer();
    }


}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
