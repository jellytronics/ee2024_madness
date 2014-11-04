/*
 *
 * EE2024 Project for kicks
 * @author Edward
 * @author Jeremias
 *
 * Parts of code adapted from source code obtained
 * from http://www.youtube.com/watch?v=BmRyRGZSRng
 *
 */


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
#define CHAR_ARRAY_BUFFER 100
#define MESSAGE_WIDTH 100

const uint8_t ACC_ST_BRIGHT = 50;
const uint8_t ACC_ST_DIM_RELAY = 100;
const uint32_t LS_TS_ST_BRIGHT = 1000;
const uint32_t LS_TS_ST_DIM_RELAY = 3000;

const uint32_t reportingTime = 5;
const uint32_t distressTime = 1;
const uint32_t RELAY_RGB_TIME = 2000;
const uint32_t OLED_UPDATE = 500;
const uint16_t allLedsOn = 0xff00;
const uint16_t allLedsOff = 0x0;

const uint32_t LIGHT_THRESHOLD = 800;
const uint32_t TEMP_THRESHOLD = 330; //260

const uint32_t note = 1703;

const char *WELCOME_MESSAGE = "BOOTING\n";
const char *DEBUG_SYSTICK = "SYSTICK INIT ERROR\n";
const char *NODE_ID = "N001";

/*
 * Program Variables
 */

static uint32_t msTicks = 0;
static uint8_t buzzerState = 0;
static uint8_t i2c0Lock = 0;
static uint8_t spiLock = 0;
static uint32_t rgbState = 0;

static uint32_t lcdNumber = 0;
static uint32_t lcdTime = 1000;
static uint32_t ASCII_Number = 48;

static char tempString[20];
static char lightString[20];
static char varString[20];
static char stateString[20];

/*
 * Circular Buffer
 */

struct dataCB {
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
	uint32_t lightIntensity;
	uint32_t temperature;
};

static struct dataCB dataBuffer;

/**
 * Initializes Accelerometer and variance calculations
 */

void initAccCB(void){
	dataBuffer.pointer = 0;
	acc_read(&dataBuffer.xoff, &dataBuffer.yoff, &dataBuffer.zoff);
	dataBuffer.xvar = 0;
	dataBuffer.yvar = 0;
	dataBuffer.zvar = 0;
	dataBuffer.lightIntensity = 0;
	dataBuffer.temperature = 0;
}

/*
 * Timing States
 */

typedef enum {BRIGHT, DIM, RELAY} timingState_t;

struct timingState {
	timingState_t state;
	uint8_t accelerationTime;
	uint32_t tempLightTime;
	uint32_t lastAccTime;
	uint32_t lastTempLightTime;
	uint32_t lastLCDTime;
	uint32_t lastRGBTime;
	uint32_t lastOLEDTime;
};

static struct timingState timeState;

void initTimingState(void){
	timeState.state = BRIGHT;
	timeState.accelerationTime = ACC_ST_BRIGHT;
	timeState.tempLightTime = LS_TS_ST_BRIGHT;
	timeState.lastAccTime = 0;
	timeState.lastTempLightTime = 0;
	timeState.lastLCDTime = 0;
	timeState.lastRGBTime = 0;
	timeState.lastOLEDTime = 0;
}

/*
 * implement check here
 */

static void led7seg_update(void){
	lcdNumber = (lcdNumber + 1) % 10;
	led7seg_setChar(lcdNumber + ASCII_Number, 0);
}


void nextElements(void){
	dataBuffer.pointer = (dataBuffer.pointer + 1) % ACCELEROMETER_BUFFER;
}

/**
 * Accelerometer Handler
 */

void accelerationHandler(void){
	nextElements();
	acc_read(&dataBuffer.x[dataBuffer.pointer], &dataBuffer.y[dataBuffer.pointer], &dataBuffer.z[dataBuffer.pointer]);
}

int calcAccVarZ(void){

	int sumz = 0;
	int meanz = 0;
	int sumSQz = 0;
	int index;

	for (index = 0; index < ACCELEROMETER_BUFFER; index++){
		//dataBuffer.z[index] = dataBuffer.z[index] - dataBuffer.zoff;
		sumz += dataBuffer.z[index];
	}

	meanz = sumz / ACCELEROMETER_BUFFER;

	for (index = 0; index < ACCELEROMETER_BUFFER; index++){
		sumSQz += (dataBuffer.z[index] - meanz)*(dataBuffer.z[index] - meanz);
	}

	dataBuffer.zvar = sumSQz/ACCELEROMETER_BUFFER;

	return dataBuffer.zvar;
}

void toggleRGBLed(void) {
	if (rgbState == 0){
		GPIO_SetValue(2, 1);
	}else{
		GPIO_ClearValue(2, 1);
	}
	rgbState = (rgbState + 1) % 2;
}

char* getState(void);

void updateOLED(void){
	sprintf(tempString, "Temp: %d.%d  ", dataBuffer.temperature/10, dataBuffer.temperature%10);
	sprintf(lightString, "Lux: %d  ", dataBuffer.lightIntensity);
	sprintf(varString, "Var: %d   ", calcAccVarZ());
	sprintf(stateString, "State: %s  ", getState());
	oled_putString(0, 1 + 8, tempString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 1 + 16, lightString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 1 + 24, varString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 1 + 32, stateString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

void SysTick_Handler(void) {
	if (msTicks < 5000){
		msTicks = 5000;
	}
	msTicks++;
	if (msTicks - timeState.accelerationTime > timeState.lastAccTime && i2c0Lock == 0){
		timeState.lastAccTime = msTicks;
		i2c0Lock = 1;
		accelerationHandler();
		i2c0Lock = 0;
	}
	if (msTicks - lcdTime > timeState.lastLCDTime && spiLock == 0){
		timeState.lastLCDTime = msTicks;
		spiLock = 1;
		led7seg_update();
		spiLock = 0;
	}
	if ((msTicks - RELAY_RGB_TIME > timeState.lastRGBTime) && timeState.state != BRIGHT){
		timeState.lastRGBTime = msTicks;
		toggleRGBLed();
	}
}

uint32_t getTicks(void){
    return msTicks;
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
 * UART
 */

static uint32_t UART_LOCK = 0;
static uint32_t parseMessageReady = 0;
static char circularCharBuffer[CHAR_ARRAY_BUFFER];
static uint32_t rearPointer = 0;
static char messageBuffer[CHAR_ARRAY_BUFFER];
static char relayedMessage[MESSAGE_WIDTH];

void UART3_IRQHandler(void){

	if (UART_LOCK != 0){
		return;
	}

	UART_LOCK = 1;
    if(LPC_UART3->IIR & 0x1) return;
    if(LPC_UART3->IIR & 0x4)
	while(LPC_UART3->LSR & 0x1){
		pushCharToStack(LPC_UART3->RBR);
	}

    UART_LOCK = 0;
}

void UART3_DISABLE_INT(void){
	LPC_UART3->IER = 0;
}

void UART3_ENABLE_INT(){
	LPC_UART3->IER = 0x1;
}

void pushCharToStack(uint8_t newChar){
	if (newChar == '\r'){
		parseMessageReady = 1;
		return;
	}

	if ((rearPointer + 1) == CHAR_ARRAY_BUFFER){
		return;
	}

	circularCharBuffer[rearPointer] = (char) newChar;
	rearPointer++;
}

void parseMessage(void){

	if (parseMessageReady == 0){
		return;
	}

	while(UART_LOCK == 1){
		;
	}
	UART_LOCK = 1;

	/*
	 * Sample Message
	 * #N083_T23.1_L132_V450#/r
	 */

	if (circularCharBuffer[0] != "#" || circularCharBuffer[21] != "#" || circularCharBuffer[1] != "N" || circularCharBuffer[6] != "T" || circularCharBuffer[12] != "L" || circularCharBuffer[17] != "V"){
		relayedMessage[0] = "0";
		rearPointer = 0;
		parseMessageReady = 0;
		UART_LOCK = 0;
		return;
	}

	strncpy(relayedMessage, circularCharBuffer + 1, MESSAGE_WIDTH);
	rearPointer = 0;
	parseMessageReady = 0;
	UART_LOCK = 0;
	return;
}

void printMessage(void){

	if(relayedMessage[0] == "0"){
		sprintf(messageBuffer, "%s_T%d.%d_L%d_V%d", NODE_ID, dataBuffer.temperature/10, dataBuffer.temperature%10, dataBuffer.lightIntensity, dataBuffer.zvar);
	}else{
		sprintf(messageBuffer, "%s_T%d.%d_L%d_V%d_%s", NODE_ID, dataBuffer.temperature/10, dataBuffer.temperature%10, dataBuffer.lightIntensity, dataBuffer.zvar, relayedMessage);
	}

	while(UART_LOCK == 1){
		;
	}
	UART_LOCK = 1;
	UART_SendString(LPC_UART3, (uint8_t*) messageBuffer);
	UART_LOCK = 0;
}

void init_uart(void){


	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	UART_LOCK = 1;
	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);
	UART_SendString(LPC_UART3, (uint8_t*)"Jelly UART Initialized.\r\n");
	UART_LOCK = 0;

	UART3_ENABLE_INT();
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
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(1, 1 << 31, 0);

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

void checkTempLight(void){

	dataBuffer.temperature = temp_read();
	i2c0Lock = 1;
	dataBuffer.lightIntensity = light_read();
	i2c0Lock = 0;
	if (timeState.state == RELAY){
		return;
	}

	if (dataBuffer.lightIntensity < LIGHT_THRESHOLD){
		timeState.state = DIM;
		timeState.accelerationTime = ACC_ST_DIM_RELAY;
		timeState.tempLightTime = LS_TS_ST_DIM_RELAY;
		i2c0Lock = 1;
		pca9532_setLeds(allLedsOff, 0xffff);
		i2c0Lock = 0;
	} else {
		timeState.state = BRIGHT;
		timeState.accelerationTime = ACC_ST_BRIGHT;
		timeState.tempLightTime = LS_TS_ST_BRIGHT;
		i2c0Lock = 1;
		pca9532_setLeds(allLedsOn, 0xffff);
		i2c0Lock = 0;
		if (rgbState == 1){
			GPIO_ClearValue(2, 1);
			rgbState = 0;
		}
	}

	if (dataBuffer.temperature >= TEMP_THRESHOLD){
		if (buzzerState == 0){
			buzzerState = 1;
			activateBuzzer();
		}
	}else{
		buzzerState = 0;
	}
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
	while(spiLock = 0){
		;
	}
	spiLock = 1;
	oled_fillRect(0,9,95,40,OLED_COLOR_WHITE);
	oled_putString(0,16, "Buzzer", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	spiLock = 0;
	while (((FIO_ReadValue(1) >> 31) ^ 0x0)) {
		GPIO_SetValue(0, 1<<26);
		Timer0_us_Wait(note / 2);

		GPIO_ClearValue(0, 1<<26);
		Timer0_us_Wait(note / 2);
	}
	while(spiLock = 0){
			;
	}
	spiLock = 1;
	oled_fillRect(0,9,95,40,OLED_COLOR_BLACK);
	spiLock = 0;
}

void enableButtonsInterrupts(void){

	PINSEL_CFG_Type PinCfg;

	/*
	 * SW3
	 */

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(2, 1 << 10, 0);

	LPC_GPIOINT->IO2IntEnF |= 1<<10;

	/*
	 * SW4
	 */

	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(1, 1 << 31, 0);

	/*
	 * RGB LED RED P1 9
	 */

	/*
	 * Red LED of RGB light due to conflict with OLED
	 */

	GPIO_SetDir(2, 1, 1);
	GPIO_SetDir(1, 1 << 9, 1);

	LPC_SC->EXTINT = 1;

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(EINT0_IRQn, NVIC_EncodePriority(4,0,0));
	NVIC_EnableIRQ(EINT0_IRQn);

	/*
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(EINT3_IRQn, NVIC_EncodePriority(4,2,0));
	NVIC_EnableIRQ(EINT3_IRQn);
	*/

	NVIC_ClearPendingIRQ(UART3_IRQn);
	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(UART3_IRQn, NVIC_EncodePriority(4,3,0));
	NVIC_EnableIRQ(UART3_IRQn);




}

char* getState(void){
	switch (timeState.state){
	case BRIGHT:
			return "BRIGHT";
			break;
	case DIM:
		return "DIM   ";
		break;
	case RELAY:
			return "RELAY ";
			break;
	default:
		return "NULL   ";
		break;
	}
}

void EINT0_IRQHandler(void){

	printf("EINT0\n");

	if (timeState.state != RELAY){
		printf("RELAY\n");
		timeState.state = RELAY;
		timeState.accelerationTime = ACC_ST_DIM_RELAY;
		timeState.tempLightTime = LS_TS_ST_DIM_RELAY;
		i2c0Lock = 1;
		pca9532_setLeds(allLedsOff, 0xffff);
		i2c0Lock = 0;
	}else{
		timeState.state = BRIGHT;
		printf("BRIGHT\n");
		timeState.accelerationTime = ACC_ST_BRIGHT;
		timeState.tempLightTime = LS_TS_ST_BRIGHT;
		i2c0Lock = 1;
		pca9532_setLeds(allLedsOn, 0xffff);
		i2c0Lock = 0;
	}
	toggleRGBLed();

	LPC_SC->EXTINT = 1;
	LPC_GPIOINT->IO2IntClr = 1 << 10;
	NVIC_ClearPendingIRQ(EINT0_IRQn);
}



void EINT3_IRQHandler(void){

	printf("EINT3\n");
	LPC_SC->EXTINT = 1;
	NVIC_ClearPendingIRQ(EINT3_IRQn);
}

static void GUI_drawTitlebar(void){
	//We want to overwrite only the title bar portion which is (0,0) to (95,8)
    oled_fillRect(0,0,95,8,OLED_COLOR_WHITE);							//Black bg
    oled_putString(1,1, "Edward | Jelly", OLED_COLOR_BLACK, OLED_COLOR_WHITE);	//White text on black bg
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

    temp_init(&getTicks);
    eeprom_init();
    light_enable();
    buzzer_init();
    initAccCB();
    initSystick();
    init_uart();

    i2c0Lock = 1;
    pca9532_setLeds(allLedsOn, 0xffff);
    i2c0Lock = 0;

    spiLock = 1;
    oled_clearScreen(OLED_COLOR_BLACK);
    oled_fillRect(0,8,95,8,OLED_COLOR_WHITE);
    GUI_drawTitlebar();
    spiLock = 0;

    initTimingState();
    enableButtonsInterrupts();
    __enable_irq();
    while(TRUE){

    	if (getTicks() - timeState.tempLightTime > timeState.lastTempLightTime){
    		timeState.lastTempLightTime = getTicks();
    		checkTempLight();
    	}

    	if (getTicks() - OLED_UPDATE > timeState.lastOLEDTime && spiLock == 0){
    		timeState.lastOLEDTime = msTicks;
			spiLock = 1;
			updateOLED();
			spiLock = 0;
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
