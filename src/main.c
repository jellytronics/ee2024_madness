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



#include "string.h"
#include "stdio.h"

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


#define N_SAMPLE 10
#define CHAR_ARRAY_BUFFER 100
#define MESSAGE_WIDTH 20

const uint8_t ACC_ST_BRIGHT = 50;
const uint8_t ACC_ST_DIM = 100;
const uint32_t LS_TS_ST_BRIGHT = 1000;
const uint32_t LS_TS_ST_DIM = 3000;

const uint32_t REPORTING_TIME = 5000;
const uint32_t DISTRESS_TIME = 1000;
const uint32_t RELAY_RGB_TIME = 2000;
const uint32_t OLED_UPDATE = 500;
const uint16_t allLedsOn = 0xff00;
const uint16_t allLedsOff = 0x0;

const uint32_t BRIGHT_CONDITION = 800;
const uint32_t TEMP_WARN = 260; //260

const uint32_t note = 1703;

const char *WELCOME_MESSAGE = "BOOTING\n";
const char *DEBUG_SYSTICK = "SYSTICK INIT ERROR\n";
const char *NODE_ID = "N001";
const char *SOURCE_ADD = "ATMY33a\r";
const char *DESTINATION_ADD = "ATDL33b\r";

/*
 * Program declarations
 *
 * This is to prevent conflicting types error
 *
 */

void pushCharToStack(uint8_t newChar);
void activateBuzzer(void);

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
static uint32_t lightBlocked = 0;

static char tempString[20];
static char lightString[20];
static char varString[20];
static char stateString[20];

static uint32_t UART_LOCK = 0;
static uint32_t parseMessageReady = 0;
static char circularCharBuffer[CHAR_ARRAY_BUFFER];
static uint32_t rearPointer = 0;
static char messageBuffer[CHAR_ARRAY_BUFFER];
static char relayedMessage[MESSAGE_WIDTH];
static uint32_t tempLightIntensity = 0;

/*
 * Function Declarations
 */

char* getState(void);

/*
 * Systick
 */

uint32_t getTicks(void){
    return msTicks;
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
	uint32_t lastParseTime;
	uint32_t lastReportingTime;
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
	timeState.lastParseTime = 0;
	timeState.lastReportingTime = 0;
}

/*
 * Data (Circular) Buffer
 */

struct dataCB {
	int pointer;
	int8_t x[N_SAMPLE];
	int8_t y[N_SAMPLE];
	int8_t z[N_SAMPLE];
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

void initAccCB(void){
	dataBuffer.pointer = 0;
	acc_read(&dataBuffer.xoff, &dataBuffer.yoff, &dataBuffer.zoff);
	dataBuffer.xvar = 0;
	dataBuffer.yvar = 0;
	dataBuffer.zvar = 0;
	dataBuffer.lightIntensity = 0;
	dataBuffer.temperature = 0;
}

/**
 * Accelerometer
 */

void nextElements(void){
	dataBuffer.pointer = (dataBuffer.pointer + 1) % N_SAMPLE;
}

void accelerationHandler(void){
	nextElements();
	acc_read(&dataBuffer.x[dataBuffer.pointer], &dataBuffer.y[dataBuffer.pointer], &dataBuffer.z[dataBuffer.pointer]);
}

int calcAccVarZ(void){

	int sumz = 0;
	int meanz = 0;
	int sumSQz = 0;
	int index;

	for (index = 0; index < N_SAMPLE; index++){
		//dataBuffer.z[index] = dataBuffer.z[index] - dataBuffer.zoff;
		sumz += dataBuffer.z[index];
	}

	meanz = sumz / N_SAMPLE;

	for (index = 0; index < N_SAMPLE; index++){
		sumSQz += (dataBuffer.z[index] - meanz)*(dataBuffer.z[index] - meanz);
	}

	dataBuffer.zvar = sumSQz/N_SAMPLE;

	return dataBuffer.zvar;
}

/*
 * RGB LED
 */

void toggleRGBLed(void) {
	if (rgbState == 0){
		GPIO_SetValue(2, 1);
	}else{
		GPIO_ClearValue(2, 1);
	}
	rgbState = (rgbState + 1) % 2;
}

/*
 * 7 Segment
 */

static void led7seg_update(void){
	led7seg_setChar(lcdNumber + ASCII_Number, 0);
	lcdNumber = (lcdNumber + 1) % 10;
}

/**
 * UART
 */

void UART3_ENABLE_INT(void){
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

	/*
	 * Sample Message
	 * #N083_T23.1_L132_V450#/r
	 */

	if (circularCharBuffer[0] != '#' || circularCharBuffer[21] != '#' || circularCharBuffer[1] != 'N' || circularCharBuffer[6] != 'T' || circularCharBuffer[12] != 'L' || circularCharBuffer[17] != 'V'){
		if (relayedMessage[1] == '1'){
			relayedMessage[0] = '\0';
		}
		circularCharBuffer[0] = '\0';
		rearPointer = 0;
		parseMessageReady = 0;
		return;
	}

	strncpy(relayedMessage, circularCharBuffer + 1, MESSAGE_WIDTH);
	relayedMessage[20] = '\0';
	circularCharBuffer[0] = '\0';
	rearPointer = 0;
	parseMessageReady = 0;
	return;
}

void printUARTMessage(void){

	if (dataBuffer.temperature >= 1000){
		dataBuffer.temperature = 999;
	}

	if (dataBuffer.lightIntensity >= 1000){
		tempLightIntensity = 999;
	}else{
		tempLightIntensity = dataBuffer.lightIntensity;
	}

	if(relayedMessage[0] == '\0' || timeState.state != RELAY){
		sprintf(messageBuffer, "%s_T%02d.%01d_L%03d_V%03d\r", NODE_ID, (int) dataBuffer.temperature/10, (int) dataBuffer.temperature%10, (int) tempLightIntensity, (int) dataBuffer.zvar);
	}else{
		sprintf(messageBuffer, "%s_T%02d.%01d_L%03d_V%03d_%s", NODE_ID, (int) dataBuffer.temperature/10, (int) dataBuffer.temperature%10, (int) tempLightIntensity, (int) dataBuffer.zvar, relayedMessage);
		messageBuffer[41] = '\r';
		messageBuffer[42] = '\0';
		relayedMessage[0] = '\0';
		relayedMessage[1] = '1';
	}

	UART_SendString(LPC_UART3, (uint8_t*) messageBuffer);
}

/*
 * Initialization
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

void init_xBee() {

	/*
	 * Code adpated from
	 *
	 * https://github.com/ashrayjain/ee2024-assignment2
	 */

	UART_DeInit(LPC_UART3);

	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);

   	// configure xBee source and destination addresses
	UART_SendString(LPC_UART3, (uint8_t*) SOURCE_ADD);
	xBee_checkOk();
	UART_SendString(LPC_UART3, (uint8_t*) DESTINATION_ADD);
	xBee_checkOk();
	UART_SendString(LPC_UART3, (uint8_t*) "ATWR\r");
	xBee_checkOk();
	UART_SendString(LPC_UART3, (uint8_t*) "ATCN\r");
	xBee_checkOk();

	UART_DeInit(LPC_UART3);

	init_uart();
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

	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);


	LPC_UART3->IER |= UART_IER_THREINT_EN;
	LPC_UART3->FCR |= UART_FCR_TRG_LEV0;
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);
	UART_SendString(LPC_UART3, (uint8_t*)"Jelly UART Initialized.\r\n");

	NVIC_ClearPendingIRQ(UART3_IRQn);
	NVIC_SetPriority(UART3_IRQn, NVIC_EncodePriority(4,3,0));
	NVIC_EnableIRQ(UART3_IRQn);

	UART3_ENABLE_INT();
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

void buzzer_init(){

	/*
	 * Enables buzzer ++
	 */

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

static void initLight(void){
	while (i2c0Lock == 1){
		;
	}
	i2c0Lock = 1;
	light_enable();
	light_setRange(LIGHT_RANGE_64000);
	light_setHiThreshold(BRIGHT_CONDITION);
	light_setLoThreshold(0);
	light_clearIrqStatus();
	i2c0Lock = 0;
}

static void initOLED(void){
    spiLock = 1;
    oled_clearScreen(OLED_COLOR_BLACK);
    oled_fillRect(0,8,95,8,OLED_COLOR_WHITE);
    GUI_drawTitlebar();
    spiLock = 0;
}

void init7Seg() {
	led7seg_init();
	led7seg_setChar(' ', 0);
}

void enableButtonsInterrupts(void){

	PINSEL_CFG_Type PinCfg;

	/*
	 * SW4 - ALARM / WAKE UP
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
	 * Red LED of RGB light due to conflict with OLED
	 */

	GPIO_SetDir(1, 1 << 9, 1);

	/*
	 * SW3 - TOGGLE
	 */

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(2, 1 << 10, 0);

	/*
	 * Clears interrupt
	 */

	LPC_SC->EXTINT = 1;
	NVIC_SetPriorityGrouping(4);

	LPC_GPIOINT->IO2IntEnF |= 1 << 10;

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_SetPriority(EINT0_IRQn, NVIC_EncodePriority(4,0,0));
	NVIC_EnableIRQ(EINT0_IRQn);

	LPC_GPIOINT->IO2IntEnF |= 1 << 5;

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriority(EINT3_IRQn, NVIC_EncodePriority(4,2,0));
	NVIC_EnableIRQ(EINT3_IRQn);

}


/*
 * Buzzer
 */

void activateBuzzer(void) {

	/*
	 * Handles both OLED and Buzzer
	 */

	while(spiLock == 1){
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

	while(spiLock == 1){
			;
	}
	spiLock = 1;
	oled_fillRect(0,9,95,40,OLED_COLOR_BLACK);
	spiLock = 0;
}

/*
 * Light and Temp
 */

void checkTempLight(void){

	dataBuffer.temperature = temp_read();
	i2c0Lock = 1;
	dataBuffer.lightIntensity = light_read();
	i2c0Lock = 0;

	if (timeState.state == RELAY){
		return;
	}

	if (dataBuffer.lightIntensity <= BRIGHT_CONDITION){
		timeState.state = DIM;
		timeState.accelerationTime = ACC_ST_DIM;
		timeState.tempLightTime = LS_TS_ST_DIM;

		while (i2c0Lock){
			;
		}
		i2c0Lock = 1;
		light_setHiThreshold(BRIGHT_CONDITION);
		light_setLoThreshold(0);
		i2c0Lock = 0;
	}

	i2c0Lock = 1;
	pca9532_setLeds(allLedsOn, 0xffff);
	i2c0Lock = 0;

	if (dataBuffer.temperature >= TEMP_WARN){
		if (buzzerState == 0){
			buzzerState = 1;
			activateBuzzer();
		}
	}else{
		buzzerState = 0;
	}
}

/*
 * Interrupt Handlers
 */

void EINT0_IRQHandler(void){

	if (timeState.state != RELAY){

		timeState.state = RELAY;
		timeState.accelerationTime = ACC_ST_DIM;
		timeState.tempLightTime = LS_TS_ST_DIM;
		i2c0Lock = 1;
		pca9532_setLeds(allLedsOff, 0xffff);
		i2c0Lock = 0;
		toggleRGBLed();

		NVIC_DisableIRQ(EINT3_IRQn);

	}else{

		if (rgbState == 1){
			GPIO_ClearValue(2, 1);
			rgbState = 0;
		}

		i2c0Lock = 1;
		pca9532_setLeds(allLedsOn, 0xffff);
		i2c0Lock = 0;

		if (dataBuffer.lightIntensity < BRIGHT_CONDITION){
			timeState.state = DIM;
			timeState.accelerationTime = ACC_ST_DIM;
			timeState.tempLightTime = LS_TS_ST_DIM;
			light_setHiThreshold(BRIGHT_CONDITION);
			light_setLoThreshold(0);
		}else{
			timeState.state = BRIGHT;
			timeState.accelerationTime = ACC_ST_BRIGHT;
			timeState.tempLightTime = LS_TS_ST_BRIGHT;
			light_setHiThreshold(63000);
			light_setLoThreshold(BRIGHT_CONDITION);
		}

		NVIC_EnableIRQ(EINT3_IRQn);

	}

	/*
	 * Debouncing
	 */

	uint32_t tempInt = 2500000;

	while(tempInt){
		tempInt--;
	}

	if ((LPC_GPIOINT->IO2IntStatF >> 10) & 0x1){
		LPC_GPIOINT->IO2IntClr = 1 << 10;
	}

	LPC_SC->EXTINT = 1;
	NVIC_ClearPendingIRQ(EINT0_IRQn);
}

void EINT3_IRQHandler(void){

	if ((LPC_GPIOINT->IO2IntStatF >> 5)& 0x1){

		if (i2c0Lock == 1){
			LPC_GPIOINT->IO2IntClr = (1 << 5);
			lightBlocked = 1;
			return;
		}

		if (timeState.state == DIM){
			timeState.state = BRIGHT;
			timeState.accelerationTime = ACC_ST_BRIGHT;
			timeState.tempLightTime = LS_TS_ST_BRIGHT;
		    light_setHiThreshold(62000);
		    light_setLoThreshold(BRIGHT_CONDITION);
		}else if (timeState.state == BRIGHT){
			timeState.state = DIM;
			timeState.accelerationTime = ACC_ST_DIM;
			timeState.tempLightTime = LS_TS_ST_DIM;
		    light_setHiThreshold(BRIGHT_CONDITION);
		    light_setLoThreshold(0);
		}

		LPC_GPIOINT->IO2IntClr = (1 << 5);

		i2c0Lock = 1;
		light_clearIrqStatus();
		i2c0Lock = 0;

	}

	LPC_SC->EXTINT = 1;
	NVIC_ClearPendingIRQ(EINT3_IRQn);
}

void UART3_IRQHandler(void){

	/*
	UART_GetLineStatus(LPC_UART3);
	uartReadBuffer;
	UART3_StdIntHandler();
	*/

    if(LPC_UART3->IIR & UART_IIR_INTID_RDA){
    	while(LPC_UART3->LSR & 0x1){
			pushCharToStack(LPC_UART3->RBR);
		}
    }

}

void SysTick_Handler(void) {

	/*
	 * Init msTicks to prevent overflow error
	 */

	if (msTicks < 5000){
		msTicks = 5000;
	}

	msTicks++;
	if (msTicks - timeState.accelerationTime > timeState.lastAccTime && i2c0Lock == 0){
		timeState.lastAccTime = msTicks;
		i2c0Lock = 1;
		accelerationHandler();
		i2c0Lock = 0;
		if (lightBlocked == 1){
			light_clearIrqStatus();
			lightBlocked = 0;
		}
	}
	if (msTicks - lcdTime > timeState.lastLCDTime && spiLock == 0){
		timeState.lastLCDTime = msTicks;
		spiLock = 1;
		led7seg_update();
		spiLock = 0;
	}
	if ((msTicks - RELAY_RGB_TIME > timeState.lastRGBTime) && timeState.state == RELAY){
		timeState.lastRGBTime = msTicks;
		toggleRGBLed();
	}
}

/*
 * GUI
 */

void updateOLED(void){
	sprintf(tempString, "Temp: %d.%d  ", (int) dataBuffer.temperature/10, (int) dataBuffer.temperature%10);
	sprintf(lightString, "Lux: %d  ", (int) dataBuffer.lightIntensity);
	sprintf(varString, "Var: %d   ", calcAccVarZ());
	sprintf(stateString, "State: %s  ", getState());
	oled_putString(0, 1 + 8, tempString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 1 + 16, lightString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 1 + 24, varString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	oled_putString(0, 1 + 32, stateString, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
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

void GUI_drawTitlebar(void){
	//We want to overwrite only the title bar portion which is (0,0) to (95,8)
    oled_fillRect(0,0,95,8,OLED_COLOR_WHITE);							//Black bg
    oled_putString(1, 1, "Edward | Jelly", OLED_COLOR_BLACK, OLED_COLOR_WHITE);	//White text on black bg
}


int main (void) {

	/**
	 * Initialisation Steps
	 */

	__disable_irq();

	init_i2c();
    init_ssp();
	init7Seg();
    pca9532_init();
    joystick_init();
    acc_init();
    oled_init();
    temp_init(&getTicks);
    //eeprom_init();
    initLight();
    buzzer_init();
    initAccCB();
    initOLED();


    initTimingState();
    enableButtonsInterrupts();
    init_uart();
    initSystick();

    __enable_irq();

    /*
     * Main Programme
     */

    while(TRUE){

    	if (getTicks() - timeState.tempLightTime > timeState.lastTempLightTime){
    		timeState.lastTempLightTime = getTicks();
    		checkTempLight();
    	}

    	if (getTicks() - OLED_UPDATE > timeState.lastOLEDTime && spiLock == 0){
    		timeState.lastOLEDTime = getTicks();
			spiLock = 1;
			updateOLED();
			spiLock = 0;
		}

    	if (getTicks() - DISTRESS_TIME > timeState.lastParseTime){
    		timeState.lastParseTime = getTicks();
    		parseMessage();
    	}

    	if (getTicks() - REPORTING_TIME > timeState.lastReportingTime){
			timeState.lastReportingTime = getTicks();
			printUARTMessage();
		}


    	/*
    	 * Add timing for send Message -> ;
    	 */



    }


}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
