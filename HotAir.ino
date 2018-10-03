/*==============================================================================================================*
    Hot-Air Simple Controler
*===============================================================================================================*

    @file     HotAir.ino
    @author   Zorbasoft - pawel kaminski
    @license  free (c) 2016 Zorbasoft
 
    Ver. 1.0.0 - First release (2015.11.20)

*===============================================================================================================*
    INTRODUCTION
*===============================================================================================================*


*===============================================================================================================*
    CONFIGURATION
*===============================================================================================================*
// board:
// https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json

*===============================================================================================================*
    LICENSE
*===============================================================================================================*

    The MIT License (MIT)
    Copyright (c) 2016 Zorbasoft

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
    documentation files (the "Software"), to deal in the Software without restriction, including without
    limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial
    portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
    LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*==============================================================================================================*/


#include "ValColl.h"
#include <Wire.h> //I2C - 2-Wire
#include <OneWire.h>			//https://github.com/PaulStoffregen/OneWire
#include <MCP342X.h>			//https://github.com/uchip/MCP342X
#include <SevSeg.h>				//https://github.com/DeanIsMe/SevSeg
#include <DallasTemperature.h>	//https://github.com/milesburton/Arduino-Temperature-Control-Library

// Programs settings patern
struct config_t {
	int16_t Temperatura = 100;
	//void Read() { eeprom_read_block(&config, 0, sizeof(config)); }
	//void Write() { eeprom_write_block(&config, 0, sizeof(config)); }
} config;
bool zapiszConfig;

#define config_read() eeprom_read_block(&config, 0, sizeof(config))
#define config_write() eeprom_write_block(&config, 0, sizeof(config))

/* +++++ Menu buttons +++++ */
// Przycisk on/off grzalki + opcja nawiewu
#define BTN_PWR		2
// Przycisk zmniejszania temperatury
#define BTN_LO		3
// Przycisk zwiekszania temperatury
#define BTN_HI		17 

//PINY WYJSCIOWE
#define PWR_PIN 14
#define MOC_PIN 16

const int TEMP_MIN = 100;
const int TEMP_MAX = 500;
const int TEMP_STEP = 5;

SevSeg sevseg; //Instantiate a seven segment led display

MCP342X myADC(0x69);

// DS18b20 resource ------------------------------
#define ONE_WIRE_BUS 15
#define TEMPERATURE_PRECISION 9 // Lower resolution; 9-12
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress address18b30; // We'll use this variable to store a found device address
ValColl tempContact(10);
ValColl tempK(50);
int counterPWM;
int counterMENU;
int counterLoop;

// LISTA MODOW WYSWIETLANIA
enum LedModeEnum
{
	NORMAL, SWITCH
};

// Info dla wyswietlacza
struct display_t {
	volatile LedModeEnum ledMode;
	bool miganieSwitch;
	bool isConfig;
	int counterConfig;
	void ResetConfigCounter() {
		counterConfig = 8;
		isConfig = true;
	}
} display;

// +++++ MAKRA +++++
// Czy przycisk jest wcisniety ?
#define isPressed(pin) !digitalRead(pin)
#define outToggle(pin) digitalWrite(pin, isPressed(pin))

struct trend_t {
	int temp;
	int trend;
	int odch;
	int pwm;
};

// dane do kontroli grzalki
struct controller_t {
	// temperatury skladowe
	// 0-K; 1-K-contakt; 2-Finalna
	int temps[3];	// = { 0,0,0 };
	int nextCursor; // index arrays ring
	bool configChanged = true; // temp. settings changed
	trend_t tr0; // current state
	trend_t tr1; // preview state
	trend_t tr2; // prev/prev st.
	int pwmCurr;
	// 0-disabled, 1-enabled
	int pwmEnabled = 1;
	bool isMin = false;
	//int * dogrz; // [] = { 0,0,0 };
	// zgloszenie zakonczenia pelnego cyklu pwm-1s
	void End_PWM_Cycle() {
		// czas podjac decyzje jak dalej sterowac
		tr0.temp = GetTempFinal();
		tr0.odch = tr0.temp - config.Temperatura;
		tr0.trend = tr0.temp - tr1.temp;
		tr0.pwm = GetCurrPWM();	// default last state
		if (!digitalRead(PWR_PIN))
			goto Rewrite;	// power is off

		if (tr0.odch > 7) {
			// ZA GOR¥CO - sporo ?
			pwmCurr--;
		}
		if (tr0.odch > 2) {
			// ZA GOR¥CO - troche ?
			pwmEnabled = 0; // wylaczam nastepny cykl, a moze zmniwjszyc ?
			configChanged = false;
			if (!isMin) pwmCurr--;
			goto Rewrite;
		}
		pwmEnabled = 1;
		if (configChanged && tr0.odch < 0) {
			isMin = false;
			int def = map(config.Temperatura, TEMP_MIN, TEMP_MAX, 6, 20);
			int add = map(tr0.odch, -config.Temperatura, 0, 40, 10);
			pwmCurr = def + add;
			goto Rewrite;
		}
		if (tr0.odch < -20) {
			configChanged = true;
			pwmCurr++;
			goto Rewrite;
		}
		if (tr0.odch < 3 && tr0.trend < 0) {
			goto Rewrite;
		}
		if (tr0.odch > -1 && tr0.trend > 0) {
			pwmEnabled = 0;
			goto Rewrite;
		}

		if (tr0.trend < 0 && tr1.trend < 0 && tr2.trend < 0) {
			isMin = true;
			pwmCurr++;
		}
	Rewrite:;
		tr0.pwm = GetCurrPWM();
		tr2 = tr1;
		tr1 = tr0;
	}
	
	// zwraca biezaca wartosc wypelnienia PWM lub zero jesli disabled;
	int GetCurrPWM() { return pwmCurr * pwmEnabled; }

	// Przemiatanie indexem dla tablicy wartosci temp. skladowych
	void GetNextCursor() {
		nextCursor++;
		if (nextCursor > 2) nextCursor = 0;
	}

	// zwraca kolejna temp. skladowa
	int GetNextTemp() { return temps[nextCursor]; }

	// ustaw i przelicz biezaca temp. glowna z termopary K
	void setTempK(int t) {
		temps[0] = t; temps[2] = temps[0] + temps[1] - 5;
	}

	// ustaw i przelicz temp. korygujaca - styk termopary K
	void setTempContact(int t) {
		temps[1] = t; temps[2] = temps[0] + temps[1] - 5;
	}

	// Zwraca tepm. po korekcji - rzeczywista
	int GetTempFinal() { return temps[2]; }
} controller;



void setup()
{
	config_read();
	if (config.Temperatura < TEMP_MIN || config.Temperatura>TEMP_MAX) config.Temperatura = TEMP_MIN;

	cli();

	// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000,000 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 10,001 ms
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);
	TCNT1H = 0xD8;
	TCNT1L = 0xEF;
	ICR1H = 0x00;
	ICR1L = 0x00;
	OCR1AH = 0x00;
	OCR1AL = 0x00;
	OCR1BH = 0x00;
	OCR1BL = 0x00;

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1);

	// Timer Period: ? ms
	ASSR = (0 << EXCLK) | (0 << AS2);
	TCCR2A = (0 << COM2A1) | (0 << COM2A0) | (0 << COM2B1) | (0 << COM2B0) | (0 << WGM21) | (0 << WGM20);
	//TCCR2B = (0 << WGM22) | (1 << CS22) | (1 << CS21) | (0 << CS20);
	// Timer Period: 4,096 ms
	//TCCR2B = (0 << WGM22) | (1 << CS22) | (0 << CS21) | (1 << CS20);
	// Timer Period: 2,048 ms
	TCCR2B = (0 << WGM22) | (1 << CS22) | (0 << CS21) | (0 << CS20);

	TCNT2 = 0x00;
	OCR2A = 0x00;
	OCR2B = 0x00;
	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2 = (0 << OCIE2B) | (0 << OCIE2A) | (1 << TOIE2);

	sei();

	//Buttons ++++++++++++++++++++++++++++++++++
	pinMode(BTN_PWR, INPUT_PULLUP);
	pinMode(BTN_LO, INPUT_PULLUP);
	pinMode(BTN_HI, INPUT_PULLUP);

	//Outputs +++++++++++++++++++++++
	pinMode(PWR_PIN, OUTPUT);
	digitalWrite(PWR_PIN, LOW); //wylaczony
	pinMode(MOC_PIN, OUTPUT);
	digitalWrite(MOC_PIN, HIGH); //wylaczony

	// 3x 7-segments digit led ++++++++++++++++++++++++++++++++++++
	byte numDigits = 3;
	byte digitPins[] = { 13,10,9 };
	byte segmentPins[] = { 12,8,6,20,21,11,7,5 };//A-G+DP
	sevseg.begin(COMMON_ANODE, numDigits, digitPins, segmentPins, true, false, false);
	sevseg.setBrightness(100);
	//RESISTORS_ON_SEGMENTS =1;

	// mcp +++++++++++++++++++++++++++++++++++++++++
	Wire.begin();  // join I2C bus
	TWBR = 12;  // 400 kHz (maximum)
	myADC.configure(MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_18BIT | MCP342X_GAIN_8X);
	myADC.startConversion();

	//DS18B20 start-up +++++++++++++++++++++++++++++
	sensors.begin();
	//sensors.getAddress(address18b30, 0);
	//sensors.setResolution(address18b30, TEMPERATURE_PRECISION);
	sensors.setWaitForConversion(false);
	sensors.requestTemperatures();
	sensors.setCheckForConversion(true);
	//sensors.requestTemperaturesByIndex(0); // Send the command to get temperatures
	display.ResetConfigCounter();

	controller.configChanged = true;
}


void loop()
{
	MenuCheck();
	if (counterLoop % 10 == 0)
		controller.setTempContact(GetTempContact()); // 10-30ms
	controller.setTempK(GetTempK()); // 10 ms
	counterLoop++;
	if (counterLoop >= 1001)counterLoop = 0;
}

void MenuCheck()
{
	checkPowerButton();
	checkTemperatureButton(BTN_HI,  TEMP_STEP);
	checkTemperatureButton(BTN_LO, -TEMP_STEP);
}

void checkPowerButton() {
	if (!isPressed(BTN_PWR)) return;
	unsigned long  ms = millis();
	while (isPressed(BTN_PWR)) {
		if (millis() - ms > 500)
		{
			// przelaczenie funkcji specjalnej
			if (display.ledMode == SWITCH) display.ledMode = NORMAL;
			else display.ledMode = SWITCH;
			goto ucieczka;
		}
	}
	// prze³¹czenie stanu power - trzeba sprawdzic
	if (digitalRead(PWR_PIN)) {
		digitalWrite(MOC_PIN, HIGH); //off grzalka
	}
	else {

	}
	outToggle(PWR_PIN);
ucieczka:;
	while (isPressed(BTN_PWR)); // czekam na zwolnienie przycisku
}

void checkTemperatureButton(uint8_t pin, int step)
{
	if (!isPressed(pin)) return;
	if (!display.isConfig)
	{
		display.ResetConfigCounter();
		delay(250);
		return;
	}
	while (isPressed(pin))
	{
		if (((config.Temperatura > TEMP_MIN) && (step < 0)) || ((config.Temperatura < TEMP_MAX) && (step > 0)))
		{
			config.Temperatura += step;
		}
		display.ResetConfigCounter();
		delay(250);
	}
	zapiszConfig = true;
	controller.configChanged = true;
}

// PWM 100sps
ISR(TIMER1_OVF_vect)//10ms
{
	// Reinitialize Timer1 value
	TCNT1H = 0xD8EF >> 8;
	TCNT1L = 0xD8EF & 0xff;
	// Place your code here
	if (digitalRead(PWR_PIN))
	{
		digitalWrite(MOC_PIN, counterPWM >= controller.GetCurrPWM());
	}
	else
		digitalWrite(MOC_PIN, HIGH); //off
	counterPWM++;
	if (counterPWM >= 100) {
		counterPWM = 0;
		controller.End_PWM_Cycle(); // zglaszam zakonczenie cyklu pwm
	}

	counterMENU++;
	if (counterMENU % 25 == 0){ //x10ms
		MenuEffect();
	}
	if (counterMENU % 200 == 0){
		controller.GetNextCursor();
	}
	if (counterMENU == 1000)counterMENU = 0;
}

void MenuEffect() {
	if (display.counterConfig > 0) display.counterConfig--;
	display.miganieSwitch = !display.miganieSwitch;
}

// szybki timer; Timer Period: 2,048 ms
ISR(TIMER2_OVF_vect)
{
	DisplayProcess();
}

void DisplayProcess() {
	//char decPlace = ledVal < 100 ? 1 : 0;
	//sevseg.setNumber(ledVal, decPlace);
	if (!display.isConfig) {
		switch (display.ledMode)
		{
		case NORMAL:
			sevseg.setNumber(controller.GetTempFinal(), 0);
			break;
		case SWITCH:
			sevseg.setNumber(controller.GetNextTemp(), 0);
			break;
		default:
			break;
		}
		goto refresh;
	}
	// animacja
	if (display.miganieSwitch)
	{
		sevseg.blank();
		return;
	}
	else
	{
		// czy koniec animacji
		if (display.counterConfig < 1) {
			display.isConfig = false;
			if(zapiszConfig) config_write();
			zapiszConfig = false;
		}
		sevseg.setNumber(config.Temperatura, 0);
	}
refresh:;
	sevseg.refreshDisplay();
}

// Odczyt temperatury lokalnej z DS18B20
float GetTempContact()
{
	if (sensors.isConversionComplete())
	{
		//temp18B20 = sensors.getTempC(address18b30);
		tempContact.Add( sensors.getTempCByIndex(0));
		//sensors.setWaitForConversion(false);
		sensors.requestTemperatures();
		//sensors.setCheckForConversion(true);
	}	
	return tempContact.Value();
}

// Temperatura termopary K - Odczyt z ADC 18-bits
int GetTempK() {
	static int32_t  result;
	myADC.startConversion();
	uint8_t status = myADC.checkforResult18(&result);
	if (status != 0xFF)
	{
		//0.000001953125	// 18-bit, 8X Gain
		float f = (result * 0.001953125)*1.0;
		//sevseg.setNumber(f,2);
		//myADC.startConversion();
		tempK.Add( f / 0.041);
	}
	return tempK.Value();
}