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


#include <Wire.h> //I2C - 2-Wire
#include <OneWire.h>			//https://github.com/PaulStoffregen/OneWire
#include <MCP342X.h>			//https://github.com/uchip/MCP342X
#include <SevSeg.h>				//https://github.com/DeanIsMe/SevSeg
#include <DallasTemperature.h>	//https://github.com/milesburton/Arduino-Temperature-Control-Library

// Programs settings patern
struct Config{
	int16_t Temperatura;
	void test(){}
};

struct display_t{
	float value;
	uint8_t decimals;
	int miganie;
	bool state;
	int counter;
} display;

Config myConfig= {100}; //Initialise default parameters

//Menu buttons
#define BTN1_PIN 2
#define BTN2_PIN 3
#define BTN3_PIN 17

//PINY WYJSCIOWE
#define PWR_PIN 14
#define MOC_PIN 16

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
float temp18B20;



void setup()
{
	cli();
// Timer Period: 0,52429 s
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;
// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1);

// Timer Period: 8,192 ms
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (0<<CS20);
//TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);

TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;
// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);
	sei();

	//PRZYCISKI ++++++++++++++++++++++++++++++++++
	pinMode(BTN1_PIN,INPUT_PULLUP );
	pinMode(BTN2_PIN,INPUT_PULLUP );
	pinMode(BTN3_PIN,INPUT_PULLUP );

	//STEROWANIE WYJSCIAMI +++++++++++++++++++++++
	pinMode(PWR_PIN,OUTPUT );
	digitalWrite(PWR_PIN ,LOW); //wylaczony
	pinMode(MOC_PIN,OUTPUT);
	digitalWrite(MOC_PIN,HIGH); //wylaczony

	// dig led ++++++++++++++++++++++++++++++++++++
	byte numDigits = 3;
	byte digitPins[] = {13,10,9};
	byte segmentPins[] = {12,8,6,20,21,11,7,5};//A-G+DP
	sevseg.begin(COMMON_ANODE, numDigits, digitPins, segmentPins);
	sevseg.setBrightness(100);
	//RESISTORS_ON_SEGMENTS =1;

	// mcp +++++++++++++++++++++++++++++++++++++++++
	Wire.begin();  // join I2C bus
	TWBR = 12;  // 400 kHz (maximum)
	myADC.configure( MCP342X_MODE_CONTINUOUS | MCP342X_CHANNEL_1 | MCP342X_SIZE_18BIT | MCP342X_GAIN_8X );
	myADC.startConversion();

	//DS18B20 start-up +++++++++++++++++++++++++++++
	sensors.begin();
	sensors.getAddress(address18b30, 0);
	sensors.setResolution(address18b30, TEMPERATURE_PRECISION);
	sensors.setWaitForConversion(false);
	sensors.setCheckForConversion(true);
	sensors.requestTemperaturesByIndex(0); // Send the command to get temperatures
}

void LoadConfig()
{
	eeprom_read_block	(&myConfig, 0, sizeof(myConfig));
	eeprom_write_block	(&myConfig ,0, sizeof(myConfig));
}

void loop()
{
	digitalWrite(PWR_PIN, !digitalRead(BTN1_PIN));
	//DS18B20_Read();
	//temp18B20=26.6F;
	ReadMCP();
	//sevseg.refreshDisplay();
}

// timer0 overflow
ISR(TIMER1_OVF_vect)
{
	//test();
	//DS18B20_Read();
	 //ReadMCP();
}

ISR(TIMER2_OVF_vect)
{
	sevseg.refreshDisplay();
	//DS18B20_Read();
}

// Odczyt temperatury lokalnej z DS18B20
void DS18B20_Read()
{
	if (sensors.isConversionComplete())
	{
		temp18B20 = sensors.getTempC(address18b30);
		sevseg.setNumber(temp18B20,1);
		sensors.requestTemperaturesByIndex(0); // Send the command to get temperatures
	}
}

// Odczyt z DAC 18-bits
void ReadMCP() {
	static int32_t  result;
	myADC.startConversion();
	uint8_t status =  myADC.checkforResult18 (&result);
	if (status !=0xFF)
	{
		//0.000001953125	// 18-bit, 8X Gain
		float f= (result * 0.000001953125)*1000.0;
		sevseg.setNumber(f,2);
		//myADC.startConversion();
	}
}

// Odswiezenie wyswietlacza DigLed7x
void DisplayRefresh()
{
	if (display.miganie >0 )
	{
		// obsluga migania
		if (display.counter <200) 
		{
			display.counter++;
			return ;
		}
		display.counter=0;
		display.state =!display.state;
		if (display.state ) return;
		display.miganie --;
	}
	sevseg.setNumber(display.value ,display .decimals );
	sevseg.refreshDisplay(); // Must run repeatedly
}

//******************************************************************************************
void sprintDouble( char *str, double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)
  char st2[16];
  unsigned long frac;
  unsigned long mult = 1;
  int mant= int(val);
  byte padding = precision -1;
  byte sgn=0;

  sprintf(str,"");
  if (val < 0)  sprintf(str,"-");
  mant=abs(mant);
  sprintf(st2,"%d",mant); //prints the int part
  strcat(str,st2); 

  if( precision > 0) {
    strcat(str,".");

    while(precision--)
      mult *=10;

    if(val >= 0)
      frac = (val - int(val)) * mult;
    else
      frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    int cnt=precision;
    // while( frac1 /= 10 )
    while( frac1 = frac1 / 10 & cnt--  ) 
      padding--;
    while(  padding--)  strcat(str,"0");
    sprintf(st2,"%ld",frac);

    strcat(str,st2);
  }
}