

#include <Wire.h>

#include <SPI.h>

/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_MAX31865.h>
#include <pt100rtd.h>


// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 max = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0!
#define RREF 430.0
#define RNOMINAL 100.0

// Like, duh.
#define C2F(c) ((9 * c / 5) + 32)

// init the Pt100 table lookup module
pt100rtd PT100 = pt100rtd() ;

void setup()
{
	Serial.begin(57600);
	Serial.println("MAX31865 PT100 Sensor Test using NIST resistance table.");

	//max.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
	max.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
	//max.begin(MAX31865_4WIRE);	// set to 2WIRE or 4WIRE as necessary
}

void loop()
{
	uint16_t rtd, ohmsx100 ;
	uint32_t dummy ;
	float ohms, Tlut ;  
	float Tcvd, Tcube, Tpoly, Trpoly ;

	rtd = max.readRTD();

	// fast integer math:
	// fits in 32 bits as long as (100 * RREF) <= 2^16,
	//  i.e., RREF must not exceed 655.35 ohms (heh).
	// TO DO: revise for 4000 ohm reference resistor needed by Pt1000 RTDs
 
	// Use uint16_t (ohms * 100) since it matches data type in lookup table.
	dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(RREF)) ;
	dummy >>= 16 ;
	ohmsx100 = (uint16_t) (dummy & 0xFFFF) ;

	// or use exact ohms floating point value.
	ohms = (float)(ohmsx100 / 100) + ((float)(ohmsx100 % 100) / 100.0) ;

	//Serial.print("rtd: 0x") ; Serial.print(rtd,HEX) ;
	//Serial.print(", ohms: ") ; Serial.println(ohms,2) ;
 
  // compare lookup table and common computational methods
  
	Tlut	= PT100.celsius(ohmsx100) ;			// NoobNote: LUT== LookUp Table
	
	// report temperatures at 0.001C resolution to highlight methodological differences
Serial.print(Tlut  ,3); Serial.print(","); Serial.println(max.temperature(RNOMINAL,RREF));

	}
