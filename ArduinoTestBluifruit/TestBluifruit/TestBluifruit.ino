/*********************************************************************
  This is an example based on nRF51822 based Bluefruit LE modules

********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
       -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define LED2                     9
#define LED1                     3
/*=========================================================================*/


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int b = 0;
int c = 0;
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();



  ble.verbose(false);  // debug info is a little annoying after this point!


  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  //Give module a new name
  ble.println("AT+GAPDEVNAME=Bananerne"); // named Bananerne

  // Check response status
  ble.waitForOK();

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }
  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));



  pinMode (LED2, OUTPUT);
  pinMode (LED1, OUTPUT);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
String data = "";

void loop(void)
{
  // Check for user input
  char n, inputs[BUFSIZE + 1];

  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }
  if (ble.available()) {
    Serial.print("* "); Serial.print(ble.available()); Serial.println(F(" bytes available from BTLE"));
  }
  // Echo received data

  while ( ble.available() )
  {

    //int c = ble.read();
    char c = ble.read();
    //c = ble.parseInt();
    data += c;
    //Serial.print((char)c);
    //if ( c == '1')
    //digitalWrite (LED, HIGH);
    //analogWrite (LED, c)
    //if (c == '0')
    //digitalWrite (LED, LOW);
  }
  Serial.print("Lysstyrke: ");
  Serial.println (data);
  String LEDNummer = data.substring(0, 2);
  Serial.print("LEDNummer: ");
  Serial.println(LEDNummer);
  String lysstyrke = data.substring(3);
  Serial.print("lysstyrke: ");
  Serial.println(lysstyrke);
  data = "";

  delay(1000);
  int lys = lysstyrke.toInt();
  if (LEDNummer == "D1")
    analogWrite (LED1, lys);
  if (LEDNummer == "D2")
  analogWrite (LED2, lys);
  /* (ble.available() )
    {
    b = ble.parseInt ();
    }
    Serial.print("Hej: ");
    Serial.println (b);
    analogWrite (LED1, b);*/
  delay(1000);

//if sætning der gør at når man er logget af BLE slukker lysene. VIRKER IKKE!!!
  if (!ble.available) {
    analogWrite (LED1, LOW);
    analogWrite(LED2, LOW);
  }
}
