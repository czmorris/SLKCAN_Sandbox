/**
 * @brief   This example enables the listen-only mode and prints all received CAN frames to the serial interface.
 * @author  Alexander Entinger, MSc / LXRobotics GmbH
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>

#include <ArduinoMCP2515.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define GEARMODEID 0xA0
#define BMSID      0x19
#define SPDID      0x98

#define CLS             "\033[2J"
#define HOME            "\033[H"

#undef max
#undef min
#include <algorithm>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 15;
static int const MKRCAN_MCP2515_INT_PIN = 4;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void    spi_select           ();
void    spi_deselect         ();
uint8_t spi_transfer         (uint8_t const);
void    onExternalEvent      ();
void    onReceiveBufferFull  (uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

SPIClass spiH(HSPI);

char printbuff[200];

ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);

BluetoothSerial SerialBT;                    

bool  AttemptStream; 
byte  BatterySoc;
float PossibleAmps;
byte  PossibleMaxAmps;
byte  repspd;
byte  gearmode; 

TaskHandle_t Task1;  // MCP high priority updates from CAN
TaskHandle_t Task2;  // Bluetooth Terminal Interface

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  Serial.println("Application Started");
  while(!Serial) { }

  spiH.begin(14,12,13,15); //CLK,MISO,MOIS,SS

  /* Setup SPI access */
  spiH.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  //pinMode(32, OUTPUT);
  //digitalWrite(32, LOW);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  // Note: 20210309/CZM, Doesn't this require IRAM def for all called ISR routines?
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_500kBPS_8MHZ);
  //mcp2515.setListenOnlyMode();
  mcp2515.setNormalMode();
  Serial.println("Listening");

  SerialBT.begin("SLKCAN"); //Bluetooth device name


xTaskCreatePinnedToCore(   mcpTask,          /* Task function. */
                           "mcpTask",        /* String with name of task. */
                           10000,            /* Stack size in bytes. */
                           NULL,             /* Parameter passed as input of the task */
                           1,                /* Priority of the task. */
                           &Task1,            /* Task handle. */
                           1 );              /* Core */

delay(100);

xTaskCreatePinnedToCore(   BTTask,          /* Task function. */
                           "BTTask",        /* String with name of task. */
                           10000,            /* Stack size in bytes. */
                           NULL,             /* Parameter passed as input of the task */
                           1,                /* Priority of the task. */
                           &Task2,            /* Task handle. */
                           0 );              /* Core */

delay(100);
                           
}

// Used for VT100 bluetooth testing. 
void clearTermScr()
{
  SerialBT.print(CLS);
  SerialBT.print(HOME);
}


void mcpTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. 
  for(;;)
  {
    if(digitalRead(MKRCAN_MCP2515_INT_PIN)==LOW)
    {
      mcp2515.onExternalEventHandler();
    }    
    delay(1);
  }
}

// Decicated to printing a terminal screen to bluetooth for testing. 
void BTTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. 
  for(;;)
  {
    //clearTermScr();
    //SerialBT.println("SLKCAN Bluetooth Terminal");
    //SerialBT.println(" ");
    //SerialBT.print("Gear/Mode: ");

//    switch(gearmode)
//    {
//      case 0x01: // ECO
//        SerialBT.print("ECO");
//      break;
//      case 0x02: // POWER
//        SerialBT.print("POWER");
//      break;
//      case 0x03: // REVERSE
//        SerialBT.print("REVERSE");
//      break;
//      case 0x17: // PARK
//        SerialBT.print("PARK");
//      break;
//      default:
//      break;
//    }  

    SerialBT.print(gearmode);
    SerialBT.print(",");
    SerialBT.print(BatterySoc);
    SerialBT.print(",");
    SerialBT.print(repspd);
    SerialBT.print(",");
    SerialBT.print(PossibleAmps);
    SerialBT.println(";");
    SerialBT.flush();
      
//bool  AttemptStream; 
//byte  BatterySoc;
//float PossibleAmps;
//byte  PossibleMaxAmps;
//byte  repspd;
//byte  gearmode; 

    delay(500);
  }
}

// Consider adding a task here to print a terminal window.
// A command can be used to swap between streaming incoming CAN
// and the terminal. This allows for debugging known variables
// and scanning for new logfiles. 

void loop()
{
  //digitalWrite(32, HIGH);
  //delay(500);
  //digitalWrite(32, LOW);
  //delay(100);
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return spiH.transfer(data);
}

void onExternalEvent()
{
  GPIO.pin[MKRCAN_MCP2515_INT_PIN].int_ena = 0; 
  //digitalWrite(32, LOW);
  mcp2515.onExternalEventHandler();
  //digitalWrite(32, HIGH); 
  GPIO.pin[MKRCAN_MCP2515_INT_PIN].int_ena = 1; 
  GPIO.status_w1tc = GPIO.status;
}

void onReceiveBufferFull(uint32_t const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  char * pek;
  int i = 0;
    
    //Clear the buffer
    for(i = 0; i < 200; i++)
    {printbuff[i] = 0x00;}
  
    // Setup the pointer to the beginning
    pek = &printbuff[0];
    
    // Change this to a format that can be understood by can2sky... 
    // Can we get time in seconds?
  
    // Print the start of the line, adjust the buffer
    pek += sprintf(pek, "%d %08X %d ", timestamp_us, id, len);
  
    // Print each data byte, adjust the buffer
    for(i = 0; i < len; i++)
    {
      pek += sprintf(pek, "%02X ", data[i]);
    }
  
    // Finally print the entire buffer.
    Serial.println(printbuff);

    // Lets pick out some data... 
    switch(id)
    {
      case GEARMODEID:
        gearmode = data[0];
      break;
      case BMSID:
        BatterySoc = data[1];
        PossibleAmps = ((data[6]/255)*data[5]);
      break;
      case SPDID:
        repspd - data[2];
      break;
      default:
      break;
    }

    //bool  AttemptStream; 
//byte  BatterySoc;
//float PossibleAmps;
//byte  PossibleMaxAmps;
//byte  repspd;
//byte  gearmode; 

}