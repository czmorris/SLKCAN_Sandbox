/*
Note: This sketch uses the MCP2551 library (Alexander Entinger).

This sketch was created by Zach M. for the purpose of acting as a sandbox for the SLKCAN
board. This could be used as a jumping off point for anyone interested in working with the board.
This is not a commercial product and no warranty is provided. This code is not guaranteed to be 
error free. It is quick code intended to test the features of the board. 

This code may change frequently as a testbed for the board.
*/

#include "SD.h"
#include <SPI.h>
#include <ArduinoMCP2515.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define STREAMCANMSGS   // Define to stream to the serial port for logging.
// When not defined a simplistic terminal window will be printed instread.
// Use putty or some other terminal emulator for viewing the data.

#define BTTESTTERM     /* If defined bluetooth sends a terminal output 
rather than realdash data */


#define CLS             "\033[2J"
#define HOME            "\033[H"

// Note: Why? 
#undef max
#undef min
#include <algorithm>

static int const MKRCAN_MCP2515_CS_PIN  = 15;
static int const MKRCAN_MCP2515_INT_PIN = 35; // Note: Caused bluetooth issues on 4, SLKCAN01. 
/* 20210408 Moved to IO35, This goes up to 5v so it really should have been tri-stated/shifted. Oops... 
On IO35 it doesn't seem to bother bluetooth. This needs to be tested with the bike later
to make sure that it actually works!*/


#define SD_CS 5  // Micro-SD card chip select. 


void    spi_select           ();
void    spi_deselect         ();
uint8_t spi_transfer         (uint8_t const);
void    onExternalEvent      ();
void    onReceiveBufferFull  (uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

SPIClass spiH(HSPI);    // HSPI is used as to not conflict with the internal SPI flash on the ESP32
SPIClass spiV(VSPI);    // VSPI is used with the SD Card.

char printbuff[200];    // Buffer used to support string printing.

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
byte  repthrottle;
byte  gearmode; 
byte  odo;      // can't possibly be just one byte. need to understand can msg better.
byte  tripkm;
float tripkmfloat;
byte  tripmi;
byte  ssStatus;
byte  posTemp;
byte  blank;
float watts;
float amps;
bool  SDIsInit = false;
char filebuffer[50];

TaskHandle_t Task1;  // MCP high priority updates from CAN
TaskHandle_t Task2;  // Bluetooth Terminal Interface



// Primary entry point of the application. 
void setup()
{
  // Level Translator IC Enable. (Active High)
  //pinMode(26, OUTPUT);
  //digitalWrite(26, HIGH);

  // Heartbeat pin
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH); // Start off

  Serial.begin(115200);
  Serial.println("Application Started");
  while(!Serial) { }

  ssStatus = 0;
  blank = 0;
  tripkm = 0;

  spiH.begin(14,12,13,15); //CLK,MISO,MOIS,SS

  /* Setup SPI access */
  spiH.begin();  // Note: Why again?
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  
  // CZM, During testing I found that the ISR was getting hammered and interupt watchdog resets would happen.
  // I elected to use a dedicated task for handling messages as fast as it could instead of high interrupt load.
  // Note that not every message will be seen.
  //attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_500kBPS_8MHZ);
  mcp2515.setNormalMode();

  // VSPI is used for the SD Card.
  spiV.begin(18, 19, 23, 5);  // CLK, MISO, MOSI, SS

  Serial.println("Listening");
  SerialBT.begin("SLKCAN"); //Bluetooth device name

  // This creates the tasks. 
  // MCP reading pinned to Core 1
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



// The task handler for the MCP2551.
// Instead of interrupts it just checks if the interrupt pin is low.
void mcpTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. 
  // Note: This will not catch every single message!
  for(;;)
  {
    if(digitalRead(MKRCAN_MCP2515_INT_PIN)==LOW)
    {
      mcp2515.onExternalEventHandler();
    }    
    delay(5);
  }
}

// Decicated to printing a terminal screen to bluetooth for testing. 
// Trying different things here now playing with different phone terminal emulators.
// Eventually need to write a custom phone app. 
void BTTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. 
  for(;;)
  { 
    
  // If STREAMCANMSGS is not enabled then print a terminal window on normal serial
  #ifndef STREAMCANMSGS
     printTermScreen();
     delay(500);
  #endif


#ifdef BTTESTTERM

    // Print a terminal output to bluetooth.
    SerialBT.print(CLS);
    SerialBT.print(HOME);
    SerialBT.print("BatSoc: ");
    SerialBT.println(BatterySoc);
    SerialBT.print("repthrottle: ");
    SerialBT.println(repthrottle);
    SerialBT.print("GearMode: ");
    SerialBT.println(gearmode);
    SerialBT.print("Odo: ");
    SerialBT.println(odo);
    SerialBT.print("PossibleAmps: ");
    SerialBT.println(PossibleAmps);
    SerialBT.print("tripkm: ");
    SerialBT.println(tripkm);
    SerialBT.print("ssStatus: ");
    SerialBT.println(ssStatus);
    SerialBT.print("posTemp: ");
    SerialBT.println(posTemp);

#else
    SendCANFramesToSerialBT();
#endif

    delay(100);
  }
}


// This is the general arduino loop.
// so far unused. 
void loop()
{
  // Heartbeat LED... 
  // Note: Heartbeat LED not flashing? Design Issue?
  // Confirmed. LED D2 is backwards on the schematic. 
  // Turned it around on the test board and it works fine. 
  digitalWrite(16, LOW);
  delay(1000);
  digitalWrite(16, HIGH);
  delay(1000);
}

void spi_select()
{digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);}

void spi_deselect()
{digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);}

uint8_t spi_transfer(uint8_t const data)
{return spiH.transfer(data);}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

void onReceiveBufferFull(uint32_t const timestamp_us, uint32_t const id, uint8_t const * data, uint8_t const len)
{
  char * pek;
  int i = 0;

// Only stream the can messages to serial when the define is enabled. 
// See the top of this file.
#ifdef STREAMCANMSGS
  //Clear the buffer
  for(i = 0; i < 200; i++)
  {printbuff[i] = 0x00;}

  // Setup the pointer to the beginning
  pek = &printbuff[0];

  // Print the start of the line, adjust the buffer
  pek += sprintf(pek, "%d %08X %d ", timestamp_us, id, len);

  // Print each data byte, adjust the buffer
  for(i = 0; i < len; i++)
  {
    pek += sprintf(pek, "%02X ", data[i]);
  }

  // Finally print the entire buffer.
  Serial.println(printbuff);
  
  // Note: Add ifdef?
  // Note: How much will this impact timing? 
  writeSDCard(printbuff);  
    
#endif

  // Lets pick out some data... 
  switch(id)
  {
    case 0xA0:
      gearmode = data[0];   // ECO, POWER, REV, PARK
      break;
    case 0x19:
      BatterySoc = data[1];
      amps = (float)((data[6] + (255.0 * data[7])) / 100.0);   
      watts = (float)(amps * 72.0);     // This estimate is based on the rated voltage so its likely low. voltage can be in the 80s  
      break;
    case 0x98:
      repthrottle = data[3];  // This doesn't appear to be speed. Maybe some kind of demand signal. Not certain its throttle.
      break;
    case 0x2D0:
      odo = data[2];
      tripkm = (byte)(data[5]/10);        // Odometer kilometers
      tripkmfloat = (float)(data[5]/10.0); 
      //tripmi = (byte)(tripkm * 0.621371); // Odometer miles
      break;
    case 0x101:
      ssStatus = data[5]; // Possible Side Stand Status (or a bitfield of status indicators?) 
      break;
    case 0x3BA:
      posTemp = data[0];  // Possible controller temp? 
      break;      
    default:
      break;
  }

}

void SendCANFramesToSerialBT()
{
  byte buf[8];

  // build 1st realdash CAN frame, batterysoc, speed, gearmode
  memcpy(buf, &BatterySoc, 1);
  memcpy(buf + 1, &repthrottle, 1);
  memcpy(buf + 2, &gearmode, 1);
  memcpy(buf + 3, &odo, 1);
  memcpy(buf + 4, &PossibleAmps, 1);   
  memcpy(buf + 5, &tripkm, 1);
  memcpy(buf + 6, &ssStatus, 1);
  memcpy(buf + 7, &posTemp, 1);

  SendCANFrameToSerialBT(3201, buf); // ?? 3201 ??
}

void SendCANFrameToSerialBT(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  SerialBT.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  SerialBT.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  SerialBT.write(frameData, 8);
}

// Print a simplified terminal screen for quick testing CANbus finds. 
// Best viewed in a terminal emulator. (For example Putty)
void printTermScreen()
{ 
  Serial.print(CLS);
  Serial.print(HOME);
  Serial.print("Gear/Mode: ");
  Serial.println((int)gearmode);
  Serial.print("Battery Soc: ");
  Serial.println((int)BatterySoc);
  Serial.print("Possible Amps: ");
  Serial.println(amps);
  Serial.print("Estimated Watts: ");
  Serial.println(watts);
  Serial.print("Odometer Kilometers: ");
  Serial.println((int)odo);   // Note: This will eventually change. value doesn't fit into a byte.
  Serial.print("Trip Kilometers: ");
  Serial.println(tripkmfloat);  
  Serial.print("Throttle?: ");
  Serial.println((int)repthrottle);  
  Serial.print("Temp?: ");
  Serial.println((int)posTemp); 
  Serial.print("Stand Status?: ");
  Serial.println(ssStatus, HEX); 
}

// Try to reinit the sd card.
void trySDinit()
{
  if(!SD.begin(SD_CS, spiV))
  {
    //Serial.println("Card Mount Failed!");
    SDIsInit = false;
  }
  else
  {
    //Serial.println("SD Card Init!");
    SDIsInit = true;
  }
}

// Quick hack, nothing special. Needs more work.
void writeSDCard(char * data)
{

    // SD Card not initialized. Try
  if(SDIsInit == false)
  {
    trySDinit();
  }
  else
  {
    // Note: Add better checking here. This is just for testing.
   File file = SD.open("/TESTLOG.txt", FILE_APPEND);

   if(file)
   {        
     file.println(data);
     file.close();
   }
   else
   {
     SDIsInit = false; // Try to force a reinit on the next cycle... 
      //Serial.println("Could not write to the SD Card!");
   }     
  }
  
}
