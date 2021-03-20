// Note: This test sketch started life an example from the MCP2551 library (Alexander Entinger).
// It includes and makes use of the library but the sketch itself has been heavily modified. 

#include <SPI.h>
#include <ArduinoMCP2515.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define GEARMODEID 0xA0
#define BMSID      0x19
#define SPDID      0x98
#define UNKID      0x2D0

#define CLS             "\033[2J"
#define HOME            "\033[H"

// Note: Why? 
#undef max
#undef min
#include <algorithm>

static int const MKRCAN_MCP2515_CS_PIN  = 15;
static int const MKRCAN_MCP2515_INT_PIN = 4;

void    spi_select           ();
void    spi_deselect         ();
uint8_t spi_transfer         (uint8_t const);
void    onExternalEvent      ();
void    onReceiveBufferFull  (uint32_t const, uint32_t const, uint8_t const *, uint8_t const);

SPIClass spiH(HSPI);    // HSPI is used as to not conflict with the internal SPI flash on the ESP32
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
byte  repspd;
byte  gearmode; 
byte  odo;  // can't possibly be just one byte. need to understand can msg better.
byte blank;

TaskHandle_t Task1;  // MCP high priority updates from CAN
TaskHandle_t Task2;  // Bluetooth Terminal Interface

// Primary entry point of the application. 
void setup()
{
  Serial.begin(115200);
  Serial.println("Application Started");
  while(!Serial) { }

  blank = 0;

  spiH.begin(14,12,13,15); //CLK,MISO,MOIS,SS

  /* Setup SPI access */
  spiH.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  // has been used for debug. commented now.
  //pinMode(32, OUTPUT);
  //digitalWrite(32, LOW);

  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  // CZM, During testing I found that the ISR was getting hammered and interupt watchdog resets would happen.
  // I elected to use a dedicated task for handling messages as fast as it could instead of high interrupt load.
  //attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_500kBPS_8MHZ);
  //mcp2515.setListenOnlyMode();
  mcp2515.setNormalMode();
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

// Used for VT100 bluetooth testing. 
void clearTermScr()
{
  SerialBT.print(CLS);
  SerialBT.print(HOME);
}

// The task handler for the MCP2551.
// Instead of interrupts it just checks if the interrupt pin is low.
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
// Trying different things here now playing with different phone terminal emulators.
// Eventually need to write a custom phone app. 
void BTTask( void * parameter )
{
  // Get MCP messages as fast as possible...
  // but only as fast as possible. Does not allow ISR bog. 
  for(;;)
  {
    SendCANFramesToSerialBT();

    delay(100);
  }
}


// This is the general arduino loop.
// so far unused. 
void loop()
{
  //digitalWrite(32, HIGH);
  //delay(500);
  //digitalWrite(32, LOW);
  //delay(100);
}

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
  // Note: CZM. Commented out the interrupt handling stuff, its not necessary in the task driven configuration.
  //GPIO.pin[MKRCAN_MCP2515_INT_PIN].int_ena = 0; 
  //digitalWrite(32, LOW);
  mcp2515.onExternalEventHandler();
  //digitalWrite(32, HIGH); 
  //GPIO.pin[MKRCAN_MCP2515_INT_PIN].int_ena = 1; 
  //GPIO.status_w1tc = GPIO.status;
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
  
  // Changed this to a format that can be understood by can2sky... 
  // Note: can2sky didn't result in any helpful analysis but the format is fine how it is. 

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
      repspd = data[2];
    break;
    case UNKID:
      odo = data[2];
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
  memcpy(buf + 1, &repspd, 1);
  memcpy(buf + 2, &gearmode, 1);
  memcpy(buf + 3, &odo, 1);
  memcpy(buf + 4, &blank, 1);
  memcpy(buf + 5, &blank, 1);
  memcpy(buf + 6, &blank, 1);
  memcpy(buf + 7, &blank, 1);

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
