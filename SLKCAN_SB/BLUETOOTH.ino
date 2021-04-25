#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//#define BTTESTTERM     /* If defined bluetooth sends a terminal output rather than realdash data */

BluetoothSerial SerialBT; 

void BTSetup ()
{
    SerialBT.begin("SLKCAN"); //Bluetooth device name
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
  // This would probably be better done somewhere else...
  //#ifndef STREAMCANMSGS
  //   printTermScreen();
  //   delay(500);
  //#endif


#ifdef BTTESTTERM

    // Print a terminal output to bluetooth.
    SerialBT.print(CLS);
    SerialBT.print(HOME);
    SerialBT.print("Gear/Mode: ");
    SerialBT.println((int)gearmode);
    SerialBT.print("Battery Soc: ");
    SerialBT.println(BatterySoc);
    SerialBT.print("Amps: ");
    SerialBT.println(amps);
    SerialBT.print("Estimated Watts: ");
    SerialBT.println(watts);
    SerialBT.print("Odometer Miles: ");
    SerialBT.println((odo / 1.609));
    SerialBT.print("Trip Miles: ");
    SerialBT.println((tripkm / 1.609));  
    SerialBT.print("Speed kph: ");
    SerialBT.println(corrkph);  
    SerialBT.print("Speed mph: ");
    SerialBT.println((float)(corrkph / 1.609));    
    SerialBT.print("Controller Temp (C): ");
    SerialBT.println(posTemp); 
    SerialBT.print("Stand Status?: ");
    SerialBT.println(ssStatus, HEX); 
    SerialBT.print("Debug: ");
    SerialBT.println(debugcnt); 

#else
    SendCANFramesToSerialBT();
#endif

    delay(100);
  }
}

void SendCANFramesToSerialBT()
{
  byte buf[8];

  unsigned short RDOdo = (unsigned short)odo;              // Odometer in km float to int
  unsigned short RDTrip = (unsigned short)(tripkm * 10.0); // trip in km float to int. 
  byte CTemp = (byte)(posTemp);                            // Controller Temp
  byte kph = (byte)(kphhires);                             // For now kph.... change later
  byte BSoC = (byte)BatterySoc;

  // Made up frameid 3201
  // BatterySoc, speed, gearmode, odo, trip, controller temp
  memcpy(buf, &BSoC, 1);
  memcpy(buf + 1, &kph, 1);
  memcpy(buf + 2, &gearmode, 1);
  memcpy(buf + 3, &RDOdo, 2);
  // buf + 4 is still RDOdo  
  memcpy(buf + 5, &RDTrip, 2);
  // buf + 5 is still RDTrip
  memcpy(buf + 7, &CTemp, 1);

  SendCANFrameToSerialBT(3201, buf);
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
