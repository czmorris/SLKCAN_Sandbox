

//#define STREAMCANMSGS   // Define to stream to the serial port for logging.
// When not defined a simplistic terminal window will be printed instread.
// Use putty or some other terminal emulator for viewing the data. See TERMINAL tab.

char  printbuff[200];     // Buffer used to support string printing.
//bool  AttemptStream; 
float BatterySoc;         // Battery State of Charge %
byte  rawkph;             // not exactly kph, a factor of around 1.275 seems to be needed?
float corrkph;            // kph with factor applied
byte  gearmode;           // Gear/Mode Park, Eco, Power, Reverse
float odo;                // Odometer in kilometers
float tripkm;             // Trip meter in kilometers.. (Note: Try to capture the reset trip signal later...)
byte  ssStatus;
float posTemp;
float watts;              // For estimated watts. We don't have battery voltage via CAN yet. (if even available)
float amps;               // Filtered amps
bool  SDIsInit;
char  filebuffer[50];

int debugcnt;

TaskHandle_t Task1;  // MCP high priority updates from CAN
TaskHandle_t Task2;  // Bluetooth Terminal Interface

// Primary entry point of the application. 
void setup()
{
   debugcnt = 0;

  // Heartbeat pin, LED to flash when running.
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH); // Start off

  Serial.begin(115200);
  Serial.println("Application Started");
  while(!Serial) { }

  ssStatus = 0;
  tripkm = 0.0;

  // Setup MCP/CAN Interface
  SetupCAN();

  // Setup the SD-Card Interface
  SDSetup();

  // Setup Bluetooth
  BTSetup();

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
