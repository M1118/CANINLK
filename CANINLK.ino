
#include <Arduino.h>
#include <SPI.h>
#include <MergCBUS.h>
#include <Message.h>
#include <EEPROM.h>
#include <SD.h>
#include <LiquidCrystal.h>

#define VERSION "1.0"
#define DEBUG   1

/**
   The following block of #defines configures the pins that are
   used for various special functions:
     CHIPSELECT  is the select pin for the CANBUS interface
     INTPIN      is the "interrupt" pin used by the CANBUS interface
                 to signal that a CANBUS packet is ready to be read
     BUZZER      is the pin the buzzer is connected to
     SDCS        is the pin used for the chip select of the SD card reader
*/
#define GREEN_LED      2    // MERG green led port
#define YELLOW_LED     3    // MERG yellow led port
#define PUSH_BUTTON    4    // std merg push button
#define CANPORT       10    // CBUS enable pin
#define INTPIN         8    // CBUS interupt pin
#define BUZZER         6    // Peizo buzzer pin
#define SDCS           7    // SD card chip select pin

#define N_LEVERS      64    // Number of levers supported

#define DEFAULT_BUZZER_FREQ 200
/**
   CBUS Node Variables:
      1     Number of columns to skip in the spreadsheet
      2     Number of rows to skip in the spreadsheet
      3     Buzzer frequency (10s of Hz)
      4     Debug output
*/
#define NUM_NODE_VARS     4   //the node variables
#define NUM_EVENTS        N_LEVERS        //supported events
#define NUM_EVENT_VARS    1   //no need for supported event variables
#define NUM_DEVICES       1   //one device number
#define MODULE_ID        58   //module id
#define MANUFACTURER_ID 165   //manufacturer id
#define MIN_CODE          0   //min code version
#define MAX_CODE          1   //max code version

#define BITS_PER_LONG     (8 * sizeof(unsigned long))
#define NUM_STATE_VARS    ((N_LEVERS + 7) / BITS_PER_LONG)

unsigned char debugmask = 0xff;

/* Debug node variable bit definitions */
#define DEBUGPULLOFF      0x01  // Output the conditions used to check if a lever can be pulled off
#define DEBUGPUTBACK      0x02  // Output the conditions used to check if a lever can be put back
#define DEBUGREQUEST      0x04  // Output the request data, i.e. which lever is being pulled off
#define DEBUGLOCK         0x08  // Output the lock table each time it is updated
#define DEBUGSTATE        0x10  // Output the state of the levers after every change

MergCBUS cbus = MergCBUS(NUM_NODE_VARS, NUM_EVENTS, NUM_EVENT_VARS, NUM_DEVICES);

class Display {
  public:
    Display(LiquidCrystal *lcd, int width) {
      m_lcd = lcd;
      m_width = width;
      m_insert = 0;
      m_line1 = (char *)malloc(width + 1);
      m_line2 = (char *)malloc(width + 1);
      m_line1[0] = 0;
      m_line2[0] = 0;
      m_offset = 0;
      m_fullLine[0] = 0;
    };

    void message(const char *msg) {
      if (m_insert == 0)
      {
        strncpy(m_line1, msg, m_width);
        m_insert++;
      }
      else
      {
        strncpy(m_line2, m_line1, m_width);
        strncpy(m_line1, msg, m_width);
        m_line1[m_width] = 0;
        m_line2[m_width] = 0;
      }
      m_offset = 0;
      m_fullLine[0] = 0;
      m_lcd->clear();
      m_lcd->setCursor(0, 1);
      m_lcd->print(m_line1);
      m_lcd->setCursor(0, 0);
      m_lcd->print(m_line2);
      m_nextScroll = millis() + 1000;
      m_clearTime = 0;
    };

    void message(String str) {
      message(str.c_str());
    }

    void message(int lever, bool good, String str) {
      String s("L:");
      s.concat(String(lever));
      if (good)
        s.concat(" OK");
      else
        s.concat(" bad");
      message(s);
      strncpy(m_fullLine, str.c_str(), 80);
      m_fullLine[80] = 0;
      message(str);
      if (good)
        m_clearTime = millis() + 10000;
      else
        m_clearTime = 0;
    }

    void ready()
    {
      String str = "V" + String(VERSION);
      message(str);
      message("Ready");
    }

    void loop() {
      if (millis() > m_nextScroll)
      {
        int fulllen = strlen(m_fullLine);
        if (fulllen > m_width)
        {
          m_offset++;
          if (m_offset + m_width >= fulllen)
            m_offset = 0;
          strncpy(m_line2, &m_fullLine[m_offset], m_width);
          m_lcd->setCursor(0, 1);
          m_lcd->print(m_line2);
        }
        m_nextScroll = millis() + 1000;
      }
      if (m_clearTime > 0 && millis() > m_clearTime)
      {
        m_clearTime = 0;
        ready();
      }
    }

  private:
    LiquidCrystal     *m_lcd;
    int     m_width;
    int     m_insert;
    char    *m_line1;
    char    *m_line2;
    int     m_offset;
    char    m_fullLine[81];
    unsigned long m_nextScroll;
    unsigned long m_clearTime;
};

LiquidCrystal lcd(36, 34, 32, 30, 28, 26);
Display *display;

/*
   Global state of the levers and bitmasks used to show locking rules
*/
unsigned long state[NUM_STATE_VARS];  // State of the levers
unsigned long masks[NUM_STATE_VARS][N_LEVERS];
unsigned long states[NUM_STATE_VARS][N_LEVERS];
unsigned long locked[NUM_STATE_VARS];
unsigned long pbmasks[NUM_STATE_VARS][N_LEVERS];    // Used if a seperate spreadsheet controls the putting back of levers
unsigned long pbstates[NUM_STATE_VARS][N_LEVERS];
bool pblogic;   // True if we are using two spreadsheets

int  buzLever = -1;         // The lever that is causing the buzzer to go off
unsigned int buzzerFreq;

/**
   Turn the buzzer off, also reset the variable that says which lever
   caused the buzzer to sound.
*/
void buzzerOff()
{
  noTone(BUZZER);
  buzLever = -1;
}

/**
   Turn the buzzer on and set a variable that is used to track the
   lever that is in error
*/
void buzzerOn(int lever)
{
  tone(BUZZER, buzzerFreq);
  buzLever = lever;
}

/**
   Check if it is legal to pull a lever off. This is merely a case of
   checking the bitmask and state for the current lever
*/
bool canPullOff(unsigned long lever, bool show)
{
  String blocker = "";
  bool result = true;
  for (int i = 0; i < NUM_STATE_VARS; i++)
  {
    if ((state[i] & masks[i][lever]) != states[i][lever])
    {
      result = false;
      for (int j = 0; j < BITS_PER_LONG; j++)
      {
        if (((state[i] & masks[i][lever]) & (1 << j)) != (states[i][lever] & (i << j)))
        {
          if (debugmask % DEBUGPULLOFF)
          {
            Serial.print("Lever ");
            Serial.print(i);
            Serial.print(" blocks ");
            Serial.print(lever);
            Serial.println(" from being pulled off.");
          }
          blocker.concat(String(j + 1));
          blocker.concat(" ");
        }
      }
    }
  }
  if (show)
  {
    if (!result)
    {
      display->message(lever + 1, false, blocker);
    }
    else
    {
      display->message(lever + 1, true, "off");
    }
  }
  return result;
}

/**
   Check if a lever can be put back in the frame. Here we need to check
   every other lever to see if that lever is currently pulled off and if
   it needs this lever to be pulled off in order for it to be pulled off.
*/
bool canPutBack(unsigned long lever, bool show)
{

  String blocker = "";
  bool result = true;
  if (pblogic)    // Deduce the rules rather than use a second spreadsheet
  {
    int maskind = lever / BITS_PER_LONG;
    unsigned long mask = (1 << (lever % BITS_PER_LONG));
    for (int i = 0; i < N_LEVERS; i++)
    {
      if (i == lever) // Don't check ourself
        continue;
      if (masks[maskind][i] & mask && (states[maskind][i] & mask))
      {
        // This lever depends on the one we want to put back being off
        if (state[i / BITS_PER_LONG] & (1 << (i % BITS_PER_LONG)))
        {
          // Lever is on, block putting this lever back
          result = false;

          if (debugmask % DEBUGPUTBACK)
          {
            Serial.print("Lever ");

            Serial.print(i);
            Serial.print(" prevents ");
            Serial.print(lever);
            Serial.println(" from being put back.");
          }
          blocker.concat(String(i + 1));
          blocker.concat(" ");
        }
      }
    }
  }
  else  // Use the second spread sheet to control the put back logic
  {
    for (int i = 0; i < NUM_STATE_VARS; i++)
    {
      if ((state[i] & pbmasks[i][lever]) != pbstates[i][lever])
      {
        result = false;
        if (debugmask % DEBUGPUTBACK)
        {
          Serial.print("Lever ");
          Serial.print(i);
          Serial.print(" prevents ");
          Serial.print(lever);
          Serial.println(" from being put back.");
        }
        blocker.concat(String(i + 1));
        blocker.concat(" ");
      }
    }
  }
  if (show)
  {
    if (!result)
    {
      display->message(lever + 1, false, blocker);
    }
    else
    {
      display->message(lever + 1, true, "back");
    }
  }
  return result;
}

/**
   Update the locked state for all the levers. We loop over every lever
   and if it is currently pulled off we check if it can be put back. If it
   is currently in the frame then we look if it can be pulled off. This
   updates the locked state of the lever and sends a cbus event if the locked
   state changes.
*/
void updateLocked()
{
  if (debugmask % DEBUGLOCK)
  {
    Serial.println();
    Serial.println("Update locked state");
  }
  for (int i = 0; i < N_LEVERS; i++)
  {
    unsigned long leverState = state[i / BITS_PER_LONG] & (1 << (i % BITS_PER_LONG));
    if (leverState && canPutBack(i, false))
    {
      if (locked[i / BITS_PER_LONG] & (1 << (i % BITS_PER_LONG)))
      {
        locked[i / BITS_PER_LONG] &= ~(1 << (i % BITS_PER_LONG));
        cbus.sendOffEvent(true, N_LEVERS + i);
        if (debugmask % DEBUGLOCK)
        {
          Serial.print("Unlock lever ");
          Serial.println(i);
        }
      }
    }
    else if (leverState == 0 && canPullOff(i, false))
    {
      if (locked[i / BITS_PER_LONG] & (1 << (i % BITS_PER_LONG)))
      {
        locked[i / BITS_PER_LONG] &= ~(1 << (i % BITS_PER_LONG));
        cbus.sendOffEvent(true, N_LEVERS + i);
        if (debugmask % DEBUGLOCK)
        {
          Serial.print("Unlock lever ");
          Serial.println(i);
        }
      }
    }
    else
    {
      locked[i / BITS_PER_LONG] |= (1 << (i % BITS_PER_LONG));
      cbus.sendOnEvent(true, N_LEVERS + i);
      if (debugmask % DEBUGLOCK)
      {
        Serial.print("Lock lever ");
        Serial.println(i);
      }
    }
  }
}

/**
   Pull a lever off, checks if it can be pulled off and either sends
   the CBUS event to the signal or point or sounds the buzzer if the
   lever can not be pulled off.
*/
void leverOn(unsigned long lever)
{
  if (buzLever == lever)
  {
    buzzerOff();
    display->ready();
    return;
  }
  if (buzLever != -1)
    return;
  if (debugmask % DEBUGREQUEST)
  {
    Serial.println();
    Serial.print("Lever ");
    Serial.print(lever);
    Serial.println(" On request");
    Serial.println("==================");
  }
  bool allowed = canPullOff(lever, true);
  if (allowed)
  {
    // allow lever
    cbus.sendOnEvent(true, lever);
    state[lever / BITS_PER_LONG] |= (1 << (lever % BITS_PER_LONG));
    if (debugmask % DEBUGREQUEST)
    {
      Serial.println("allowed");
    }
    updateLocked();
  }
  else
  {
    // disallow lever
    buzzerOn(lever);
    if (debugmask % DEBUGREQUEST)
    {
      Serial.println("disallowed");
    }
  }
  if (debugmask % DEBUGSTATE)
  {
    Serial.print("state ");
    Serial.print(state[0], HEX);
    Serial.print(" ");
    Serial.println(state[1], HEX);
  }
}

/**
   Put a lever back. Checks if it is legal to put the
   lever back and either sends the event to the point or
   singal or, if it is not legal it will sound the buzzer
*/
void leverOff(unsigned long lever)
{
  if (buzLever == lever)
  {
    buzzerOff();
    display->ready();
    return;
  }
  if (buzLever != -1)
    return;
  if (debugmask % DEBUGREQUEST)
  {
    Serial.println();
    Serial.print("Lever ");
    Serial.print(lever);
    Serial.println(" Off request");
    Serial.println("===================");
  }
  bool allow = canPutBack(lever, true);
  int maskind = lever / BITS_PER_LONG;
  unsigned long maskbit = (1 << (lever % BITS_PER_LONG));
  if (allow)
  {
    cbus.sendOffEvent(true, lever);
    state[maskind] &= ~maskbit;
    if (debugmask % DEBUGREQUEST)
    {
      Serial.println("allowed");
    }
    updateLocked();
  }
  else
  {
    buzzerOn(lever);
    if (debugmask % DEBUGREQUEST)
    {
      Serial.println("disallowed");
    }
  }
  if (debugmask % DEBUGSTATE)
  {
    Serial.print("state ");
    Serial.print(state[0], HEX);
    Serial.print(" ");
    Serial.println(state[1], HEX);
  }
}

/**
   Handle the CBUS events for the levers.
*/
void myUserFunc(Message * msg, MergCBUS * mcbus)
{
  /* getting standard on/off events */
  boolean onEvent;

  if (mcbus->eventMatch()) {
    onEvent = mcbus->isAccOn();
    int lever = mcbus->getEventVar(msg, 1); // Get first event variable

    if (onEvent)
    {
      leverOn(lever); // On event for the lever, this equates to pulling off the lever
    }
    else
    {
      leverOff(lever); // Off event for the lever, this equates to putting back the lever
    }
  }
}

void nodeVarHandler(int ind, int val)
{
  switch (ind)
  {
    case 3:
      buzzerFreq = val * 10;
      break;
    case 4:
      debugmask = val;
      break;
  }
}

void myUserFuncDCC(Message * msg, MergCBUS * mcbus) {
  //  Serial.print("DCC Code: ");
  //  Serial.println(msg->getOpc());
}

/**
   Send an audio code - this gives feedback if thre has been an issue
   starting up the system.
*/
void audioCode(int code)
{
  for (int i = 0; i < code; i++)
  {
    if (i & 1)
    {
      tone(BUZZER, 2500, 250);
      delay(300);
    }
    else
    {
      tone(BUZZER, 1000, 500);
      delay(500);
    }
  }
}

/**
   Fatal error has occured. Repeatably sound an audio code every 2 seconds
*/
void fatalError(int errCode)
{
  Serial.print("Fatal Error: ");
  Serial.println(errCode);

  // Send the error code out as an audio pattern
  while (true)
  {
    audioCode(errCode);
    delay(2000);
  }
}

/**
   Skip a number of rows in the spreadsheet
*/
void skipRows(File & dataFile, int rows)
{
#if DEBUG
  Serial.print("Skip rows ");
  Serial.println(rows);
#endif
  while (rows > 0 && dataFile.available() > 0)
  {
    unsigned char character = dataFile.read();
    if (character == '\n')
    {
      rows--;
    }
  }
}

/*
   Read a file from the SD card. Convert the CSV file into a set of bit values and msks
   The file shoudl be a CSV file with 1's and 0's in each cell

   Return false if the file could not be read
*/
bool readFile(const char *fileName, unsigned long masks[NUM_STATE_VARS][N_LEVERS], unsigned long states[NUM_STATE_VARS][N_LEVERS])
{
  if (! SD.exists(fileName))
    return false;
  File dataFile = SD.open(fileName);

  // if the file is available, write to it:
  if (dataFile)
  {
    skipRows(dataFile, cbus.getNodeVar(1));
    int col = 0, row = 0;
    int skipCols = cbus.getNodeVar(2);
#if DEBUG
    Serial.print("Skip Columns ");
    Serial.println(skipCols);
#endif
    int offset = col / (8 * sizeof(unsigned long));
    while (dataFile.available() > 0)
    {
      unsigned char character = dataFile.read();

      if (col - skipCols >= N_LEVERS)
      {
        audioCode(5);
        break;
      }
      unsigned long bitmask = 1 << ((col  - skipCols) % BITS_PER_LONG);
      if (character == '\n')
      {
        row++;
        col = 0;
        if (row >= N_LEVERS)
        {
          audioCode(4);
          break;
        }
      }
      else if (character == ',')
      {
        col++;  // BUGFIX Update offset here
        offset = (col - skipCols)  / BITS_PER_LONG;
      }
      else if (character == '0')
      {
        masks[offset][row] |= bitmask;
      }
      else if (character == '1')
      {
        masks[offset][row] |= bitmask;
        states[offset][row] |= bitmask;
      }
    }
    dataFile.close();
    return true;
  }
  else
  {
#if DEBUG
    Serial.print("Failed to open file ");
    Serial.println(fileName);
#endif
    display->message("Unable to open file");
    display->message(fileName);
    return false;
  }
}

/*
   The one time setup call that is called when the processor resets
*/
void setup()
{
#if DEBUG
  Serial.begin(115200);
  Serial.println("Setup");
#endif
  lcd.begin(8, 2);
  display = new Display(&lcd, 8);
  display->message("setup");
  //Configuration data for the node
  cbus.getNodeId()->setNodeName("INLK", 4);        //node name
  cbus.getNodeId()->setModuleId(MODULE_ID);            //module number
  cbus.getNodeId()->setManufacturerId(MANUFACTURER_ID);//merg code
  cbus.getNodeId()->setMinCodeVersion(MIN_CODE);       //Version 1
  cbus.getNodeId()->setMaxCodeVersion(MAX_CODE);
  cbus.getNodeId()->setProducerNode(true);
  cbus.getNodeId()->setConsumerNode(true);
  cbus.setPushButton(PUSH_BUTTON);//set the push button ports
  cbus.setStdNN(999); //standard node number

  //used to manually reset the node. while turning on keep the button pressed
  //this forces the node for slim mode with an empty memory for learned events and devices
  if (digitalRead(PUSH_BUTTON) == LOW) {
#if DEBUG
    Serial.println("Setup new memory");
#endif
    cbus.setUpNewMemory();
    cbus.saveNodeFlags();
    cbus.setNodeVariable(1, 1);
    cbus.setNodeVariable(2, 1);
    cbus.setNodeVariable(3, DEFAULT_BUZZER_FREQ);
    cbus.setNodeVariable(4, 0);
  }

  cbus.setLeds(GREEN_LED, YELLOW_LED); //set the led ports

  cbus.setUserHandlerFunction(&myUserFunc);//function that implements the node logic
  cbus.setDCCHandlerFunction(&myUserFuncDCC);
  cbus.setNodeVarHandlerFunction(&nodeVarHandler);
  cbus.initCanBus(CANPORT, CAN_125KBPS, MCP_16MHz, 20, 30);  //initiate the transport layer
  cbus.setFlimMode();

  pinMode(INTPIN, INPUT);

  buzzerFreq = cbus.getNodeVar(3) * 10;
  debugmask = cbus.getNodeVar(4);
  if (!SD.begin(SDCS)) {
#if DEBUG
    Serial.println("Failed to start SD card");
#endif
    display->message("Please check SD card");
    fatalError(1);
  }
  for (int j = 0; j < NUM_STATE_VARS; j++)
  {
    state[j] = 0;
    locked[j] = 0;
    for (int i = 0; i < N_LEVERS; i++)
    {
      states[j][i] = 0;
      masks[j][i] = 0;
    }
  }

  if (!readFile("/intlock.csv", masks, states))
  {
    fatalError(2);
  }

  pblogic = true;
  Serial.print("Put back logic will be ");
  // Read the putback spreadsheet if one exists
  if (readFile("/putback.csv", pbmasks, pbstates))
  {
    Serial.println("read from the second spreadsheet.");
  }
  else
  {
    Serial.println("derived from the pull off logic.");
  }
  display->message("Putback");
  display->message(pblogic ? "sheet" : "logic" );

#if DEBUG
  for (int i = 0; i < N_LEVERS; i++)
  {
    Serial.print("Row: ");
    Serial.print(i);
    Serial.print(" States: ");
    for (int ind = 0; ind < NUM_STATE_VARS; ind++)
      Serial.print(states[ind][i], HEX);
    Serial.print(" Mask: ");
    for (int ind = 0; ind < NUM_STATE_VARS; ind++)
      Serial.print(masks[ind][i], HEX);
    Serial.println();
  }
  Serial.println("Dump complete");
#endif
  updateLocked();
  audioCode(3);
  display->ready();

}

/*
   The loop routine that is continuously being called
*/
void loop()
{
  // Process the CBUS
  cbus.cbusRead();
  cbus.run(); //do all logic
  if (digitalRead(PUSH_BUTTON) == LOW)
  {
    cbus.dumpMemory();
  }
  display->loop();
}
