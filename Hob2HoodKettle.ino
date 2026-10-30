/* 
* Hob2Hood Kettle
* 1Pats@oto.lv March 2024
* Code is developed according to KISS principle
*
* Linking the electric kettle with the kitchen hood. 
* While the kettle is switched on, the ventilation switches on to pick up steam.
* As the hob and hood support the Hob2Hood interface, the hood is commanded by infrared signals broadcast by the Arduino UNO / KY-005 module, 
* which emits infrared signals. 
* Detection of the kettle is done based on the current sensor. In my case, ZMCT103C.
* The power wire is wired through the induction coil. The sensor module is glued with hot glue to the opposite side of the power socket where the kettle is plugged in.
* If the sensor detects power consumption in the appropriate range, it assumes this is a kettle, and UNO sends IR commands to the hood. 
* My kettle boils completely in about 4 minutes and 45 seconds and then switches off (if filled fully)
* As the fan has 4 levels, fan speed rises proportionally to heating time. Boost level switches on shortly before expected end, but 
* if the lid is unintentionally left open and the kettle is not switching off, it might help to prevent kitchen cabinets to spoil due to steam.
* During kettle's usage, the hood light is on.
* Code was developed with significant help from Arminjo/Armin Joachimsmeyer, one of IRremote library authors, 
* who presented an example how to communicate with Hob2Hood devices.
* Without him, I would not be able to finish this setup as receiving IR signals and sending them out differ significantly, 
* but the Hob2Hood protocol so far was not in the supported protocols list.
* For debugging purposes and an alternate communication way, remote's support is added.
* As my TV box is Sony and TV remote is always ready at hand, I decided to apply it- rarely used buttons from the teletext row
* (but make sure- as Bluetooth is an alternate communication channel for the Sony remote - it should be switched off)
* Teletext seems, now is in past...
* RED button - cancel everything immediately
* GREEN      - looping: "light on", "fan speed 1", "fan speed 2", "fan speed 3", "fan speed 4", "all off"
* YELLOW     - switch test mode on/off.  In test mode sends Hob2Hood commands to the hood every 5 seconds 
* Be aware of specifics of Sony remote - each infrared command is sent by Sony remote several times, but not as repeats. 
* You need to ignore duplicates -otherwise the second Sony's command interferes with Hob2Hood command and the result is undetermined...
* I widely use bitwise operations as they are very convenient. If you are not familiar with it, browse the internet. One example: 
* https://www.geeksforgeeks.org/bitwise-operators-in-c-cpp/
*
* Hobbyists could reuse code without limitation.
* Code contains a lot of debugging features; I intentionally left them in the code. 
* For the final version, all lines related to Serial might be commented out. Also, code used for Hob2Hood command confirmation
*/

#define DECODE_HASH                                                    // per Arminjo
#define ZMCT103C_PIN                       A0                          // GND,VCC-5V, 14    ZMCT103C  (yellow)
#define IR_SEND_PIN                        3                           // GND,VCC-5V, 3     KY-005    (blue)
#define IR_RECEIVE_PIN                     2                           // GND,VCC-5V, 2     KY-022    (green)

#include <IRremote.hpp>  

#define BOILING_TIME                        (5L*60L*1000L)              // in about 5 minutes my kettle boils up  
#define FAN1_TIME                           (BOILING_TIME/4L)           // time when start first fans level
#define FAN2_TIME                           (BOILING_TIME/2L)           // time when second level for fans  is turned on    
#define FAN3_TIME                           (FAN2_TIME + BOILING_TIME/4L) // third
#define FAN4_TIME                           (BOILING_TIME- 10L*1000L)   // boost level  - maximum fan speed
#define FAN_EXTRA                           (15L*1000L)                 // when kettle turns off, fan will be switched off after this delay                                          

// Current's levels for my kettle (no measurement units, just some values)  < ~50 is a level of unplugged socket
#define A_START                             50                          // smth plugged-in, light should be turned on
#define A_MIN                               550                         // in my case - electric kettle
#define A_MAX                               675                         // these are experimental values, and need to be adjusted for each particular device

// Status bits
#define LIGHT_IS_ON                         0x0001                      // set this bit, if light is on          0000000000000001
#define FAN_1                               0x0002                      // first fan level                       0000000000000010           
#define FAN_2                               0x0004                      // second fan level                      0000000000000100  
#define FAN_3                               0x0008                      // third fan level                       0000000000001000      
#define FAN_4                               0x0010                      // fourth fan level                      0000000000010000                     
#define MANUAL_MODE                         0x0020                      // if hood is commanded by SONY remote   0000000000100000
#define TEST_MODE                           0x0040                      // if test mode turned on                0000000001000000 
#define LIGHT_OFF                           0x0080                      // request to turn light off             0000000010000000  
#define REPEAT_COMMAND                      0x0100                      // used to ask for repetition
#define KETTLE_ON                           0x0200                      // current is in kettle's range, fan timing is running

// IR commands (code received) from Hob2Hood device or SONY remote
#define HOB_TO_HOOD_HASH_CODE_FAN_1         0xE3C01BE2
#define HOB_TO_HOOD_HASH_CODE_FAN_2         0xD051C301
#define HOB_TO_HOOD_HASH_CODE_FAN_3         0xC22FFFD7
#define HOB_TO_HOOD_HASH_CODE_FAN_4         0xB9121B29
#define HOB_TO_HOOD_HASH_CODE_FAN_OFF       0x55303A3
#define HOB_TO_HOOD_HASH_CODE_LIGHT_ON      0xE208293C
#define HOB_TO_HOOD_HASH_CODE_LIGHT_OFF     0x24ACF947
#define SONY_RED                            0xE23B4151                  // Sony remote buttons:
#define SONY_GREEN                          0xCE5541E4                  // Red, Green and Yellow 
#define SONY_YELLOW                         0xC0C52AFF                  // from teletext row    

#define REPEAT_PREVIOUS_COMMAND             0                           // Command failed -repeat it                                                                   

//  This part is completely taken from IRremote example ReceiceAndSendHob2Hood
#define HOB_TO_HOOD_UNIT_MICROS             725
#define H2H_1                               HOB_TO_HOOD_UNIT_MICROS     // 725
#define H2H_2                               (HOB_TO_HOOD_UNIT_MICROS*2) // 1450
#define H2H_3                               (HOB_TO_HOOD_UNIT_MICROS*3) // 2175
#define H2H_4                               (HOB_TO_HOOD_UNIT_MICROS*4) // 2900
#define H2H_5                               (HOB_TO_HOOD_UNIT_MICROS*5) // 3625

// Each infrared command has a specific sequence. The first entry is the length of the raw command
const uint16_t Fan1[]     PROGMEM { 15, H2H_2, H2H_2, H2H_1, H2H_2, H2H_3, H2H_2, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1, H2H_2, H2H_1, H2H_3, H2H_1 };
const uint16_t Fan2[]     PROGMEM { 9,  H2H_2, H2H_2, H2H_1, H2H_4, H2H_1, H2H_3, H2H_5, H2H_3, H2H_3 };
const uint16_t Fan3[]     PROGMEM { 9,  H2H_1, H2H_3, H2H_4, H2H_4, H2H_3, H2H_1, H2H_1, H2H_3, H2H_3 };
const uint16_t Fan4[]     PROGMEM { 13, H2H_2, H2H_3, H2H_2, H2H_1, H2H_2, H2H_3, H2H_2, H2H_2, H2H_1, H2H_3, H2H_1, H2H_1, H2H_2 };
const uint16_t FanOff[]   PROGMEM { 15, H2H_1, H2H_2, H2H_1, H2H_2, H2H_3, H2H_2, H2H_1, H2H_2, H2H_2, H2H_3, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1 };
const uint16_t LightOn[]  PROGMEM { 17, H2H_1, H2H_2, H2H_1, H2H_1, H2H_2, H2H_1, H2H_1, H2H_2, H2H_1, H2H_1, H2H_2, H2H_4, H2H_1, H2H_1, H2H_1, H2H_1, H2H_2 };
const uint16_t LightOff[] PROGMEM { 17, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1, H2H_1, H2H_1, H2H_3, H2H_1, H2H_1, H2H_1, H2H_2, H2H_1, H2H_2, H2H_1, H2H_1, H2H_1 };

// global variables
unsigned int uiStatus           = 0;                                  // system status
unsigned long ulKettleStartTime = 0;                                  // time, when kettle is turned on
unsigned long ulActionTime      = 0;                                  // time when last IR command is received or sent


void setup() {
  pinMode(ZMCT103C_PIN,INPUT);                                        // current's sensor pin
  Serial.begin(115200);                                               // for debugging
  Serial.println(F("\n\nStart Hob2HoodKetle " __FILE__ " from " __DATE__ "\r\nUsing IRremote library version " VERSION_IRREMOTE));
  Serial.print(F("Ready to read the current value. Module: ZMCT103C. Pin:")); Serial.println(ZMCT103C_PIN);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive IR signals. Module: KY-005. Pin:"));  Serial.println(IR_RECEIVE_PIN);
  IrSender.begin();                                                   // Start with IR_SEND_PIN as send pin and enable feedback LED at default feedback LED pin
  Serial.print(F("Ready to send Hob2Hood IR signals. Module: KY-022. Pin:"));  Serial.println(IR_SEND_PIN);
  Serial.println();
}

/* 
 *  Process test and manual modes, if such are used
 *  Detects kettle, command light and fan 
 */
#define TIME_FOR_CONFIRMATION          50
void loop() {
  if (bGetIRCommand()) {                                         // if smth received 
    ulActionTime = millis();                                     // memorize time
    return;                                                      // no other activities in this loop
  }
  else {                                                         // if no recognized IR commands   
     if (millis()-ulActionTime < TIME_FOR_CONFIRMATION) return;  // give time to process previous command
  } 

  if (!!(uiStatus & REPEAT_COMMAND)){                            // now we can check result- is previous command processed?
     Serial.println(F("Repeat previous command"));
     vSendHob2HoodCommand(REPEAT_PREVIOUS_COMMAND);              // signal was not received or received incorrectly, repeat it once more
     ulActionTime = millis();                                    // count down from this
     return;
  }

  if (uiStatus & TEST_MODE) {                                    // if test mode
     if (bTest()) ulActionTime = millis();                       // periodically sends commands to HOOD
     return;                                                     // and ignores rest
  }

  if (uiStatus & MANUAL_MODE) return;                            // manipulation with Sony remote. No automation  
  
  vAutomation();                                                 // detects kettle and command light and fan
}

/* 
* Activities according to current's sensor value-
* If smth is plugged in and if it fits in the kettle current range
* command light and fan accordingly
*/
void vAutomation(){
  unsigned int uiVal = uiReadZMCT103C();                       // read current's sensor 
  unsigned int uiAverage = uiGetAverageValue(uiVal);           // get average value from recent readings
  if (uiAverage > A_START)  vTurnLightOn();                    // first thing if smth plugged - turn light on                     
  else vTurnLightOff();                                        // and last - turn it off                                              
  if ((uiAverage >=  A_MIN) && (uiAverage <= A_MAX)) {
    if (!(uiStatus & KETTLE_ON)) {                             // kettle just switched on
      uiStatus |= KETTLE_ON;                                   // mark it
      ulKettleStartTime = millis();                            // fan levels are counted from this moment
    }
    vTurnHoodOn();
  }
  else  {
    uiStatus &= ~KETTLE_ON;                                    // kettle is off, next switching on starts timing again
    if (!(uiStatus & FAN_1)) return;                          // FAN might not be turned on so far- no reason to switch it off
    delay(FAN_EXTRA);                                          // delay to turn fan and light off (steam still is coming out from kettle) 
    vTurnHoodOff();                                            // switch fan off                                  
  }          
}


/*
* Turn hood light on
*/
void vTurnLightOn(){
    if (uiStatus & LIGHT_IS_ON) return;                                  // if lights is already on- nothing to do
    Serial.println(F("Turn Light on"));
    vSendHob2HoodCommand(LightOn);                                       // send infrared command to hood
}

/*
* Turn hood lights off
*/
void vTurnLightOff(){
    if (!(uiStatus & LIGHT_IS_ON)) return;                               // if lights is alraedy off- nothing to do                                           // reset light bit
    Serial.println(F("Turn Light off"));
    vSendHob2HoodCommand(LightOff);                                      // send infrared command                      
}

/* 
 * Turn Hood fan on
 * Increase fan speed by time  
 */
void vTurnHoodOn(){
   unsigned long ulNow = millis();
   if ((ulNow - ulKettleStartTime >= FAN1_TIME) && !(uiStatus & FAN_1)){  // time to turn fan on?
      if (!(uiStatus & LIGHT_IS_ON)) return;                              // light still is not on, wait for light
      Serial.println(F("Fan Speed 1"));
      vSendHob2HoodCommand(Fan1);                                         // send command to hood to switch fan at level 1 
      return;
   }
   if ((ulNow - ulKettleStartTime >= FAN2_TIME) && !(uiStatus & FAN_2)) { // time for level 2?                                                 // mark it as done 
      Serial.println(F("Fan Speed 2"));
      vSendHob2HoodCommand(Fan2);                                          // and send command to hood
      return;
   }
   if ((ulNow - ulKettleStartTime >= FAN3_TIME) && !(uiStatus & FAN_3)) {  // time to turn  level 3?
      Serial.println(F("Fan Speed 3"));
      vSendHob2HoodCommand(Fan3);
      return;
   }
   if ((ulNow - ulKettleStartTime >= FAN4_TIME) && !(uiStatus & FAN_4)) {  // Max fan speed. Few seconds before expected end                                                 // but it might help, steam continues to come 
      Serial.println(F("Fan Speed 4"));
      vSendHob2HoodCommand(Fan4);                                          // send this command to hood
      return;
   }
}

/*
 * Turn fan off independently of running speed
 */
void vTurnHoodOff(){
     if (!(uiStatus & FAN_1)) return;                                       // if it is in waiting state- do nothing
     Serial.println(F("Turn all Fans off"));
     vSendHob2HoodCommand(FanOff);                                          // Send infrared command to hood to switch all off
}


/*
 * Cancel all off - fan & light
 */
void vResetAll(){
     //if (uiStatus == 0) return;               // nothing to cancel 
     //Serial.print(F("Status=")); Serial.println(uiStatus, BIN);
     Serial.println(F("Cancel All"));
     uiStatus |= LIGHT_OFF;                // light-off command will be sent out afterwards
     vSendHob2HoodCommand(FanOff); 
}


/* 
 * Send command to hood according to Hob2Hood standard
 * It is based on ReceiveAndSendHob2Hood example by Armin Joachimsmeyer
 * If arg is REPEAT_PREVIOUS_COMMAND -previous command is broadcasted one more time
 */ 
#define MAX_REPETITION_TIMES 5                          // if command is repeated this times, reset all
void vSendHob2HoodCommand(const uint16_t *pCommand){
     static const uint16_t *pCommandPrevious = nullptr; // remember the command being sent out         
     static unsigned  int uiRepeatCount = 0;            // count repetition to avoid forever cycle
     if ((pCommand == REPEAT_PREVIOUS_COMMAND) && (pCommandPrevious != nullptr)){        // repetition is asked
        pCommand = pCommandPrevious;                    // previous commnd
        uiRepeatCount++;                                // count repetition
        if (uiRepeatCount >= MAX_REPETITION_TIMES) {    // smth goes wrong, reset all
          pCommand = FanOff;                            // turn fan off
          uiRepeatCount = 0;                            // and assume all is canceled
          uiStatus = 0;                                 // reset status as well
          uiStatus |= LIGHT_OFF;                        // +request: turn light off (after reset, otherwise it is lost)
          Serial.print(F("*** Too much repetitions, reset all ***"));                                
        }
        else Serial.print(F(" Repeat previous command ")); // print it out to serial - show repetition 
      }
      else uiRepeatCount = 0;                           // new command requested, no repetition
      // only listed commands are supported
      if ((pCommand != Fan1    ) &&         
          (pCommand != Fan2    ) &&           
          (pCommand != Fan3    ) &&          
          (pCommand != Fan4    ) &&          
          (pCommand != FanOff  ) &&        
          (pCommand != LightOn ) &&       
          (pCommand != LightOff)){                                                            
            pCommandPrevious = nullptr;                                         // nothing to memorize
            Serial.println(F("*** Attempt to send unlisted command ***"));
            return;                                                             // no reason to send it out
      }   
      uiStatus |= REPEAT_COMMAND;                                                  // repeat for case, if broadcasting failed. Sender sets this flag, receiver removes it  
      pCommandPrevious = pCommand;                                                 // memorize command 
      uint16_t tLengthOfRawCommand = pgm_read_word(pCommand);                      // length is the 1. word in array
      const uint16_t *tAddressOfRawCommandSequence = pCommand + 1;                 // raw sequence starts at the 2. word of array
      IrSender.sendRaw_P(tAddressOfRawCommandSequence, tLengthOfRawCommand, 38);   // send it out
      // next two commands are taken from SendAndReceive example. No clue what they do, but it helps to avoid unexpected behaviour
      IrReceiver.restartAfterSend();                                               // looks like it helps synchronize sequence of events  
      delay(10);                                                                   // wait for the receiver state machine to detect the end of a protocol
      ulActionTime = millis();                                                     // give receiver time to confirm, before command is repeated
      

    /* 
     // print out sequence for debugging purposes
    Serial.print(F("RAW Data(")); Serial.print(tLengthOfRawCommand); Serial.print(F("): "));
    for (int i = 0; i < tLengthOfRawCommand; i++){
        uint16_t iw = pgm_read_word(tAddressOfRawCommandSequence+i);
        Serial.print(iw); 
        if (i < tLengthOfRawCommand -1); Serial.print(",");
    }
    Serial.println();
     */
}

/*
 * Receive and process IR commands coming from Infrared Sender module or Sony TV remote
 * For Hood commands, it is confirmation - signal is broadcast correctly 
 * Sony TV:
 * RED -    cancel/turn all off 
 * GREEN -  loop:  LIGHT ON->FAN1->FAN2->FAN3->FAN4->ALL OFF->starts from beginning...
 * YELLOW - turns test mode on/of. In test mode- sends commands to the hood every 5 seconds 
 */ 
bool bGetIRCommand(){
   if (IrReceiver.decode()) {                                  // if smth received
        IrReceiver.resume();                                   // early enable receiving of the next IR frame 
        //IrReceiver.printIRResultShort(&Serial);              // print it out
        switch (IrReceiver.decodedIRData.decodedRawData) {     // check the received data and perform actions according to the received command
           case HOB_TO_HOOD_HASH_CODE_FAN_OFF:                 // received HOB TO HOOD command
              if (!(uiStatus & REPEAT_COMMAND)) return false;  // if command was send by hob, not by UNO
              uiStatus &= ~(FAN_1|FAN_2|FAN_3|FAN_4|REPEAT_COMMAND); // reset fan bits
              Serial.println(F("Confirm:FANS OFF")); 
              if (uiStatus & LIGHT_OFF){                       // second command is requested                                                              
                  uiStatus &= ~LIGHT_OFF;                      // reset request bit
                  delay(1000);                                 // otherwise second command comes out too quickly  
                  Serial.println(F("Request LIGHT OFF")); 
                  vSendHob2HoodCommand(LightOff);              // send infrared command  to turn light off
                  return true;                                 // IR signal broadcasted 
              }
              return true;
           case HOB_TO_HOOD_HASH_CODE_FAN_1:                   // Confirmation about turning fan to level 1
              if (!(uiStatus & REPEAT_COMMAND)) return false; // if command was send by hob, not by UNO
              uiStatus |= FAN_1;                               // set confirmation bit
              uiStatus &= ~REPEAT_COMMAND;                     // this is done, no reason to do it once more
              Serial.println(F("Confirm:FAN 1 ON"));
              return true;
          case HOB_TO_HOOD_HASH_CODE_FAN_2:
              if (!(uiStatus & REPEAT_COMMAND)) return false; // if command was send by hob, not by UNO
              uiStatus |= FAN_2;
              uiStatus &= ~REPEAT_COMMAND;                 
              Serial.println(F("Confirm:FAN 2 ON"));
              return true; 
          case HOB_TO_HOOD_HASH_CODE_FAN_3:   
              if (!(uiStatus & REPEAT_COMMAND)) return false;  // if command was send by hob, not by UNO              
              uiStatus |= FAN_3;
              uiStatus &= ~REPEAT_COMMAND; 
              Serial.println(F("Confirm:FAN 3 ON"));
              return true;
          case HOB_TO_HOOD_HASH_CODE_FAN_4:   
              if (!(uiStatus & REPEAT_COMMAND)) return false;  // if command was send by hob, not by UNO
              uiStatus |= FAN_4; 
              uiStatus &= ~REPEAT_COMMAND;      
              Serial.println(F("Confirm:FAN 4 ON"));
              return true;
         case HOB_TO_HOOD_HASH_CODE_LIGHT_ON:
              if (!(uiStatus & REPEAT_COMMAND)) return false;  // if command was send by hob, not by UNO
              uiStatus |= LIGHT_IS_ON;
              uiStatus &= ~REPEAT_COMMAND;               
              Serial.println(F("Confirm:LIGHT ON"));
              return true;
         case HOB_TO_HOOD_HASH_CODE_LIGHT_OFF:
              if (!(uiStatus & REPEAT_COMMAND)) return false;  // if command was send by hob, not by UNO
              uiStatus &= ~LIGHT_IS_ON;  
              uiStatus &= ~REPEAT_COMMAND;             
              Serial.println(F("Confirm:LIGHT OFF"));
              return true;
         case SONY_RED:                                        // red button on Sony remote is pressed                                   
              Serial.println(F("Remote:SONY RED - Cancel All"));              
              vDrainRemote();                                  // skip all duplicates  
              uiStatus &= ~(MANUAL_MODE|TEST_MODE);            // stop testing or manual mode      
              vResetAll();                                     // turn all off
              return true;
         case SONY_GREEN:                                      // Loop: light,fan1,2,3,4, all off,...
              Serial.println(F("Remote:SONY GREEN - Manual Mode ON")); 
              vDrainRemote();                                  // remote repeats command at least 3 times (but not mark it as repetitons) 
              uiStatus &= ~(TEST_MODE|REPEAT_COMMAND);         // recall test mode
              uiStatus |= MANUAL_MODE;                         // actual status at this point might be variable
              if (!(uiStatus & LIGHT_IS_ON)) {                 // start with light
                  Serial.print(F("Ask:Light on. "));
                  vSendHob2HoodCommand(LightOn);               // send infrared command to hood
                  return true;                                 // processed
              }
              if (!(uiStatus & FAN_1)) {                       // if fan1 is not turned on, do it now
                 Serial.print(F("Ask:FAN1 on. "));
                 vSendHob2HoodCommand(Fan1);                   // send infrared commnd to hood
                 return true;                                  // processed
              }
              if (!(uiStatus & FAN_2)) {
                 Serial.print(F("Ask:FAN2 on. "));
                 vSendHob2HoodCommand(Fan2);                   // send infrared commnd to hood
                 return true;
              }
              if (!(uiStatus & FAN_3)) {
                 Serial.print(F("Ask:FAN3 on. "));
                 vSendHob2HoodCommand(Fan3);                   // send infrared commnd to hood
                 return true;
              }  
              if (!(uiStatus & FAN_4)) { 
                 Serial.print(F("Ask:FAN4 on. "));
                 vSendHob2HoodCommand(Fan4);                   // send infrared commnd to hood
                 return true;
              }
              if (uiStatus & (FAN_1|FAN_2|FAN_3|FAN_4)) {      // fan at full speed
                 Serial.print(F("Ask all off. "));
                 vResetAll();                                  // turn all off
                 return true;
              }
              break;
            case SONY_YELLOW:                                  // turn test mode on/off 
               uiStatus &= ~(MANUAL_MODE|REPEAT_COMMAND);      // actual status at this point might be variable
               Serial.print(F("Remote:SONY YELLOW - Test Mode "));
               vDrainRemote();                                 // skip all duplicates   
               if (uiStatus & TEST_MODE) {                     // test modde is on?
                  Serial.println(F("Off"));
                  uiStatus = 0;                                // set test mode off, reset all (before vResetAll - it sets its own request bits)
                  vResetAll();                                 // turn fan & light off - might be in any state at this point
               }
               else {  
                  uiStatus = TEST_MODE;                        // set test mode on and nothing else 
                  vResetAll();                                 // turn fan & light off - might be in any state at this point
                  Serial.println(F("On"));  
               }
               return true;
            default:
               Serial.println(F("*** IR command is not recognized! ***"));
               IrReceiver.printIRResultShort(&Serial);         // see raw code
               vDrainRemote();                                 // most likelly this is a Sony remote, skip all duplicates
               return false;                                   // ignore this
        }
   }
   return false;                                               // nothing detetcted
}

/*
* On button press, Sony remote sends command several times (and not as repetitions)
* As command is already received, no necessity for such duplications
* Do fake readings to skip extra signals
*/
#define WAIT_FOR_REMOTE 100                                    // if no new infrared command in ... miliseconds- 
void vDrainRemote(){                                           // - assume broadcasting from remote is completed
  unsigned long ulTime = millis();                             // looks like interval might be shorter  
  while(millis() - ulTime < WAIT_FOR_REMOTE){                  // if no new signl in this time- assume - now all duplicates are ignored
    if (IrReceiver.decode()){                                  // check remote
        IrReceiver.resume();                                   // early enable receiving of the next IR frame 
        //Serial.print(F("Diff:")); Serial.print(millis()-ulTime); Serial.print(F(" "));   // see miliseconds between signals
        //IrReceiver.printIRResultShort(&Serial);              // print it out to see what is ignored
        ulTime = millis();                                     // start it again
    }
  }  
}

/*
 * Function to test communication with hood
 * just to send out Hob2Hod commands sequentially each 5 seconds
 * returns true if command was sent out, false otherwise
 */
#define SEND_TIMEOUT                  5000                  // 5 seconds 
bool bTest(){
  static int iCount = 0;                                    // count sendings
  static unsigned long ulTime = millis();                   // time when command is sent
  if (millis()- ulTime < SEND_TIMEOUT) return false;        // once in 5 seconds
  ulTime = millis();                                        // memorize time now
  int i = iCount++%7;                                       // which command?
  //Serial.print(i); Serial.print(": ");
  switch(i){                                                // switch to next command sequentially
    case 0:   if (!(uiStatus & LIGHT_IS_ON)) vSendHob2HoodCommand(LightOn);   break;         //   -->
    case 1:   if (!(uiStatus & FAN_1))       vSendHob2HoodCommand(Fan1);      break;         //     |
    case 2:   if (!(uiStatus & FAN_2))       vSendHob2HoodCommand(Fan2);      break;         //     |   
    case 3:   if (!(uiStatus & FAN_3))       vSendHob2HoodCommand(Fan3);      break;         //     |  
    case 4:   if (!(uiStatus & FAN_4))       vSendHob2HoodCommand(Fan4);      break;         //     |  
    case 5:   if ((uiStatus & FAN_1))        vSendHob2HoodCommand(FanOff);    break;         //     |   
    default:  if ((uiStatus & LIGHT_IS_ON))  vSendHob2HoodCommand(LightOff);  break;         //  <--  
  }
  return true;
}

/* 
 * Accumulate values and calculate the average from recent readings, 
 * but no more than recent AVERAGE_READINGS_COUNT instances 
 * returns average value 
 */
#define AVERAGE_READINGS_COUNT                         64                     // number of instances for averaging sensor readings
unsigned int uiGetAverageValue(unsigned int uiValue){
    static int aiLastReadings[AVERAGE_READINGS_COUNT];                        // array where to accumulate readings
    static int iLastReadingsCounter       = 0;                                // count of fulfilled values in the array
    static int iLastReadingsIndex         = 0;                                // actual index to replace actual value
    static long lTotal                    = 0L;                               // sum of values 
    if ((iLastReadingsCounter == 0) && (iLastReadingsIndex == 0)) memset((void *)aiLastReadings,0,AVERAGE_READINGS_COUNT*sizeof(int));    // set all cells to 0 
    lTotal -= long(aiLastReadings[iLastReadingsIndex]);                       // previous value has to be replaced (0 initially)
    aiLastReadings[iLastReadingsIndex] = uiValue;                             // store new value 
    lTotal += long(uiValue);                                                  // add new value instead of previous one
    if (++iLastReadingsIndex >= AVERAGE_READINGS_COUNT) iLastReadingsIndex = 0;  // if end of array, start filling from beginning
    if (iLastReadingsCounter < AVERAGE_READINGS_COUNT) iLastReadingsCounter++;   // next one 
    unsigned int uiAverage = (unsigned int) (lTotal/long(iLastReadingsCounter)); // calculate average value
    return uiAverage;                                                         // return average value
}
 
/*
 * Read current's sensor
 * Return delta between min and max instead of value, as it suffers less from fluctuations
 * Approach was suggested in Arduino.cc forum by Wawa Brattain
 */
unsigned int uiReadZMCT103C(){
  unsigned int uiMaxPeak = 0;                                            // reset
  unsigned int uiMinPeak = 1023;                                         // reset
  unsigned long ulStartMillis = millis();                                // mark
  while (millis() - ulStartMillis < 16) {                                // 15ms is ~3/4 of 50Hz  
    unsigned int uiRawValue = analogRead(ZMCT103C_PIN);                  // sample continuously
    if (uiRawValue > uiMaxPeak) uiMaxPeak = uiRawValue;                  // getting max value
    if (uiRawValue < uiMinPeak) uiMinPeak = uiRawValue;                  // getting min value
  }
  return (uiMaxPeak - uiMinPeak);                                        // return delta
}

