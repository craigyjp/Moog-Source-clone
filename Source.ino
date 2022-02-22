/*
  Source MUX - Firmware Rev 1.3

  Includes code by:
    Dave Benn - Handling MUXs, a few other bits and original inspiration  https://www.notesandvolts.com/2019/01/teensy-synth-part-10-hardware.html

  Arduino IDE
  Tools Settings:
  Board: "Teensy3.6"
  USB Type: "Serial + MIDI + Audio"
  CPU Speed: "180"
  Optimize: "Fastest"

  Additional libraries:
    Agileware CircularBuffer available in Arduino libraries manager
    Replacement files are in the Modified Libraries folder and need to be placed in the teensy Audio folder.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <MIDI.h>
#include <USBHost_t36.h>
#include "MidiCC.h"
#include "Constants.h"
#include "Parameters.h"
#include "PatchMgr.h"
#include "HWControls.h"
#include "EepromMgr.h"
#include "Settings.h"
#include <ShiftRegister74HC595.h>
#include <RoxMux.h>

#define PARAMETER 0 //The main page for displaying the current patch and control (parameter) changes
#define RECALL 1 //Patches list
#define SAVE 2 //Save patch page
#define REINITIALISE 3 // Reinitialise message
#define PATCH 4 // Show current patch bypassing PARAMETER
#define PATCHNAMING 5 // Patch naming page
#define DELETE 6 //Delete patch page
#define DELETEMSG 7 //Delete patch message page
#define SETTINGS 8 //Settings page
#define SETTINGSVALUE 9 //Settings page
unsigned int state = PARAMETER;
#include "ST7735Display.h"

//
// Mux values
//
int DelayForSH3 = 10;
int patchNo = 0;
unsigned long buttonDebounce = 0;
boolean cardStatus = false;

//
//USB HOST MIDI Class Compliant
//
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);

//MIDI 5 Pin DIN
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI); //RX - Pin 0

//
// MIDI to CV conversion
//

bool notes[88] = {0};
int8_t noteOrder[80] = {0}, orderIndx = {0};
int noteMsg;
unsigned long trigTimer = 0;
#define NOTE_SF 51.57f // This value can be tuned if CV output isn't exactly 1V/octave
#define trigTimeout 20
int transpose = 0;
int realoctave = 0;
float previousMillis = millis(); //For MIDI Clk Sync
int count = 0;//For MIDI Clk Sync
long earliestTime = millis(); //For voice allocation - initialise to now
int modulation;
int pitchbend;
float bend = 0;
int8_t d2, i;
bool S1, S2;

//
//Shift Register setup
//

ShiftRegister74HC595<6> srpanel(6, 7, 9);
ShiftRegister74HC595<3> srp(36, 39, 37);
#define MUX_TOTAL 6
Rox74HC165 <MUX_TOTAL> mux;
#define PIN_DATA  35 // pin 9 on 74HC165 (DATA)
#define PIN_LOAD  34 // pin 1 on 74HC165 (LOAD)
#define PIN_CLK   33 // pin 2 on 74HC165 (CLK))

//
// Start setup
//

void setup()
{
  SPI.begin();
  setupDisplay();
  setUpSettings();
  setupHardware();

  cardStatus = SD.begin(BUILTIN_SDCARD);
  if (cardStatus)
  {
    Serial.println("SD card is connected");
    //Get patch numbers and names from SD card
    loadPatches();
    if (patches.size() == 0)
    {
      //save an initialised patch to SD card
      savePatch("1", INITPATCH);
      loadPatches();
    }
  }
  else
  {
    Serial.println("SD card is not connected or unusable");
    reinitialiseToPanel();
    showPatchPage("No SD", "conn'd / usable");
  }

  //Read MIDI Channel from EEPROM
  midiChannel = getMIDIChannel();
  Serial.println("MIDI Ch:" + String(midiChannel) + " (0 is Omni On)");

  //USB HOST MIDI Class Compliant
  delay(200); //Wait to turn on USB Host
  myusb.begin();
  midi1.setHandleControlChange(myControlChange);
  midi1.setHandlePitchChange(myPitchBend);
  midi1.setHandleProgramChange(myProgramChange);
  midi1.setHandleNoteOff(myNoteOff);
  midi1.setHandleNoteOn(myNoteOn);
  Serial.println("USB HOST MIDI Class Compliant Listening");

  //USB Client MIDI
  usbMIDI.setHandleControlChange(myControlChange);
  usbMIDI.setHandlePitchChange(myPitchBend);
  usbMIDI.setHandleProgramChange(myProgramChange);
  usbMIDI.setHandleNoteOff(myNoteOff);
  usbMIDI.setHandleNoteOn(myNoteOn);
  Serial.println("USB Client MIDI Listening");

  //MIDI 5 Pin DIN
  MIDI.begin();
  MIDI.setHandleControlChange(myControlChange);
  MIDI.setHandlePitchBend(myPitchBend);
  MIDI.setHandleProgramChange(myProgramChange);
  MIDI.setHandleNoteOn(myNoteOn);
  MIDI.setHandleNoteOff(myNoteOff);

  Serial.println("MIDI In DIN Listening");

  //Read Key Tracking from EEPROM, this can be set individually by each patch.
  keyMode = getKeyMode();

  //Read Pitch Bend Range from EEPROM, this can be set individually by each patch.
  pitchBendRange = getPitchBendRange();

  //Read Mod Wheel Depth from EEPROM, this can be set individually by each patch.
  modWheelDepth = getModWheelDepth();

  //Read Encoder Direction from EEPROM
  encCW = getEncoderDir();
  level1 = 1;
  level2 = 0;
  srpanel.set(28, HIGH);
  srpanel.set(29, LOW);
  patchNo = getLastPatch();
  recallPatch(patchNo); //Load first patch
  //showPatchNumberButton();

  mux.begin(PIN_DATA, PIN_LOAD, PIN_CLK);


  //  reinitialiseToPanel();
}

void setVoltage(int dacpin, bool channel, bool gain, unsigned int mV)
{
  int command = channel ? 0x9000 : 0x1000;

  command |= gain ? 0x0000 : 0x2000;
  command |= (mV & 0x0FFF);

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dacpin, LOW);
  SPI.transfer(command >> 8);
  SPI.transfer(command & 0xFF);
  digitalWrite(dacpin, HIGH);
  SPI.endTransaction();
}

void showPatchNumberButton()
{
  srpanel.set(32, LOW);
  srpanel.set(33, LOW);
  srpanel.set(34, LOW);
  srpanel.set(35, LOW);
  srpanel.set(36, LOW);
  srpanel.set(37, LOW);
  srpanel.set(38, LOW);
  srpanel.set(39, LOW);
  srpanel.set(40, LOW);
  srpanel.set(41, LOW);
  srpanel.set(42, LOW);
  srpanel.set(43, LOW);
  srpanel.set(44, LOW);
  srpanel.set(45, LOW);
  srpanel.set(46, LOW);
  srpanel.set(47, LOW);
  switch (patchNo)
  {
    case 1:
      srpanel.set(32, HIGH);
      break;
    case 2:
      srpanel.set(33, HIGH);
      break;
    case 3:
      srpanel.set(34, HIGH);
      break;
    case 4:
      srpanel.set(35, HIGH);
      break;
    case 5:
      srpanel.set(36, HIGH);
      break;
    case 6:
      srpanel.set(37, HIGH);
      break;
    case 7:
      srpanel.set(38, HIGH);
      break;
    case 8:
      srpanel.set(39, HIGH);
      break;
    case 9:
      srpanel.set(40, HIGH);
      break;
    case 10:
      srpanel.set(41, HIGH);
      break;
    case 11:
      srpanel.set(42, HIGH);
      break;
    case 12:
      srpanel.set(43, HIGH);
      break;
    case 13:
      srpanel.set(44, HIGH);
      break;
    case 14:
      srpanel.set(45, HIGH);
      break;
    case 15:
      srpanel.set(46, HIGH);
      break;
    case 16:
      srpanel.set(47, HIGH);
      break;
  }
}

void myPitchBend(byte channel, int bend) {
  if ((midiChannel == channel) || (channel == 0 )) {
    // Pitch bend output from 0 to 1023 mV.  Left shift d2 by 4 to scale from 0 to 2047.
    // With DAC gain = 1X, this will yield a range from 0 to 1023 mV.  Additional amplification
    // after DAC will rescale to -1 to +1V.
    d2 = midi1.getData2(); // d2 from 0 to 127, mid point = 64;
    pitchbend = bend;
    //setVoltage(PITCH_DAC, PITCH_AB, 1, int((bend / 8) + 1024)); // DAC7, channel 0, gain = 1X
  }
}

void myControlChange(byte channel, byte number, byte value) {
  if ((midiChannel == channel || channel == 0)) {
    d2 = MIDI.getData2();
    // CC range from 0 to 2047 mV  Left shift d2 by 5 to scale from 0 to 2047,
    // and choose gain = 2X
    modulation = value;
    //setVoltage(CC_DAC, CC_AB, 1, value << 4); // DAC7, channel 1, gain = 1X
  }
}

void commandTopNote()
{
  int topNote = 0;
  bool noteActive = false;
  for (int i = 0; i < 88; i++)
  {
    if (notes[i]) {
      topNote = i;
      noteActive = true;
    }
  }
  if (noteActive)
    commandNote(topNote);
  else // All notes are off, turn off gate
    digitalWrite(GATE_NOTE1, LOW);
    gatepulse = 0;
}

void commandBottomNote()
{
  int bottomNote = 0;
  bool noteActive = false;
  for (int i = 87; i >= 0; i--)
  {
    if (notes[i]) {
      bottomNote = i;
      noteActive = true;
    }
  }
  if (noteActive)
    commandNote(bottomNote);
  else // All notes are off, turn off gate
    digitalWrite(GATE_NOTE1, LOW);
    gatepulse = 0;
}

void commandLastNote()
{
  int8_t noteIndx;
  for (int i = 0; i < 80; i++) {
    noteIndx = noteOrder[ mod(orderIndx - i, 80) ];
    if (notes[noteIndx]) {
      commandNote(noteIndx);
      return;
    }
  }
  digitalWrite(GATE_NOTE1, LOW); // All notes are off
  gatepulse = 0;
}

void commandNote(int noteMsg) {
  unsigned int CV = (unsigned int) ((float) (noteMsg + transpose + realoctave) * NOTE_SF * 1.0 + 0.5);

  analogWrite(A21, CV);

  if ( gatepulse == 0 && single == 1 )
  {
  digitalWrite(TRIG_NOTE1, HIGH);
  }
  if  ( multi == 1 )
  {
    digitalWrite(TRIG_NOTE1, HIGH);
  }
  digitalWrite(GATE_NOTE1, HIGH);
  gatepulse = 1;
  trigTimer = millis();
  while (millis() < trigTimer + trigTimeout) {
    // wait 50 milliseconds
  }
  digitalWrite(TRIG_NOTE1, LOW);
}

void myNoteOn(byte channel, byte note, byte velocity)
{
  if (keyMode == 4)
  {
    S1 = 1;
    S2 = 1;
  }
  if (keyMode == 5)
  {
    S1 = 0;
    S2 = 1;
  }
  if (keyMode == 6)
  {
    S1 = 0;
    S2 = 0;
  }
  noteMsg = note;

  if (velocity == 0)  {
    notes[noteMsg] = false;
  }
  else {
    notes[noteMsg] = true;
  }
  unsigned int velCV = ((unsigned int) ((float) velocity) * 22);

  analogWrite(A22, velCV);

  if (S1 && S2) { // Highest note priority
    commandTopNote();
  }
  else if (!S1 && S2) { // Lowest note priority
    commandBottomNote();
  }
  else { // Last note priority
    if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
      orderIndx = (orderIndx + 1) % 40;
      noteOrder[orderIndx] = noteMsg;
    }
    commandLastNote();
  }
}

void myNoteOff(byte channel, byte note, byte velocity) {
  if (keyMode == 4)
  {
    S1 = 1;
    S2 = 1;
  }
  if (keyMode == 5)
  {
    S1 = 0;
    S2 = 1;
  }
  if (keyMode == 6)
  {
    S1 = 0;
    S2 = 0;
  }
  noteMsg = note;

  if (velocity == 0)  {
    notes[noteMsg] = false;
  }
  else {
    notes[noteMsg] = true;
  }

  // Pins NP_SEL1 and NP_SEL2 indictate note priority
  unsigned int velmV = ((unsigned int) ((float) velocity) * 1.25);
  setVoltage(DAC_NOTE1, 1, 1, velmV << 4 );
  if (S1 && S2) { // Highest note priority
    commandTopNote();
  }
  else if (!S1 && S2) { // Lowest note priority
    commandBottomNote();
  }
  else { // Last note priority
    if (notes[noteMsg]) {  // If note is on and using last note priority, add to ordered list
      orderIndx = (orderIndx + 1) % 40;
      noteOrder[orderIndx] = noteMsg;
    }
    commandLastNote();
  }
}


void allNotesOff() {
  digitalWrite(GATE_NOTE1, LOW);
  gatepulse = 0;
}

void firstNoteOff() {
  digitalWrite(GATE_NOTE1, LOW);
  gatepulse = 0;
}

void updateosc1_32()
{
  if (osc1_32 == 1)
  {
    showCurrentParameterPage("Osc1 Footage", "32 Foot");
    osc1foot = 0;
    srpanel.set(0, HIGH);  // LED on
    srpanel.set(1, LOW);  // LED off
    srpanel.set(2, LOW);  // LED off
  }
}

void updateosc1_16()
{
  if (osc1_16 == 1)
  {
    showCurrentParameterPage("Osc1 Footage", "16 Foot");
    osc1foot = 2012;
    srpanel.set(0, LOW);  // LED on
    srpanel.set(1, HIGH);  // LED off
    srpanel.set(2, LOW);  // LED off
  }
}

void updateosc1_8()
{
  if (osc1_8 == 1)
  {
    showCurrentParameterPage("Osc1 Footage", "8 Foot");
    osc1foot = 4024;
    srpanel.set(0, LOW);  // LED on
    srpanel.set(1, LOW);  // LED off
    srpanel.set(2, HIGH);  // LED off
  }
}

void updateosc1_saw()
{
  if (osc1_saw == 1)
  {
    showCurrentParameterPage("Osc1 Wave", "Sawtooth");
    srpanel.set(3, HIGH);
    srpanel.set(4, LOW);
    srpanel.set(5, LOW);
    srp.set(3, LOW);
    srp.set(4, LOW);
  }
}

void updateosc1_tri()
{
  if (osc1_tri == 1)
  {
    showCurrentParameterPage("Osc1 Wave", "Triangle");
    srpanel.set(3, LOW);  // LED on
    srpanel.set(4, HIGH);  // LED off
    srpanel.set(5, LOW);  // LED off
    srp.set(3, HIGH);
    srp.set(4, LOW);
  }
}

void updateosc1_pulse()
{
  if (osc1_pulse == 1)
  {
    showCurrentParameterPage("Osc1 Wave", "Pulse");
    srpanel.set(3, LOW);  // LED on
    srpanel.set(4, LOW);  // LED off
    srpanel.set(5, HIGH);  // LED off
    srp.set(3, LOW);
    srp.set(4, HIGH);
  }
}

void updatesingle()
{
  if (single == 1)
  {
    showCurrentParameterPage("Single Trigger", "On");
    srpanel.set(8, HIGH);  // LED on
    srpanel.set(9, LOW);  // LED off
  }
}

void updatemulti()
{
  if (multi == 1)
  {
    showCurrentParameterPage("Multi Trigger", "On");
    srpanel.set(9, HIGH);  // LED on
    srpanel.set(8, LOW);  // LED off
  }
}

void updatelfoTriangle()
{
  if (lfoTriangle == 1)
  {
    showCurrentParameterPage("LFO Waveform", "Triangle");
    LfoWave = 400;
    srpanel.set(10, HIGH);  // LED on
    srpanel.set(11, LOW);  // LED off
  }
}

void updatelfoSquare()
{
  if (lfoSquare == 1)
  {
    showCurrentParameterPage("LFO Waveform", "Square");
    LfoWave = 300;
    srpanel.set(10, LOW);  // LED on
    srpanel.set(11, HIGH);  // LED off
  }
}

void updatesyncOff()
{
  if (syncOff == 1)
  {
    showCurrentParameterPage("Oscillator Sync", "Off");
    srp.set(11, LOW); // pb osc1 on
    srp.set(12, LOW); // pb osc2 on
    srp.set(13, LOW); // sync off
    srp.set(14, LOW); // soft sync off
    srp.set(15, LOW); // hard sync off
    srpanel.set(12, HIGH);
    srpanel.set(13, LOW);
  }
}

void updatesyncOn()
{
  if (syncOn == 1)
  {
    showCurrentParameterPage("Oscillator Sync", "On");
    srp.set(11, LOW); // pb osc1 off
    srp.set(12, HIGH);  // pb osc2 on
    srp.set(13, HIGH); // sync on
    srp.set(14, HIGH);  // soft sync on
    srp.set(15, HIGH);  // hard sync off
    srpanel.set(12, LOW);
    srpanel.set(13, HIGH);
  }
}

void updateoctave0()
{
  if (octave0 == 1)
  {
    showCurrentParameterPage("KBD Octave", "0");
    srp.set(2, LOW);  // LED on
    srpanel.set(14, HIGH);
    srpanel.set(15, LOW);
  }
}

void updateoctave1()
{
  if (octave1 == 1)
  {
    showCurrentParameterPage("KBD Octave", "+1");
    srp.set(2, HIGH);  // LED on
    srpanel.set(14, LOW);
    srpanel.set(15, HIGH);
  }
}

void updatekbOff()
{
  if (kbOff == 1)
  {
    showCurrentParameterPage("KBD Tracking", "Off");
    srp.set(7, LOW);
    srp.set(8, LOW);
    srpanel.set(16, HIGH);
    srpanel.set(17, LOW);
    srpanel.set(18, LOW);
  }
}

void updatekbHalf()
{
  if (kbHalf == 1)
  {
    showCurrentParameterPage("KBD Tracking", "Half");
    srp.set(7, LOW);
    srp.set(8, HIGH);
    srpanel.set(16, LOW);
    srpanel.set(17, HIGH);
    srpanel.set(18, LOW);
  }
}

void updatekbFull()
{
  if (kbFull == 1)
  {
    showCurrentParameterPage("KBD Tracking", "Full");
    srp.set(7, HIGH);
    srp.set(8, HIGH);
    srpanel.set(16, LOW);
    srpanel.set(17, LOW);
    srpanel.set(18, HIGH);
  }
}

void updateosc2_32()
{
  if (osc2_32 == 1)
  {
    showCurrentParameterPage("Osc2 Footage", "32 Foot");
    osc2foot = 0;
    srpanel.set(26, HIGH);
    srpanel.set(27, LOW);
    srpanel.set(19, LOW);
  }
}

void updateosc2_16()
{
  if (osc2_16 == 1)
  {
    showCurrentParameterPage("Osc2 Footage", "16 Foot");
    osc2foot = 2024;
    srpanel.set(26, LOW);
    srpanel.set(27, HIGH);
    srpanel.set(19, LOW);
  }
}

void updateosc2_8()
{
  if (osc2_8 == 1)
  {
    showCurrentParameterPage("Osc2 Footage", "8 Foot");
    osc2foot = 4048;
    srpanel.set(26, LOW);
    srpanel.set(27, LOW);
    srpanel.set(19, HIGH);
  }
}

void updateosc2_saw()
{
  if (osc2_saw == 1)
  {
    showCurrentParameterPage("Osc2 Wave", "Sawtooth");
    srpanel.set(20, HIGH);
    srpanel.set(21, LOW);
    srpanel.set(22, LOW);
    srp.set(5, LOW);
    srp.set(6, LOW);
  }
}

void updateosc2_tri()
{
  if (osc2_tri == 1)
  {
    showCurrentParameterPage("Osc2 Wave", "Triangle");
    srpanel.set(20, LOW);
    srpanel.set(21, HIGH);
    srpanel.set(22, LOW);
    srp.set(5, HIGH);
    srp.set(6, LOW);
  }
}

void updateosc2_pulse()
{
  if (osc2_pulse == 1)
  {
    showCurrentParameterPage("Osc2 Wave", "On");
    srpanel.set(20, LOW);
    srpanel.set(21, LOW);
    srpanel.set(22, HIGH);
    srp.set(5, LOW);
    srp.set(6, HIGH);
  }
}

void updatelfoOscOff()
{
  if (lfoOscOff == 1)
  {
    showCurrentParameterPage("LFO to Osc", "Off");
    srp.set(9, LOW);
    srpanel.set(24, HIGH);
    srpanel.set(25, LOW);
  }
}

void updatelfoOscOn()
{
  if (lfoOscOn == 1)
  {
    showCurrentParameterPage("LFO to Osc", "On");
    srp.set(9, HIGH);
    srpanel.set(24, LOW);
    srpanel.set(25, HIGH);
  }
}

void updatelfoVCFOff()
{
  if (lfoVCFOff == 1)
  {
    showCurrentParameterPage("LFO to VCF", "Off");
    srp.set(10, LOW);
    srpanel.set(30, HIGH);
    srpanel.set(31, LOW);
  }
}

void updatelfoVCFOn()
{
  if (lfoVCFOn == 1)
  {
    showCurrentParameterPage("LFO to VCF", "On");
    srp.set(10, HIGH);
    srpanel.set(30, LOW);
    srpanel.set(31, HIGH);
  }
}

void updatelevel1()
{
  if (level1 == 1)
    level2 = 0;
  {
    showCurrentParameterPage("Level 1", "Selected");
    srpanel.set(28, HIGH);
    srpanel.set(29, LOW);

    showPatchNumberButton();
  }
}

void updatelevel2()
{
  if (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "Selected");
    srpanel.set(28, LOW);
    srpanel.set(29, HIGH);

    srpanel.set(32, LOW);
    srpanel.set(33, LOW);
    srpanel.set(34, LOW);
    srpanel.set(35, LOW);
    srpanel.set(36, LOW);
    srpanel.set(37, LOW);
    srpanel.set(38, LOW);
    srpanel.set(39, LOW);
    srpanel.set(40, LOW);
    srpanel.set(41, LOW);
    srpanel.set(42, LOW);
    srpanel.set(43, LOW);
    srpanel.set(44, LOW);
    srpanel.set(45, LOW);
    srpanel.set(46, LOW);
    srpanel.set(47, LOW);
    level1 = 0;

    updateshvco();
    updateshvcf();
    updatevcfVelocity();
    updatevcaVelocity();
    updatevcfLoop();
    updatevcaLoop();
    updatevcfLinear();
    updatevcaLinear();
  }
}

void updateshvco()
{
  if  (shvco  == 1)
  {
    srp.set(16, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(41, HIGH);
    }
  }
  else
  {
    srp.set(16, LOW);
    srpanel.set(41, LOW);
  }
}

void updateshvcf()
{
  if  (shvcf  == 1)
  {
    srp.set(17, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(42, HIGH);
    }
  }
    else
  {
    srp.set(17, LOW);
    srpanel.set(42, LOW);
  }
}

void updatevcfVelocity()
{
  if  (vcfVelocity  == 1)
  {
    srp.set(1, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(34, HIGH);
    }
  }
    else
  {
    srp.set(1, LOW);
    srpanel.set(34, LOW);
  }
}

void updatevcaVelocity()
{
  if  (vcaVelocity  == 1)
  {
    srp.set(22, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(35, HIGH);
    }
  }
    else
  {
    srp.set(22, LOW);
    srpanel.set(35, LOW);
  }
}

void updatevcfLoop()
{
  if  (vcfLoop  == 1)
  {
    srp.set(18, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(36, HIGH);
    }
  }
    else
  {
    srp.set(18, LOW);
    srpanel.set(36, LOW);
  }
}

void updatevcaLoop()
{
  if  (vcaLoop  == 1)
  {
    srp.set(19, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(37, HIGH);
    }
  }
    else
  {
    srp.set(19, LOW);
    srpanel.set(37, LOW);
  }
}

void updatevcfLinear()
{
  if  (vcfLinear  == 1)
  {
    srp.set(20, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(38, HIGH);
    }
  }
    else
  {
    srp.set(20, LOW);
    srpanel.set(38, LOW);
  }
}

void updatevcaLinear()
{
  if  (vcaLinear  == 1)
  {
    srp.set(21, HIGH);
    if  (level2  == 1)
    {
      srpanel.set(39, HIGH);
    }
  }
    else
  {
    srp.set(21, LOW);
    srpanel.set(39, LOW);
  }
}

void updatebutton1()
{
  if  (level2 == 1 && button1switch == 1)
  {
    showCurrentParameterPage("Bend Depth", pitchBendRange);

    srpanel.set(32, HIGH);

  }
  if  (level2 == 1 && button1switch == 0)
  {
    showCurrentParameterPage("Bend Depth", "Not Selected");

    srpanel.set(32, LOW);

  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "1");
    patchNo = 1;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton2()
{
  if  (level2 == 1 && button2switch == 1)
  {
    showCurrentParameterPage("Mod Depth", modWheelDepth);

    srpanel.set(33, HIGH);

  }
  if  (level2 == 1 && button2switch == 0)
  {
    showCurrentParameterPage("Mod Depth", "Not Selected");

    srpanel.set(33, LOW);

  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "2");
    patchNo = 2;
    recallPatch(patchNo);
    showPatchNumberButton();
  }

}

void updatebutton3()
{
  if  (level2 == 1 && button3switch == 1)
  {
    showCurrentParameterPage("VCF Vel", "On");
    vcfVelocity = 1;
    srpanel.set(34, HIGH);
    srp.set(1, HIGH);
  }
  if  (level2 == 1 && button3switch == 0)
  {
    showCurrentParameterPage("VCF Vel", "Off ");
    vcfVelocity = 0;
    srpanel.set(34, LOW);
    srp.set(1, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "3");
    patchNo = 3;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton4()
{
  if  (level2 == 1 && button4switch == 1)
  {
    showCurrentParameterPage("VCA Vel", "On");
    vcaVelocity = 1;
    srpanel.set(35, HIGH);
    srp.set(22, HIGH);
  }
  if  (level2 == 1 && button4switch == 0)
  {
    showCurrentParameterPage("VCA Vel", "Off ");
    vcaVelocity = 0;
    srpanel.set(35, LOW);
    srp.set(22, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "4");
    patchNo = 4;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton5()
{
  if  (level2 == 1 && button5switch == 1)
  {
    showCurrentParameterPage("VCF Loop", "On");
    vcfLoop = 1;
    srpanel.set(36, HIGH);
    srp.set(18, HIGH);
  }
  if  (level2 == 1 && button5switch == 0)
  {
    showCurrentParameterPage("VCF Loop", "Off ");
    vcfLoop = 0;
    srpanel.set(36, LOW);
    srp.set(18, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "5");
    patchNo = 5;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton6()
{
  if  (level2 == 1 && button6switch == 1)
  {
    showCurrentParameterPage("VCA Loop", "On");
    vcaLoop = 1;
    srpanel.set(37, HIGH);
    srp.set(19, HIGH);
  }
  if  (level2 == 1 && button6switch == 0)
  {
    showCurrentParameterPage("VCA Loop", "Off ");
    vcaLoop = 0;
    srpanel.set(37, LOW);
    srp.set(19, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "6");
    patchNo = 6;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton7()
{
  if  (level2 == 1 && button7switch == 1)
  {
    showCurrentParameterPage("VCF Lin EG", "On");
    vcfLinear = 1;
    srpanel.set(38, HIGH);
    srp.set(20, HIGH);
  }
  if  (level2 == 1 && button7switch == 0)
  {
    showCurrentParameterPage("VCF Lin EG", "Off ");
    vcfLinear = 0;
    srpanel.set(38, LOW);
    srp.set(20, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "7");
    patchNo = 7;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton8()
{
  if  (level2 == 1 && button8switch == 1)
  {
    showCurrentParameterPage("VCA Lin EG", "On");
    vcaLinear = 1;
    srpanel.set(39, HIGH);
    srp.set(21, HIGH);
  }
  if  (level2 == 1 && button8switch == 0)
  {
    showCurrentParameterPage("VCA Lin EG", "Off ");
    vcaLinear = 0;
    srpanel.set(39, LOW);
    srp.set(21, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "8");
    patchNo = 8;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton9()
{
  if  (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "Arpeggio");
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "9");
    patchNo = 9;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton10()
{
  if  (level2 == 1 && button10switch == 1)
  {
    showCurrentParameterPage("Sample & Hold", "To VCO & VCF ");
    shvco = 1;
    shvcf = 0;
    srpanel.set(41, HIGH);
    srpanel.set(42, LOW);
    srp.set(16, HIGH);
    srp.set(17, HIGH);
  }
  if  (level2 == 1 && button10switch == 0)
  {
    showCurrentParameterPage("Sample & Hold", "Off ");
    shvco = 0;
    shvcf = 0;
    srpanel.set(41, LOW);
    srp.set(16, LOW);
    srp.set(17, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "10");
    patchNo = 10;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton11()
{
  if  (level2 == 1 && button11switch == 1)
  {
    showCurrentParameterPage("Sample & Hold", "To VCF ");
    shvco = 0;
    shvcf = 1;
    srpanel.set(41, LOW);
    srpanel.set(42, HIGH);
    srp.set(16, LOW);
    srp.set(17, HIGH);
  }
  if  (level2 == 1 && button11switch == 0)
  {
    showCurrentParameterPage("Sample & Hold", "Off ");
    shvco = 0;
    shvcf = 0;
    srpanel.set(42, LOW);
    srp.set(16, LOW);
    srp.set(17, LOW);
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "11");
    patchNo = 11;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton12()
{
  if  (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "Auto Trigger");
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "12");
    patchNo = 12;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton13()
{
  if  (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "Ext Trigger");
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "13");
    patchNo = 13;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton14()
{
  if  (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "No Function");
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "14");
    patchNo = 14;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton15()
{
  if  (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "No Function");
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "15");
    patchNo = 15;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}

void updatebutton16()
{
  if  (level2 == 1)
  {
    showCurrentParameterPage("Level 2", "No Function");
  }
  if (level1 == 1)
  {
    showCurrentParameterPage("Recall Patch", "16");
    patchNo = 16;
    recallPatch(patchNo);
    showPatchNumberButton();
  }
}


void updatefilterRes()
{
  showCurrentParameterPage("Resonance", int(filterResstr));
}

void updatefilterLevel()
{
  showCurrentParameterPage("Contour Amt", int(filterLevelstr));
}

void updateglide()
{
  showCurrentParameterPage("Glide", int(glidestr));
}

void updateNoiseLevel()
{
  showCurrentParameterPage("Noise Level", int(noiseLevelstr));
}

void updateFilterCutoff()
{
  showCurrentParameterPage("Cutoff", String(filterCutoffstr) + " Hz");
}

void updateLfoRate()
{
  showCurrentParameterPage("LFO Rate", String(LfoRatestr) + " Hz");
}

void updatepwLFO()
{
  showCurrentParameterPage("PWM Rate", int(pwLFOstr));
}

void updateosc2level()
{
  showCurrentParameterPage("OSC2 Level", int(osc2levelstr));
}

void updateosc1level()
{
  showCurrentParameterPage("OSC1 Level", int(osc1levelstr));
}

void updateosc2interval()
{
  showCurrentParameterPage("OSC2 Interval", int(osc2intervalstr));
}

void updateosc1PW()
{
  showCurrentParameterPage("OSC1 PW", String(osc1PWstr) + " %");
}

void updateosc2PW()
{
  showCurrentParameterPage("OSC2 PW", String(osc2PWstr) + " %");
}

void updateosc1PWM()
{
  showCurrentParameterPage("OSC1 PWM", int(osc1PWMstr));
}

void updateosc2PWM()
{
  showCurrentParameterPage("OSC2 PWM", int(osc2PWMstr));
}

void updateampAttack()
{
  if (ampAttackstr < 1000)
  {
    showCurrentParameterPage("Amp Attack", String(int(ampAttackstr)) + " ms", AMP_ENV);
  }
  else
  {
    showCurrentParameterPage("Amp Attack", String(ampAttackstr * 0.001) + " s", AMP_ENV);
  }
}

void updateampDecay()
{
  if (ampDecaystr < 1000)
  {
    showCurrentParameterPage("Amp Decay", String(int(ampDecaystr)) + " ms", AMP_ENV);
  }
  else
  {
    showCurrentParameterPage("Amp Decay", String(ampDecaystr * 0.001) + " s", AMP_ENV);
  }
}

void updateampSustain()
{
  showCurrentParameterPage("Amp Sustain", String(ampSustainstr), AMP_ENV);
}

void updateampRelease()
{
  if (ampReleasestr < 1000)
  {
    showCurrentParameterPage("Amp Release", String(int(ampReleasestr)) + " ms", AMP_ENV);
  }
  else
  {
    showCurrentParameterPage("Amp Release", String(ampReleasestr * 0.001) + " s", AMP_ENV);
  }
}

void updatefilterAttack()
{
  if (filterAttackstr < 1000)
  {
    showCurrentParameterPage("Filter Attack", String(int(filterAttackstr)) + " ms", AMP_ENV);
  }
  else
  {
    showCurrentParameterPage("Filter Attack", String(filterAttackstr * 0.001) + " s", AMP_ENV);
  }
}

void updatefilterDecay()
{
  if (filterDecaystr < 1000)
  {
    showCurrentParameterPage("Filter Decay", String(int(filterDecaystr)) + " ms", AMP_ENV);
  }
  else
  {
    showCurrentParameterPage("Filter Decay", String(filterDecaystr * 0.001) + " s", AMP_ENV);
  }
}

void updatefilterSustain()
{
  showCurrentParameterPage("Filter Sustain", String(filterSustainstr), AMP_ENV);
}

void updatefilterRelease()
{
  if (filterReleasestr < 1000)
  {
    showCurrentParameterPage("Filter Release", String(int(filterReleasestr)) + " ms", AMP_ENV);
  }
  else
  {
    showCurrentParameterPage("Filter Release", String(filterReleasestr * 0.001) + " s", AMP_ENV);
  }
}


void updatePatchname()
{
  showPatchPage(String(patchNo), patchName);
}

void myControlChange(byte channel, byte control, int value)
{

  //  Serial.println("MIDI: " + String(control) + " : " + String(value));
  switch (control)
  {
    case CCglide:
      glide = value;
      glidestr = value / 8;
      updateglide();
      break;

    case CCosc1_32:
      osc1_32 = 1;
      osc1_16 = 0;
      osc1_8 = 0;
      updateosc1_32();
      break;

    case CCosc1_16:
      osc1_32 = 0;
      osc1_16 = 1;
      osc1_8 = 0;
      updateosc1_16();
      break;

    case CCosc1_8:
      osc1_32 = 0;
      osc1_16 = 0;
      osc1_8 = 1;
      updateosc1_8();
      break;

    case CCosc1_saw:
      osc1_saw = 1;
      osc1_tri = 0;
      osc1_pulse = 0;
      updateosc1_saw();
      break;

    case CCosc1_tri:
      osc1_saw = 0;
      osc1_tri = 1;
      osc1_pulse = 0;
      updateosc1_tri();
      break;

    case CCosc1_pulse:
      osc1_saw = 0;
      osc1_tri = 0;
      osc1_pulse = 1;
      updateosc1_pulse();
      break;

    case CCsingle:
      single = 1;
      multi = 0;
      updatesingle();
      break;

    case CCmulti:
      multi = 1;
      single = 0;
      updatemulti();
      break;

    case CClfoTriangle:
      lfoTriangle = 1;
      lfoSquare = 0;
      updatelfoTriangle();
      break;

    case CClfoSquare:
      lfoSquare = 1;
      lfoTriangle = 0;
      updatelfoSquare();
      break;

    case CCsyncOff:
      syncOff = 1;
      syncOn = 0;
      updatesyncOff();
      break;

    case CCsyncOn:
      syncOn = 1;
      syncOff = 0;
      updatesyncOn();
      break;

    case CCoctave0:
      octave0 = 1;
      octave1 = 0;
      updateoctave0();
      break;

    case CCoctave1:
      octave0 = 0;
      octave1 = 1;
      updateoctave1();
      break;

    case CCkbOff:
      kbOff = 1;
      kbHalf = 0;
      kbFull = 0;
      updatekbOff();
      break;

    case CCkbHalf:
      kbOff = 0;
      kbHalf = 1;
      kbFull = 0;
      updatekbHalf();
      break;

    case CCkbFull:
      kbOff = 0;
      kbHalf = 0;
      kbFull = 1;
      updatekbFull();
      break;

    case CCosc2_32:
      osc2_32 = 1;
      osc2_16 = 0;
      osc2_8 = 0;
      updateosc2_32();
      break;

    case CCosc2_16:
      osc2_32 = 0;
      osc2_16 = 1;
      osc2_8 = 0;
      updateosc2_16();
      break;

    case CCosc2_8:
      osc2_32 = 0;
      osc2_16 = 0;
      osc2_8 = 1;
      updateosc2_8();
      break;

    case CCosc2_saw:
      osc2_saw = 1;
      osc2_tri = 0;
      osc2_pulse = 0;
      updateosc2_saw();
      break;

    case CCosc2_tri:
      osc2_saw = 0;
      osc2_tri = 1;
      osc2_pulse = 0;
      updateosc2_tri();
      break;

    case CCosc2_pulse:
      osc2_saw = 0;
      osc2_tri = 0;
      osc2_pulse = 1;
      updateosc2_pulse();
      break;

    case CClfoOscOff:
      lfoOscOff = 1;
      lfoOscOn = 0;
      updatelfoOscOff();
      break;

    case CClfoOscOn:
      lfoOscOff = 0;
      lfoOscOn = 1;
      updatelfoOscOn();
      break;

    case CClfoVCFOff:
      lfoVCFOff = 1;
      lfoVCFOn = 0;
      updatelfoVCFOff();
      break;

    case CClfoVCFOn:
      lfoVCFOff = 0;
      lfoVCFOn = 1;
      updatelfoVCFOn();
      break;

    case CClevel1:
      level1 = 1;
      level2 = 0;
      updatelevel1();
      break;

    case CClevel2:
      level2 = 1;
      level1 = 0;
      updatelevel2();
      break;

    case CCbutton1:
      button1 = 1;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton1();
      break;

    case CCbutton2:
      button1 = 0;
      button2 = 1;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton2();
      break;

    case CCbutton3:
      button1 = 0;
      button2 = 0;
      button3 = 1;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton3();
      break;

    case CCbutton4:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 1;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton4();
      break;

    case CCbutton5:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 1;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton5();
      break;

    case CCbutton6:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 1;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton6();
      break;

    case CCbutton7:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 1;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton7();
      break;

    case CCbutton8:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 1;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton8();
      break;

    case CCbutton9:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 1;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton9();
      break;

    case CCbutton10:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 1;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton10();
      break;

    case CCbutton11:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 1;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton11();
      break;

    case CCbutton12:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 1;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton12();
      break;

    case CCbutton13:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 1;
      button14 = 0;
      button15 = 0;
      button16 = 0;
      updatebutton13();
      break;

    case CCbutton14:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 1;
      button15 = 0;
      button16 = 0;
      updatebutton14();
      break;

    case CCbutton15:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 1;
      button16 = 0;
      updatebutton15();
      break;

    case CCbutton16:
      button1 = 0;
      button2 = 0;
      button3 = 0;
      button4 = 0;
      button5 = 0;
      button6 = 0;
      button7 = 0;
      button8 = 0;
      button9 = 0;
      button10 = 0;
      button11 = 0;
      button12 = 0;
      button13 = 0;
      button14 = 0;
      button15 = 0;
      button16 = 1;
      updatebutton16();
      break;

    case CCnoiseLevel:
      noiseLevel = value;
      noiseLevelstr = int(value / 8);
      updateNoiseLevel();
      break;

    case CCfilterCutoff:
      filterCutoff = value;
      filterCutoffstr = FILTERCUTOFF[value / 8];
      updateFilterCutoff();
      break;

    case CCfilterRes:
      filterRes = value;
      filterResstr = int(value / 8);
      updatefilterRes();
      break;

    case CCfilterlevel:
      filterLevel = value;
      filterLevelstr = int(value / 8);
      updatefilterLevel();
      break;

    case CCosc1PW:
      osc1PW = value;
      osc1PWstr = PULSEWIDTH[value / 8];
      updateosc1PW();
      break;

    case CCosc2PW:
      osc2PW = value;
      osc2PWstr = PULSEWIDTH[value / 8];
      updateosc2PW();
      break;

    case CCosc1PWM:
      osc1PWM = value;
      osc1PWMstr = int(value / 8);
      updateosc1PWM();
      break;

    case CCosc2PWM:
      osc2PWM = value;
      osc2PWMstr = int(value / 8);
      updateosc2PWM();
      break;

    case CCLfoRate:
      LfoRatestr = LFOTEMPOINV[value / 8]; // for display
      LfoRate = value;
      updateLfoRate();
      break;

    case CCpwLFO:
      pwLFO = value;
      pwLFOstr = value / 8; // for display
      updatepwLFO();
      break;

    case CCosc2level:
      osc2level = value;
      osc2levelstr = value / 8; // for display
      updateosc2level();
      break;

    case CCosc1level:
      osc1level = value;
      osc1levelstr = value / 8; // for display
      updateosc1level();
      break;

    case CCosc2interval:
      osc2intervalstr = INTERVAL[value / 8 ];
      osc2interval = value;
      updateosc2interval();
      break;

    case CCampAttack:
      ampAttack = value;
      ampAttackstr = ENVTIMES[value / 8];
      updateampAttack();
      break;

    case CCampDecay:
      ampDecay = value;
      ampDecaystr = ENVTIMES[value / 8];
      updateampDecay();
      break;

    case CCampSustain:
      ampSustain = value;
      ampSustainstr = LINEAR_FILTERMIXERSTR[value / 8];
      updateampSustain();
      break;

    case CCampRelease:
      ampRelease = value;
      ampReleasestr = ENVTIMES[value / 8];
      updateampRelease();
      break;

    case CCfilterAttack:
      filterAttack = value;
      filterAttackstr = ENVTIMES[value / 8];
      updatefilterAttack();
      break;

    case CCfilterDecay:
      filterDecay = value;
      filterDecaystr = ENVTIMES[value / 8];
      updatefilterDecay();
      break;

    case CCfilterSustain:
      filterSustain = value;
      filterSustainstr = LINEAR_FILTERMIXERSTR[value / 8];
      updatefilterSustain();
      break;

    case CCfilterRelease:
      filterRelease = value;
      filterReleasestr = ENVTIMES[value / 8];
      updatefilterRelease();
      break;

    case CCallnotesoff:
      allNotesOff();
      break;
  }
}

void myProgramChange(byte channel, byte program)
{
  state = PATCH;
  patchNo = program + 1;
  recallPatch(patchNo);
  Serial.print("MIDI Pgm Change:");
  Serial.println(patchNo);
  state = PARAMETER;
}

void recallPatch(int patchNo)
{
  allNotesOff();
  File patchFile = SD.open(String(patchNo).c_str());
  if (!patchFile)
  {
    Serial.println("File not found");
  }
  else
  {
    String data[NO_OF_PARAMS]; //Array of data read in
    recallPatchData(patchFile, data);
    setCurrentPatchData(data);
    patchFile.close();
    storeLastPatch(patchNo);
    showPatchNumberButton();
    updatelevel2();
  }
}

void setCurrentPatchData(String data[])
{
  patchName = data[0];
  noiseLevel = data[1].toFloat();
  glide = data[2].toFloat();
  osc1_32 = data[3].toInt();
  osc1_16 = data[4].toInt();
  osc1_8 = data[5].toInt();
  osc1_saw = data[6].toInt();
  osc1_tri = data[7].toInt();
  osc1_pulse = data[8].toInt();
  osc2_32 = data[9].toInt();
  osc2_16 = data[10].toInt();
  osc2_8 = data[11].toInt();
  osc2_saw = data[12].toInt();
  osc2_tri = data[13].toInt();
  osc2_pulse = data[14].toInt();
  single = data[15].toInt();
  multi = data[16].toInt();
  lfoTriangle = data[17].toInt();
  lfoSquare = data[18].toInt();
  lfoOscOff = data[19].toInt();
  lfoOscOn = data[20].toInt();
  lfoVCFOff  = data[21].toInt();
  lfoVCFOn  = data[22].toInt();
  syncOff  = data[23].toInt();
  syncOn  = data[24].toInt();
  kbOff  = data[25].toInt();
  kbHalf  = data[26].toInt();
  kbFull  = data[27].toInt();
  LfoRate = data[28].toFloat();
  pwLFO = data[29].toFloat();
  osc1level = data[30].toFloat();
  osc2level = data[31].toFloat();
  osc1PW = data[32].toFloat();
  osc2PW = data[33].toFloat();
  osc1PWM = data[34].toFloat();
  osc2PWM = data[35].toFloat();
  ampAttack = data[36].toFloat();
  ampDecay = data[37].toFloat();
  ampSustain = data[38].toFloat();
  ampRelease = data[39].toFloat();
  osc2interval = data[40].toFloat();
  filterAttack = data[41].toFloat();
  filterDecay = data[42].toFloat();
  filterSustain = data[43].toFloat();
  filterRelease = data[44].toFloat();
  filterRes = data[45].toFloat();
  filterCutoff = data[46].toFloat();
  filterLevel = data[47].toFloat();
  osc1foot = data[48].toFloat();
  osc2foot = data[49].toFloat();
  octave0 = data[50].toInt();
  octave1 = data[51].toInt();
  shvco = data[52].toInt();
  shvcf = data[53].toInt();
  vcfVelocity = data[54].toInt();
  vcaVelocity = data[55].toInt();
  vcfLoop = data[56].toInt();
  vcaLoop = data[57].toInt();
  vcfLinear = data[58].toInt();
  vcaLinear = data[59].toInt();
  keyMode = data[60].toInt();
  modWheelDepth = data[61].toInt();
  pitchBendRange = data[62].toInt();

  //Switches
  updateosc1_32();
  updateosc1_16();
  updateosc1_8();
  updateosc1_saw();
  updateosc1_tri();
  updateosc1_pulse();
  updatesingle();
  updatemulti();
  updatelfoTriangle();
  updatelfoSquare();
  updatesyncOff();
  updatesyncOn();
  updateoctave0();
  updateoctave1();
  updatekbOff();
  updatekbHalf();
  updatekbFull();
  updateosc2_32();
  updateosc2_16();
  updateosc2_8();
  updateosc2_saw();
  updateosc2_tri();
  updateosc2_pulse();
  updatelfoOscOff();
  updatelfoOscOn();
  updatelfoVCFOff();
  updatelfoVCFOn();
  updateshvco();
  updateshvcf();
  updatevcfVelocity();
  updatevcaVelocity();
  updatevcfLoop();
  updatevcaLoop();
  updatevcfLinear();
  updatevcaLinear();


  //Patchname
  updatePatchname();

  Serial.print("Set Patch: ");
  Serial.println(patchName);
}

String getCurrentPatchData()
{
  return patchName + "," + String(noiseLevel) + "," + String(glide) + "," + String(osc1_32) + "," + String(osc1_16) + "," + String(osc1_8) + "," + String(osc1_saw) + "," + String(osc1_tri) + "," + String(osc1_pulse) + "," +
         String(osc2_32) + "," + String(osc2_16) + "," + String(osc2_8) + "," + String(osc2_saw) + "," + String(osc2_tri) + "," + String(osc2_pulse) + "," + String(single) + "," + String(multi) + "," +
         String(lfoTriangle) + "," + String(lfoSquare) + "," + String(lfoOscOff) + "," + String(lfoOscOn) + "," + String(lfoVCFOff) + "," + String(lfoVCFOn) + "," + String(syncOff) + "," + String(syncOn) + "," +
         String(kbOff) + "," + String(kbHalf) + "," + String(kbFull) + "," + String(LfoRate) + "," + String(pwLFO) + "," + String(osc1level) + "," + String(osc2level) + "," +
         String(osc1PW) + "," + String(osc2PW) + "," + String(osc1PWM) + "," + String(osc2PWM) + "," + String(ampAttack) + "," + String(ampDecay) + "," + String(ampSustain) + "," + String(ampRelease) + "," +
         String(osc2interval) + "," + String(filterAttack) + "," + String(filterDecay) + "," + String(filterSustain) + "," + String(filterRelease) + "," + String(filterRes) + "," + String(filterCutoff) + "," +
         String(filterLevel) + "," + String(osc1foot) + "," + String(osc2foot) + "," + String(octave0) + "," + String(octave1)  + "," + String(shvco) + "," + String(shvcf) + "," + String(vcfVelocity)  + "," +
         String(vcaVelocity) + "," + String(vcfLoop) + "," + String(vcaLoop) + "," + String(vcfLinear) + "," + String(vcaLinear) + "," + String(keyMode) + "," + String(modWheelDepth) + "," + String(pitchBendRange);
}

void checkMux()
{

  mux1Read = adc->adc1->analogRead(MUX1_S);
  mux2Read = adc->adc1->analogRead(MUX2_S);

  if (mux1Read > (mux1ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux1Read < (mux1ValuesPrev[muxInput] - QUANTISE_FACTOR))
  {
    mux1ValuesPrev[muxInput] = mux1Read;

    switch (muxInput)
    {
      case MUX1_KBGLIDE:
        myControlChange(midiChannel, CCglide, mux1Read);
        break;
      case MUX1_LFORATE:
        myControlChange(midiChannel, CCLfoRate, mux1Read);
        break;
      case MUX1_OSC1LEVEL:
        myControlChange(midiChannel, CCosc1level, mux1Read);
        break;
      case MUX1_CUTOFF:
        myControlChange(midiChannel, CCfilterCutoff, mux1Read);
        break;
      case MUX1_EMPHASIS:
        myControlChange(midiChannel, CCfilterRes, mux1Read);
        break;
      case MUX1_CONTOURAMT:
        myControlChange(midiChannel, CCfilterlevel, mux1Read);
        break;
      case MUX1_OSC1PW:
        myControlChange(midiChannel, CCosc1PW, mux1Read);
        break;
      case MUX1_OSC1PWM:
        myControlChange(midiChannel, CCosc1PWM, mux1Read);
        break;
      case MUX1_NOISE:
        myControlChange(midiChannel, CCnoiseLevel, mux1Read);
        break;
    }
  }

  if (mux2Read > (mux2ValuesPrev[muxInput] + QUANTISE_FACTOR) || mux2Read < (mux2ValuesPrev[muxInput] - QUANTISE_FACTOR))
  {
    mux2ValuesPrev[muxInput] = mux2Read;

    switch (muxInput)
    {
      case MUX2_OSC2LEVEL:
        myControlChange(midiChannel, CCosc2level, mux2Read);
        break;
      case MUX2_FILTERATTACK:
        myControlChange(midiChannel, CCfilterAttack, mux2Read);
        break;
      case MUX2_FILTERDECAY:
        myControlChange(midiChannel, CCfilterDecay, mux2Read);
        break;
      case MUX2_FILTERSUSTAIN:
        myControlChange(midiChannel, CCfilterSustain, mux2Read);
        break;
      case MUX2_FILTERRELEASE:
        myControlChange(midiChannel, CCfilterRelease, mux2Read);
        break;
      case MUX2_INTERVAL:
        myControlChange(midiChannel, CCosc2interval, mux2Read);
        break;
      case MUX2_OSC2PW:
        myControlChange(midiChannel, CCosc2PW, mux2Read);
        break;
      case MUX2_OSC2PWM:
        myControlChange(midiChannel, CCosc2PWM, mux2Read);
        break;
      case MUX2_AMPATTACK:
        myControlChange(midiChannel, CCampAttack, mux2Read);
        break;
      case MUX2_AMPDECAY:
        myControlChange(midiChannel, CCampDecay, mux2Read);
        break;
      case MUX2_AMPSUSTAIN:
        myControlChange(midiChannel, CCampSustain, mux2Read);
        break;
      case MUX2_AMPRELEASE:
        myControlChange(midiChannel, CCampRelease, mux2Read);
        break;
      case MUX2_PWLFORATE:
        myControlChange(midiChannel, CCpwLFO, mux2Read);
        break;
    }
  }

  muxInput++;
  if (muxInput >= MUXCHANNELS)
    muxInput = 0;

  digitalWriteFast(MUX_0, muxInput & B0001);
  digitalWriteFast(MUX_1, muxInput & B0010);
  digitalWriteFast(MUX_2, muxInput & B0100);
  digitalWriteFast(MUX_3, muxInput & B1000);

}

void writeDemux()
{
  delayMicroseconds(DelayForSH3);

  //DEMUX 1
  switch (muxOutput)
  {
    case 0: // 5volt
      setVoltage(DAC_NOTE1, 0, 1, int(filterAttack * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(filterDecay * 1.85));
      break;
    case 1: // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(filterSustain * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(filterRelease * 1.85));
      break;
    case 2: // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(ampAttack * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(ampDecay * 1.85));
      break;
    case 3: // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(ampSustain * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(ampRelease * 1.85));
      break;
    case 4: // 5Volt
      setVoltage(DAC_NOTE1, 0, 1, int(LfoRate * 1.85));
      setVoltage(DAC_NOTE1, 1, 1, int(pwLFO * 1.85));
      break;
    case 5: // 2Volt
      setVoltage(DAC_NOTE1, 0, 1, modulation << 4);
      setVoltage(DAC_NOTE1, 1, 1, int(LfoWave * 1.85));
      break;
    case 6: // 2Volt
      setVoltage(DAC_NOTE1, 0, 1, int(osc1PWM / 1.07));
      setVoltage(DAC_NOTE1, 1, 1, int(osc2PWM / 1.07));
      break;
    case 7: // 2Volt
      setVoltage(DAC_NOTE1, 0, 1, int((pitchbend / 8) + 1024));
      setVoltage(DAC_NOTE1, 1, 1, int(noiseLevel * 2));
      break;
    case 8: // 10 Volt
      setVoltage(DAC_NOTE1, 0, 1, int(filterCutoff * 1.9));
      setVoltage(DAC_NOTE1, 1, 1, int(filterRes * 1.9));
      break;
    case 9: // 6 Volt
      setVoltage(DAC_NOTE1, 0, 1, int((osc1PW * 1.22) + 50));
      setVoltage(DAC_NOTE1, 1, 1, int((osc2PW * 1.22) + 50));
      break;
    case 10: // 2 Volt
      setVoltage(DAC_NOTE1, 0, 1, int(osc1level * 2));
      setVoltage(DAC_NOTE1, 1, 1, int(osc2level * 2));
      break;
    case 11:
      // 0-2V
      if (osc2interval < 9 )
      {
      setVoltage(DAC_NOTE1, 0, 1, 0);
      }
      else
      {
        setVoltage(DAC_NOTE1, 0, 1, int(osc2interval * 2));
      }
      // 10 Volt
      setVoltage(DAC_NOTE1, 1, 1, int(glide * 1.9));
      break;
    case 12:
      // 0-5V
      setVoltage(DAC_NOTE1, 0, 1, int(filterLevel * 1.85));
      // undefined spare
      break;
    case 13:
      setVoltage(DAC_NOTE1, 0, 0, int(osc1foot));
      setVoltage(DAC_NOTE1, 1, 0, int(osc2foot));
      break;
    case 14:
      break;
    case 15:
      break;
  }
  delayMicroseconds(DelayForSH3);

  muxOutput++;
  if (muxOutput >= DEMUXCHANNELS)
    muxOutput = 0;

  digitalWriteFast(DEMUX_0, muxOutput & B0001);
  digitalWriteFast(DEMUX_1, muxOutput & B0010);
  digitalWriteFast(DEMUX_2, muxOutput & B0100);
  digitalWriteFast(DEMUX_3, muxOutput & B1000);

}

void checkSwitches()
{

  osc1_32Switch.update();
  if ( mux.readPin(OSC1_32) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC1_32) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc1_32switch = !osc1_32switch;
      myControlChange(midiChannel, CCosc1_32, osc1_32switch);
    }
    buttonDebounce = 0;
  }

  osc1_16Switch.update();
  if ( mux.readPin(OSC1_16) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC1_16) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc1_16switch = !osc1_16switch;
      myControlChange(midiChannel, CCosc1_16, osc1_16switch);
    }
    buttonDebounce = 0;
  }

  osc1_8Switch.update();
  if ( mux.readPin(OSC1_8) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC1_8) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    { ;
      osc1_8switch = !osc1_8switch;
      myControlChange(midiChannel, CCosc1_8, osc1_8switch);
    }
    buttonDebounce = 0;
  }

  osc1_sawSwitch.update();
  if ( mux.readPin(OSC1_SAW) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC1_SAW) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc1_sawswitch = !osc1_sawswitch;
      myControlChange(midiChannel, CCosc1_saw, osc1_sawswitch);
    }
    buttonDebounce = 0;
  }

  osc1_triSwitch.update();
  if ( mux.readPin(OSC1_TRI) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC1_TRI) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc1_triswitch = !osc1_triswitch;
      myControlChange(midiChannel, CCosc1_tri, osc1_triswitch);
    }
    buttonDebounce = 0;
  }

  osc1_pulseSwitch.update();
  if ( mux.readPin(OSC1_PULSE) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC1_PULSE) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc1_pulseswitch = !osc1_pulseswitch;
      myControlChange(midiChannel, CCosc1_pulse, osc1_pulseswitch);
    }
    buttonDebounce = 0;
  }

  singleSwitch.update();
  if ( mux.readPin(SINGLE_TRIG) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(SINGLE_TRIG) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      singleswitch = !singleswitch;
      myControlChange(midiChannel, CCsingle, singleswitch);
    }
    buttonDebounce = 0;
  }

  multiSwitch.update();
  if ( mux.readPin(MULTIPLE_TRIG) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(MULTIPLE_TRIG) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      multiswitch = !multiswitch;
      myControlChange(midiChannel, CCmulti, multiswitch);
    }
    buttonDebounce = 0;
  }

  lfoTriangleSwitch.update();
  if ( mux.readPin(LFO_TRIANGLE) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LFO_TRIANGLE) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      lfoTriangleswitch = !lfoTriangleswitch;
      myControlChange(midiChannel, CClfoTriangle, lfoTriangleswitch);
    }
    buttonDebounce = 0;
  }

  lfoSquareSwitch.update();
  if ( mux.readPin(LFO_SQUARE) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LFO_SQUARE) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      lfoSquareswitch = !lfoSquareswitch;
      myControlChange(midiChannel, CClfoSquare, lfoSquareswitch);
    }
    buttonDebounce = 0;
  }

  syncOffSwitch.update();
  if ( mux.readPin(SYNC_OFF) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(SYNC_OFF) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      syncOffswitch = 1;
      syncOnswitch = 0;
      myControlChange(midiChannel, CCsyncOff, syncOffswitch);
    }
    buttonDebounce = 0;
  }

  syncOnSwitch.update();
  if ( mux.readPin(SYNC_ON) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(SYNC_ON) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      syncOnswitch = 1;
      syncOffswitch = 0;
      myControlChange(midiChannel, CCsyncOn, syncOnswitch);
    }
    buttonDebounce = 0;
  }

  octave0Switch.update();
  if ( mux.readPin(OCTAVE_0) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OCTAVE_0) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      octave0switch = !octave0switch;
      myControlChange(midiChannel, CCoctave0, octave0switch);
    }
    buttonDebounce = 0;
  }

  octave1Switch.update();
  if ( mux.readPin(OCTAVE_1) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OCTAVE_1) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      octave1switch = !octave1switch;
      myControlChange(midiChannel, CCoctave1, octave1switch);
    }
    buttonDebounce = 0;
  }

  kbOffSwitch.update();
  if ( mux.readPin(KB_OFF) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(KB_OFF) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      kbOffswitch = !kbOffswitch;
      myControlChange(midiChannel, CCkbOff, kbOffswitch);
    }
    buttonDebounce = 0;
  }

  kbHalfSwitch.update();
  if ( mux.readPin(KB_HALF) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(KB_HALF) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      kbHalfswitch = !kbHalfswitch;
      myControlChange(midiChannel, CCkbHalf, kbHalfswitch);
    }
    buttonDebounce = 0;
  }

  kbFullSwitch.update();
  if ( mux.readPin(KB_FULL) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(KB_FULL) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      kbFullswitch = !kbFullswitch;
      myControlChange(midiChannel, CCkbFull, kbFullswitch);
    }
    buttonDebounce = 0;
  }

  osc2_32Switch.update();
  if ( mux.readPin(OSC2_32) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC2_32) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc2_32switch = !osc2_32switch;
      myControlChange(midiChannel, CCosc2_32, osc2_32switch);
    }
    buttonDebounce = 0;
  }

  osc2_16Switch.update();
  if ( mux.readPin(OSC2_16) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC2_16) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc2_16switch = !osc2_16switch;
      myControlChange(midiChannel, CCosc2_16, osc2_16switch);
    }
    buttonDebounce = 0;
  }

  osc2_8Switch.update();
  if ( mux.readPin(OSC2_8) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC2_8) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    { ;
      osc2_8switch = !osc2_8switch;
      myControlChange(midiChannel, CCosc2_8, osc2_8switch);
    }
    buttonDebounce = 0;
  }

  osc2_sawSwitch.update();
  if ( mux.readPin(OSC2_SAW) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC2_SAW) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc2_sawswitch = !osc2_sawswitch;
      myControlChange(midiChannel, CCosc2_saw, osc2_sawswitch);
    }
    buttonDebounce = 0;
  }

  osc2_triSwitch.update();
  if ( mux.readPin(OSC2_TRI) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC2_TRI) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc2_triswitch = !osc2_triswitch;
      myControlChange(midiChannel, CCosc2_tri, osc2_triswitch);
    }
    buttonDebounce = 0;
  }

  osc2_pulseSwitch.update();
  if ( mux.readPin(OSC2_PULSE) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(OSC2_PULSE) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      osc2_pulseswitch = !osc2_pulseswitch;
      myControlChange(midiChannel, CCosc2_pulse, osc2_pulseswitch);
    }
    buttonDebounce = 0;
  }

  lfoOscOffSwitch.update();
  if ( mux.readPin(LFO_OSC_OFF) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LFO_OSC_OFF) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      lfoOscOffswitch = !lfoOscOffswitch;
      myControlChange(midiChannel, CClfoOscOff, lfoOscOffswitch);
    }
    buttonDebounce = 0;
  }

  lfoOscOnSwitch.update();
  if ( mux.readPin(LFO_OSC_ON) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LFO_OSC_ON) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      lfoOscOnswitch = !lfoOscOnswitch;
      myControlChange(midiChannel, CClfoOscOn, lfoOscOnswitch);
    }
    buttonDebounce = 0;
  }

  lfoVCFOffSwitch.update();
  if ( mux.readPin(LFO_VCF_OFF) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LFO_VCF_OFF) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      lfoVCFOffswitch = !lfoVCFOffswitch;
      myControlChange(midiChannel, CClfoVCFOff, lfoVCFOffswitch);
    }
    buttonDebounce = 0;
  }

  lfoVCFOnSwitch.update();
  if ( mux.readPin(LFO_VCF_ON) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LFO_VCF_ON) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      lfoVCFOnswitch = !lfoVCFOnswitch;
      myControlChange(midiChannel, CClfoVCFOn, lfoVCFOnswitch);
    }
    buttonDebounce = 0;
  }

  level1Switch.update();
  if ( mux.readPin(LEVEL1) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LEVEL1) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      level1switch = !level1switch;
      myControlChange(midiChannel, CClevel1, level1switch);
    }
    buttonDebounce = 0;
  }

  level2Switch.update();
  if ( mux.readPin(LEVEL2) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(LEVEL2) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      level2switch = !level2switch;
      myControlChange(midiChannel, CClevel2, level2switch);
    }
    buttonDebounce = 0;
  }

  button1Switch.update();
  if ( mux.readPin(BUTTON1) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON1) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button1switch = !button1switch;
      myControlChange(midiChannel, CCbutton1, button1switch);
    }
    buttonDebounce = 0;
  }

  button2Switch.update();
  if ( mux.readPin(BUTTON2) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON2) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button2switch = !button2switch;
      myControlChange(midiChannel, CCbutton2, button2switch);
    }
    buttonDebounce = 0;
  }

  button3Switch.update();
  if ( mux.readPin(BUTTON3) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON3) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button3switch = !button3switch;
      myControlChange(midiChannel, CCbutton3, button3switch);
    }
    buttonDebounce = 0;
  }

  button4Switch.update();
  if ( mux.readPin(BUTTON4) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON4) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button4switch = !button4switch;
      myControlChange(midiChannel, CCbutton4, button4switch);
    }
    buttonDebounce = 0;
  }

  button5Switch.update();
  if ( mux.readPin(BUTTON5) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON5) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button5switch = !button5switch;
      myControlChange(midiChannel, CCbutton5, button5switch);
    }
    buttonDebounce = 0;
  }

  button6Switch.update();
  if ( mux.readPin(BUTTON6) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON6) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button6switch = !button6switch;
      myControlChange(midiChannel, CCbutton6, button6switch);
    }
    buttonDebounce = 0;
  }


  button7Switch.update();
  if ( mux.readPin(BUTTON7) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON7) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button7switch = !button7switch;
      myControlChange(midiChannel, CCbutton7, button7switch);
    }
    buttonDebounce = 0;
  }

  button8Switch.update();
  if ( mux.readPin(BUTTON8) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON8) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button8switch = !button8switch;
      myControlChange(midiChannel, CCbutton8, button8switch);
    }
    buttonDebounce = 0;
  }

  button9Switch.update();
  if ( mux.readPin(BUTTON9) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON9) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button9switch = !button9switch;
      myControlChange(midiChannel, CCbutton9, button9switch);
    }
    buttonDebounce = 0;
  }

  button10Switch.update();
  if ( mux.readPin(BUTTON10) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON10) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button10switch = !button10switch;
      myControlChange(midiChannel, CCbutton10, button10switch);
    }
    buttonDebounce = 0;
  }


  button11Switch.update();
  if ( mux.readPin(BUTTON11) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON11) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button11switch = !button11switch;
      myControlChange(midiChannel, CCbutton11, button11switch);
    }
    buttonDebounce = 0;
  }

  button12Switch.update();
  if ( mux.readPin(BUTTON12) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON12) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button12switch = !button12switch;
      myControlChange(midiChannel, CCbutton12, button12switch);
    }
    buttonDebounce = 0;
  }

  button13Switch.update();
  if ( mux.readPin(BUTTON13) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON13) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button13switch = !button13switch;
      myControlChange(midiChannel, CCbutton13, button13switch);
    }
    buttonDebounce = 0;
  }

  button14Switch.update();
  if ( mux.readPin(BUTTON14) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON14) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button14switch = !button14switch;
      myControlChange(midiChannel, CCbutton14, button14switch);
    }
    buttonDebounce = 0;
  }


  button15Switch.update();
  if ( mux.readPin(BUTTON15) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON15) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button15switch = !button15switch;
      myControlChange(midiChannel, CCbutton15, button15switch);
    }
    buttonDebounce = 0;
  }

  button16Switch.update();
  if ( mux.readPin(BUTTON16) == 0 && buttonDebounce == 0 )
  {
    buttonDebounce = millis();
  }
  if ( mux.readPin(BUTTON16) == 1 )
  {
    if ((buttonDebounce != 0) && (millis() - buttonDebounce > 20))
    {
      button16switch = !button16switch;
      myControlChange(midiChannel, CCbutton16, button16switch);
    }
    buttonDebounce = 0;
  }

  saveButton.update();
  if (saveButton.read() == LOW && saveButton.duration() > HOLD_DURATION)
  {
    switch (state)
    {
      case PARAMETER:
      case PATCH:
        state = DELETE;
        saveButton.write(HIGH); //Come out of this state
        del = true;             //Hack
        break;
    }
  }
  else if (saveButton.risingEdge())
  {
    if (!del)
    {
      switch (state)
      {
        case PARAMETER:
          if (patches.size() < PATCHES_LIMIT)
          {
            resetPatchesOrdering(); //Reset order of patches from first patch
            patches.push({patches.size() + 1, INITPATCHNAME});
            state = SAVE;
          }
          break;
        case SAVE:
          //Save as new patch with INITIALPATCH name or overwrite existing keeping name - bypassing patch renaming
          patchName = patches.last().patchName;
          state = PATCH;
          savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
          showPatchPage(patches.last().patchNo, patches.last().patchName);
          patchNo = patches.last().patchNo;
          loadPatches(); //Get rid of pushed patch if it wasn't saved
          setPatchesOrdering(patchNo);
          renamedPatch = "";
          state = PARAMETER;
          break;
        case PATCHNAMING:
          if (renamedPatch.length() > 0) patchName = renamedPatch;//Prevent empty strings
          state = PATCH;
          savePatch(String(patches.last().patchNo).c_str(), getCurrentPatchData());
          showPatchPage(patches.last().patchNo, patchName);
          patchNo = patches.last().patchNo;
          loadPatches(); //Get rid of pushed patch if it wasn't saved
          setPatchesOrdering(patchNo);
          renamedPatch = "";
          state = PARAMETER;
          break;
      }
    }
    else
    {
      del = false;
    }
  }

  settingsButton.update();
  if (settingsButton.read() == LOW && settingsButton.duration() > HOLD_DURATION)
  {
    //If recall held, set current patch to match current hardware state
    //Reinitialise all hardware values to force them to be re-read if different
    state = REINITIALISE;
    reinitialiseToPanel();
    settingsButton.write(HIGH); //Come out of this state
    reini = true;           //Hack
  }
  else if (settingsButton.risingEdge())
  { //cannot be fallingEdge because holding button won't work
    if (!reini)
    {
      switch (state)
      {
        case PARAMETER:
          settingsValueIndex = getCurrentIndex(settingsOptions.first().currentIndex);
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGS);
          state = SETTINGS;
          break;
        case SETTINGS:
          settingsOptions.push(settingsOptions.shift());
          settingsValueIndex = getCurrentIndex(settingsOptions.first().currentIndex);
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGS);
        case SETTINGSVALUE:
          //Same as pushing Recall - store current settings item and go back to options
          settingsHandler(settingsOptions.first().value[settingsValueIndex], settingsOptions.first().handler);
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGS);
          state = SETTINGS;
          break;
      }
    }
    else
    {
      reini = false;
    }
  }

  backButton.update();
  if (backButton.read() == LOW && backButton.duration() > HOLD_DURATION)
  {
    //If Back button held, Panic - all notes off
    allNotesOff();
    backButton.write(HIGH); //Come out of this state
    panic = true;           //Hack
  }
  else if (backButton.risingEdge())
  { //cannot be fallingEdge because holding button won't work
    if (!panic)
    {
      switch (state)
      {
        case RECALL:
          setPatchesOrdering(patchNo);
          state = PARAMETER;
          break;
        case SAVE:
          renamedPatch = "";
          state = PARAMETER;
          loadPatches();//Remove patch that was to be saved
          setPatchesOrdering(patchNo);
          break;
        case PATCHNAMING:
          charIndex = 0;
          renamedPatch = "";
          state = SAVE;
          break;
        case DELETE:
          setPatchesOrdering(patchNo);
          state = PARAMETER;
          break;
        case SETTINGS:
          state = PARAMETER;
          break;
        case SETTINGSVALUE:
          settingsValueIndex = getCurrentIndex(settingsOptions.first().currentIndex);
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGS);
          state = SETTINGS;
          break;
      }
    }
    else
    {
      panic = false;
    }
  }

  //Encoder switch
  recallButton.update();
  if (recallButton.read() == LOW && recallButton.duration() > HOLD_DURATION)
  {
    //If Recall button held, return to current patch setting
    //which clears any changes made
    state = PATCH;
    //Recall the current patch
    patchNo = patches.first().patchNo;
    recallPatch(patchNo);
    state = PARAMETER;
    recallButton.write(HIGH); //Come out of this state
    recall = true;            //Hack
  }
  else if (recallButton.risingEdge())
  {
    if (!recall)
    {
      switch (state)
      {
        case PARAMETER:
          state = RECALL;//show patch list
          break;
        case RECALL:
          state = PATCH;
          //Recall the current patch
          patchNo = patches.first().patchNo;
          recallPatch(patchNo);
          state = PARAMETER;
          break;
        case SAVE:
          showRenamingPage(patches.last().patchName);
          patchName  = patches.last().patchName;
          state = PATCHNAMING;
          break;
        case PATCHNAMING:
          if (renamedPatch.length() < 13)
          {
            renamedPatch.concat(String(currentCharacter));
            charIndex = 0;
            currentCharacter = CHARACTERS[charIndex];
            showRenamingPage(renamedPatch);
          }
          break;
        case DELETE:
          //Don't delete final patch
          if (patches.size() > 1)
          {
            state = DELETEMSG;
            patchNo = patches.first().patchNo;//PatchNo to delete from SD card
            patches.shift();//Remove patch from circular buffer
            deletePatch(String(patchNo).c_str());//Delete from SD card
            loadPatches();//Repopulate circular buffer to start from lowest Patch No
            renumberPatchesOnSD();
            loadPatches();//Repopulate circular buffer again after delete
            patchNo = patches.first().patchNo;//Go back to 1
            recallPatch(patchNo);//Load first patch
          }
          state = PARAMETER;
          break;
        case SETTINGS:
          //Choose this option and allow value choice
          settingsValueIndex = getCurrentIndex(settingsOptions.first().currentIndex);
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGSVALUE);
          state = SETTINGSVALUE;
          break;
        case SETTINGSVALUE:
          //Store current settings item and go back to options
          settingsHandler(settingsOptions.first().value[settingsValueIndex], settingsOptions.first().handler);
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGS);
          state = SETTINGS;
          break;
      }
    }
    else
    {
      recall = false;
    }
  }
}

void reinitialiseToPanel()
{
  //This sets the current patch to be the same as the current hardware panel state - all the pots
  //The four button controls stay the same state
  //This reinialises the previous hardware values to force a re-read
  muxInput = 0;
  for (int i = 0; i < MUXCHANNELS; i++)
  {
    mux1ValuesPrev[i] = RE_READ;
    mux2ValuesPrev[i] = RE_READ;
  }
  patchName = INITPATCHNAME;
  showPatchPage("Initial", "Panel Settings");
}

void checkEncoder()
{
  //Encoder works with relative inc and dec values
  //Detent encoder goes up in 4 steps, hence +/-3

  long encRead = encoder.read();
  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3) )
  {
    switch (state)
    {
      case PARAMETER:
        state = PATCH;
        patches.push(patches.shift());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case RECALL:
        patches.push(patches.shift());
        break;
      case SAVE:
        patches.push(patches.shift());
        break;
      case PATCHNAMING:
        if (charIndex == TOTALCHARS) charIndex = 0;//Wrap around
        currentCharacter = CHARACTERS[charIndex++];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.push(patches.shift());
        break;
      case SETTINGS:
        settingsOptions.push(settingsOptions.shift());
        settingsValueIndex = getCurrentIndex(settingsOptions.first().currentIndex);
        showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex] , SETTINGS);
        break;
      case SETTINGSVALUE:
        if (settingsOptions.first().value[settingsValueIndex + 1] != '\0')
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[++settingsValueIndex], SETTINGSVALUE);
        break;
    }
    encPrevious = encRead;
  }
  else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3))
  {
    switch (state)
    {
      case PARAMETER:
        state = PATCH;
        patches.unshift(patches.pop());
        patchNo = patches.first().patchNo;
        recallPatch(patchNo);
        state = PARAMETER;
        break;
      case RECALL:
        patches.unshift(patches.pop());
        break;
      case SAVE:
        patches.unshift(patches.pop());
        break;
      case PATCHNAMING:
        if (charIndex == -1)
          charIndex = TOTALCHARS - 1;
        currentCharacter = CHARACTERS[charIndex--];
        showRenamingPage(renamedPatch + currentCharacter);
        break;
      case DELETE:
        patches.unshift(patches.pop());
        break;
      case SETTINGS:
        settingsOptions.unshift(settingsOptions.pop());
        settingsValueIndex = getCurrentIndex(settingsOptions.first().currentIndex);
        showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[settingsValueIndex], SETTINGS);
        break;
      case SETTINGSVALUE:
        if (settingsValueIndex > 0)
          showSettingsPage(settingsOptions.first().option, settingsOptions.first().value[--settingsValueIndex], SETTINGSVALUE);
        break;
    }
    encPrevious = encRead;
  }
}

void loop()
{
  myusb.Task();
  midi1.read(midiChannel);   //USB HOST MIDI Class Compliant
  MIDI.read(midiChannel);
  usbMIDI.read(midiChannel);
  checkMux();
  writeDemux();
  mux.update();
  checkSwitches();
  checkEncoder();
}

int mod(int a, int b)
{
  int r = a % b;
  return r < 0 ? r + b : r;
}
