#include <EEPROM.h>

#define EEPROM_MIDI_CH 0
#define EEPROM_KEY_MODE 1
#define EEPROM_PITCHBEND 2
#define EEPROM_MODWHEEL_DEPTH 3
#define EEPROM_ENCODER_DIR 4
#define EEPROM_LAST_PATCH 5
#define EEPROM_CLOCK_SOURCE 6
#define EEPROM_AT_DEPTH 7
#define EEPROM_AT_DESTINATION 8

int getMIDIChannel() {
  byte midiChannel = EEPROM.read(EEPROM_MIDI_CH);
  if (midiChannel < 0 || midiChannel > 16) midiChannel = MIDI_CHANNEL_OMNI;//If EEPROM has no MIDI channel stored
  return midiChannel;
}

void storeMidiChannel(byte channel)
{
  EEPROM.update(EEPROM_MIDI_CH, channel);
}

int getPitchBendRange() {
  byte pitchbend = EEPROM.read(EEPROM_PITCHBEND);
  if (pitchbend < 0 || pitchbend > 12) return pitchBendRange; //If EEPROM has no pitchbend stored
  return pitchbend;
}

void storePitchBendRange(byte pitchbend)
{
  EEPROM.update(EEPROM_PITCHBEND, pitchbend);
}

int getModWheelDepth() {
  byte mw = EEPROM.read(EEPROM_MODWHEEL_DEPTH);
  if (mw < 0 || mw > 10) return modWheelDepth; //If EEPROM has no mod wheel depth stored
  return mw;
}

void storeModWheelDepth(byte mwDepth)
{
  byte mw = mwDepth;
  EEPROM.update(EEPROM_MODWHEEL_DEPTH, mw);
}

int getAfterTouchDepth() {
  byte at = EEPROM.read(EEPROM_AT_DEPTH);
  if (at < 0 || at > 10) return afterTouchDepth; //If EEPROM has no mod wheel depth stored
  return at;
}

void storeAfterTouchDepth(byte atDepth)
{
  byte at = atDepth;
  EEPROM.update(EEPROM_AT_DEPTH, at);
}

boolean getEncoderDir() {
  byte ed = EEPROM.read(EEPROM_ENCODER_DIR); 
  if (ed < 0 || ed > 1)return true; //If EEPROM has no encoder direction stored
  return ed == 1 ? true : false;
}

void storeEncoderDir(byte encoderDir)
{
  EEPROM.update(EEPROM_ENCODER_DIR, encoderDir);
}

float getKeyMode() {
  byte keyMode = EEPROM.read(EEPROM_KEY_MODE);
  if (keyMode < 0 || keyMode > 2) return keyMode;
  return keyMode; //If EEPROM has no key tracking stored
}

void storeKeyMode(float keyMode)
{
  EEPROM.update(EEPROM_KEY_MODE, keyMode);
}

int getClockSource() {
  byte cs = EEPROM.read(EEPROM_CLOCK_SOURCE);
  if (cs < 0 || cs > 1) return clocksource; //If EEPROM has no mod wheel depth stored
  return cs;
}

void storeClockSource(byte clocksource)
{
  byte cs =  clocksource;
  EEPROM.update(EEPROM_CLOCK_SOURCE, cs);
}

int getLastPatch() {
  int lastPatchNumber = EEPROM.read(EEPROM_LAST_PATCH);
  if (lastPatchNumber < 1 || lastPatchNumber > 999) lastPatchNumber = 1;
  return lastPatchNumber;
}

void storeLastPatch(int lastPatchNumber)
{
  EEPROM.update(EEPROM_LAST_PATCH, lastPatchNumber);
}
