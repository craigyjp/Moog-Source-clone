#include <EEPROM.h>

#define EEPROM_MIDI_CH 0
#define EEPROM_KEY_MODE 1
#define EEPROM_PITCHBEND 2
#define EEPROM_MODWHEEL_DEPTH 3
#define EEPROM_ENCODER_DIR 4
#define EEPROM_LAST_PATCH 5

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
  if (pitchbend < 1 || pitchbend > 12) return pitchBendRange; //If EEPROM has no pitchbend stored
  return pitchbend;
}

void storePitchBendRange(byte pitchbend)
{
  EEPROM.update(EEPROM_PITCHBEND, pitchbend);
}

int getModWheelDepth() {
  byte mw = EEPROM.read(EEPROM_MODWHEEL_DEPTH);
  if (mw < 1 || mw > 12) return modWheelDepth; //If EEPROM has no mod wheel depth stored
  return mw;
}

void storeModWheelDepth(byte mwDepth)
{
  byte mw =  mwDepth;
  EEPROM.update(EEPROM_MODWHEEL_DEPTH, mw);
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
  if (keyMode == 0) return 4;
  if (keyMode== 1) return 5;
  if (keyMode == 2) return 6;
  return keyMode; //If EEPROM has no key tracking stored
}

void storeKeyMode(float keyMode)
{
  EEPROM.update(EEPROM_KEY_MODE, keyMode);
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
