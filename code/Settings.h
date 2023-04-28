#include "SettingsService.h"

void settingsMIDICh(int index, const char *value);
void settingsEncoderDir(char *value);
void settingsPitchBend(int index, const char *value);
void settingsModWheelDepth(int index, const char *value);
void settingsKeyMode(int index, const char *value);
void settingsClockSource(int index, const char *value);

int currentIndexMIDICh();
int currentIndexEncoderDir();
int currentIndexPitchBend();
int currentIndexModWheelDepth();
int currentIndexKeyMode();
int currentIndexClockSource();


void settingsMIDICh(int index, const char *value) {
  if (strcmp(value, "ALL") == 0) {
    midiChannel = MIDI_CHANNEL_OMNI;
  } else {
    midiChannel = atoi(value);
  }
  storeMidiChannel(midiChannel);
}

void settingsEncoderDir(int index, const char *value) {
  if (strcmp(value, "Type 1") == 0) {
    encCW = true;
  } else {
    encCW = false;
  }
  storeEncoderDir(encCW ? 1 : 0);
}

void settingsPitchBend(int index, const char *value) {
  if (strcmp(value, "Off") == 0) {
    pitchBendRange = 0;
  } else {
    pitchBendRange = atoi(value);
  }
  storePitchBendRange(pitchBendRange);
}

void settingsModWheelDepth(int index, const char *value) {
  if (strcmp(value, "Off") == 0) {
    modWheelDepth = 0;
  } else {
    modWheelDepth = atoi(value);
  }
  storeModWheelDepth(modWheelDepth);
}

void settingsAfterTouchDepth(int index, const char *value) {
  if (strcmp(value, "Off") == 0) {
    afterTouchDepth = 0;
  } else {
    afterTouchDepth = atoi(value);
  }
  storeAfterTouchDepth(afterTouchDepth);
}

void settingsKeyMode(int index, const char *value) {
  if (strcmp(value, "Top") == 0) keyMode = 4;
  if (strcmp(value, "Bottom") == 0)  keyMode =  5;
  if (strcmp(value, "Last") == 0) keyMode =  6;
  storeKeyMode(keyMode);
}

void settingsClockSource(int index, const char *value) {
  if (strcmp(value, "External") == 0) clocksource = 0;
  if (strcmp(value, "MIDI") == 0)  clocksource = 1;
  storeClockSource(clocksource);
}

int currentIndexMIDICh() {
  return getMIDIChannel();
}

int currentIndexEncoderDir() {
  return getEncoderDir() ? 0 : 1;
}

int currentIndexPitchBend() {
  return getPitchBendRange();
}

int currentIndexModWheelDepth() {
  return getModWheelDepth();
}

int currentIndexAfterTouchDepth() {
  return getAfterTouchDepth();
}

int currentIndexKeyMode() {
  float value = getKeyMode();
  if (value == 4) return 0;
  if (value == 5) return 1;
  if (value == 6) return 2;
  return 0;
}

int currentIndexClockSource() {
  return getClockSource();
}

// add settings to the circular buffer
void setUpSettings() {
  settings::append(settings::SettingsOption{ "MIDI In Ch.", { "All", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "\0" }, settingsMIDICh, currentIndexMIDICh });
  settings::append(settings::SettingsOption{ "Key Mode", {"Top", "Bottom", "Last", "\0"}, settingsKeyMode, currentIndexKeyMode });
  settings::append(settings::SettingsOption{ "Encoder", { "Type 1", "Type 2", "\0" }, settingsEncoderDir, currentIndexEncoderDir });
  settings::append(settings::SettingsOption{ "Pitch Bend", { "Off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "\0" }, settingsPitchBend, currentIndexPitchBend });
  settings::append(settings::SettingsOption{ "MW Depth", { "Off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "\0" }, settingsModWheelDepth, currentIndexModWheelDepth });
  settings::append(settings::SettingsOption{ "AT Depth", { "Off", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "\0" }, settingsAfterTouchDepth, currentIndexAfterTouchDepth });
  settings::append(settings::SettingsOption{ "LFO Clock", {"External", "MIDI", "\0"}, settingsClockSource, currentIndexClockSource });
}
