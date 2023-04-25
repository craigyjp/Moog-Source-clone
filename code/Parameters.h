//Values below are just for initialising and will be changed when synth is initialised to current panel controls & EEPROM settings
byte midiChannel = MIDI_CHANNEL_OMNI;//(EEPROM)
String patchName = INITPATCHNAME;
boolean encCW = true;//This is to set the encoder to increment when turned CW - Settings Option
int modWheelDepth = 0;
int pitchBendRange = 0;
int keyMode = 0;
bool arp = 0;

int noiseLevel = 0;
int noiseLevelstr = 0; // for display
int glide = 0;
int glidestr = 0; // for display

int osc1_32 = 0;
int osc1_32switch = 0;
int osc1_16 = 0;
int osc1_16switch = 0;
int osc1_8 = 0;
int osc1_8switch = 0;

int osc1_saw = 0;
int osc1_sawswitch = 0;
int osc1_tri = 0;
int osc1_triswitch = 0;
int osc1_pulse = 0;
int osc1_pulseswitch = 0;

int osc2_32 = 0;
int osc2_32switch = 0;
int osc2_16 = 0;
int osc2_16switch = 0;
int osc2_8 = 0;
int osc2_8switch = 0;

int osc2_saw = 0;
int osc2_sawswitch = 0;
int osc2_tri = 0;
int osc2_triswitch = 0;
int osc2_pulse = 0;
int osc2_pulseswitch = 0;

int single;
int singleswitch = 0;
int multi;
int multiswitch = 0;
int gatepulse;

int lfoTriangle = 0;
int lfoTriangleswitch = 0;
int lfoSquare = 0;
int lfoSquareswitch = 0;
int lfoOscOff = 0;
int lfoOscOffswitch = 0;
int lfoOscOn = 0;
int lfoOscOnswitch = 0;
int lfoVCFOff = 0;
int lfoVCFOffswitch = 0;
int lfoVCFOn = 0;
int lfoVCFOnswitch = 0;

int syncOff = 0;
int syncOffswitch = 0;
int syncOn = 0;
int syncOnswitch = 0;

int level1 = 1;
int level1switch = 0;
int level2 = 0;
int level2switch = 0;


int octave0 = 0;
int octave0switch = 0;
int octave1 = 0;
int octave1switch = 0;

int kbOff = 0;
int kbOffswitch = 0;
int kbHalf = 0;
int kbHalfswitch = 0;
int kbFull = 0;
int kbFullswitch = 0;

int lfoVCO = 0;
int lfoVCF = 0;

int button1 = 0;
int button1switch = 0;
int button2 = 0;
int button2switch = 0;
int button3 = 0;
int button3switch = 0;
int vcfVelocity = 0;
int button4 = 0;
int button4switch = 0;
int vcaVelocity = 0;
int button5 = 0;
int button5switch = 0;
int vcfLoop= 0;
int button6 = 0;
int button6switch = 0;
int vcaLoop = 0;
int button7 = 0;
int button7switch = 0;
int vcfLinear = 0;
int button8 = 0;
int button8switch = 0;
int vcaLinear = 0;

int button9 = 0;
int button9switch = 0;
int button10 = 0;
int button10switch = 0;
int shvco = 0;
int button11 = 0;
int button11switch = 0;
int shvcf = 0;
int button12 = 0;
int button12switch = 0;
int button13 = 0;
int button13switch = 0;
int button14 = 0;
int button14switch = 0;
int button15 = 0;
int button15switch = 0;
int button16 = 0;
int button16switch = 0;

int returnvalue = 0;

int LfoRate = 0;
int LfoRatestr = 0; //for display
int LfoWave = 0;
int LfoWavestr = 0; //for display
int pwLFO = 0;
int pwLFOstr = 0; // for display

int osc2level = 0; // for display
int osc2levelstr = 0;
int osc1levelstr = 0; //for display
int osc1level = 0;

int osc1foot = 0;
int osc2foot = 0;

int osc1PW = 0;
int osc1PWstr = 0;
int osc2PW = 0;
int osc2PWstr = 0;
int osc2PWM = 0;
int osc2PWMstr = 0;
int osc1PWM = 0;
int osc1PWMstr = 0;

int ampAttack = 0;
int ampAttackstr = 0;
int ampDecay = 0;
int ampDecaystr = 0;
int ampSustain = 0;
int ampSustainstr = 0;
int ampRelease = 0;
int ampReleasestr = 0;

int osc2interval = 0;
int osc2intervalstr = 0;

int filterAttack = 0;
int filterAttackstr = 0;
int filterDecay = 0;
int filterDecaystr = 0;
int filterSustain = 0;
int filterSustainstr = 0;
int filterRelease = 0;
int filterReleasestr = 0;

int filterRes = 0;
int filterResstr = 0;
int filterCutoff = 12000;
int filterCutoffstr = 12000; // for display
int filterLevel = 0;
int filterLevelstr = 0;
