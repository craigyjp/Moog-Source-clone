// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Bounce.h>
#include "TButton.h"
#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC();

//Teensy 3.6 - Mux Pins
#define MUX_0 28
#define MUX_1 27
#define MUX_2 26
#define MUX_3 25

#define MUX1_S A19
#define MUX2_S A11

#define DEMUX_0 29
#define DEMUX_1 30
#define DEMUX_2 31
#define DEMUX_3 32

#define GATE_NOTE1 23
#define TRIG_NOTE1 22
#define CLOCK 19

//Note DAC
#define DAC_NOTE1 16
#define NOTE_DAC(CH) (CH==0 ? DAC_NOTE1)
#define NOTE_AB(CH)  (CH==1 ? 1 : 0)

//Mux 1 Connections
#define MUX1_KBGLIDE 0
#define MUX1_LFORATE 1
#define MUX1_OSC1LEVEL 2
#define MUX1_CUTOFF 3
#define MUX1_EMPHASIS 4
#define MUX1_CONTOURAMT 5
#define MUX1_OSC1PW 6
#define MUX1_OSC1PWM 7
#define MUX1_NOISE 8
#define MUX1_VOLUME 9
#define MUX1_10 10
#define MUX1_11 11
#define MUX1_12 12
#define MUX1_13 13
#define MUX1_14 14
#define MUX1_15 15

//Mux 2 Connections
#define MUX2_OSC2LEVEL 0
#define MUX2_FILTERATTACK 1
#define MUX2_FILTERDECAY 2
#define MUX2_FILTERSUSTAIN 3
#define MUX2_FILTERRELEASE 4
#define MUX2_INTERVAL 5
#define MUX2_OSC2PW 6
#define MUX2_OSC2PWM 7
#define MUX2_AMPATTACK 8
#define MUX2_AMPDECAY 9
#define MUX2_AMPSUSTAIN 10
#define MUX2_AMPRELEASE 11
#define MUX2_PWLFORATE 12
#define MUX2_13 13
#define MUX2_14 14
#define MUX2_15 15

#define RECALL_SW 17
#define SAVE_SW 24
#define SETTINGS_SW 12
#define BACK_SW 10

// Internal shift registers

#define CLOCK_SOURCE 0
#define VCF_VELOCITY 1
#define OCTAVE 2
#define OSC1_WAVE1 3
#define OSC1_WAVE2 4
#define OSC2_WAVE1 5
#define OSC2_WAVE2 6
#define KEYTRACK1 7

#define KEYTRACK2 8
#define LFO_TO_OSC 9
#define LFO_TO_VCF 10
#define PB_OSC1 11
#define PB_OSC2 12
#define SYNC 13
#define SOFT_SYNC 14
#define TUNE_ENABLE 15

#define SH_TO_VCO 16
#define SH_TO_VCF 17
#define VCF_LOOP 18
#define VCA_LOOP 19
#define VCF_LOG_LIN 20
#define VCA_LOG_LIN 21
#define VCA_VELOCITY 22
#define VCO_SELECT 23


// front panel Buttons

#define OSC1_32 0
#define OSC1_16 1
#define OSC1_8 2
#define OSC1_SAW 3
#define OSC1_TRI 4
#define OSC1_PULSE 5

#define SINGLE_TRIG 8
#define MULTIPLE_TRIG 9
#define LFO_TRIANGLE 10
#define LFO_SQUARE 11
#define SYNC_OFF 12
#define SYNC_ON 13
#define OCTAVE_0 14
#define OCTAVE_1 15

#define KB_OFF 16
#define KB_HALF 17
#define KB_FULL 18
#define OSC2_8 19
#define OSC2_SAW 20
#define OSC2_TRI 21
#define OSC2_PULSE 22

#define LFO_OSC_OFF 24
#define LFO_OSC_ON 25
#define OSC2_32 26
#define OSC2_16 27
#define LEVEL1 28
#define LEVEL2 29
#define LFO_VCF_OFF 30
#define LFO_VCF_ON 31

#define BUTTON1 32
#define BUTTON2 33
#define BUTTON3 34
#define BUTTON4 35
#define BUTTON5 36
#define BUTTON6 37
#define BUTTON7 38
#define BUTTON8 39

#define BUTTON9 40
#define BUTTON10 41
#define BUTTON11 42
#define BUTTON12 43
#define BUTTON13 44
#define BUTTON14 45
#define BUTTON15 46
#define BUTTON16 47

// LEDs 

#define OSC1_32_LED 0
#define OSC1_16_LED 1
#define OSC1_8_LED 2
#define OSC1_SAW_LED 3
#define OSC1_TRI_LED 4
#define OSC1_PULSE_LED 5

#define SINGLE_TRIG_LED 8
#define MULTIPLE_TRIG_LED 9
#define LFO_TRIANGLE_LED 10
#define LFO_SQUARE_LED 11
#define SYNC_OFF_LED 12
#define SYNC_ON_LED 13
#define OCTAVE_0_LED 14
#define OCTAVE_1_LED 15

#define KB_OFF_LED 16
#define KB_HALF_LED 17
#define KB_FULL_LED 18
#define OSC2_8_LED 19
#define OSC2_SAW_LED 20
#define OSC2_TRI_LED 21
#define OSC2_PULSE_LED 22

#define LFO_OSC_OFF_LED 24
#define LFO_OSC_ON_LED 25
#define OSC2_32_LED 26
#define OSC2_16_LED 27
#define LEVEL1_LED 28
#define LEVEL2_LED 29
#define LFO_VCF_OFF_LED 30
#define LFO_VCF_ON_LED 31

#define BUTTON1_LED 32
#define BUTTON2_LED 33
#define BUTTON3_LED 34
#define BUTTON4_LED 35
#define BUTTON5_LED 36
#define BUTTON6_LED 37
#define BUTTON7_LED 38
#define BUTTON8_LED 39

#define BUTTON9_LED 40
#define BUTTON10_LED 41
#define BUTTON11_LED 42
#define BUTTON12_LED 43
#define BUTTON13_LED 44
#define BUTTON14_LED 45
#define BUTTON15_LED 46
#define BUTTON16_LED 47

#define ENCODER_PINA 4
#define ENCODER_PINB 5

#define MUXCHANNELS 16
#define DEMUXCHANNELS 16


#define DEBOUNCE 30

static byte muxInput = 0;
static byte muxOutput = 0;
static int mux1ValuesPrev[MUXCHANNELS] = {};
static int mux2ValuesPrev[MUXCHANNELS] = {};

static int mux1Read = 0;
static int mux2Read = 0;

static long encPrevious = 0;

//These are pushbuttons and require debouncing
Bounce osc1_32Switch = Bounce(OSC1_32, DEBOUNCE);
Bounce osc1_16Switch = Bounce(OSC1_16, DEBOUNCE);
Bounce osc1_8Switch = Bounce(OSC1_8, DEBOUNCE);
Bounce osc1_sawSwitch = Bounce(OSC1_SAW, DEBOUNCE);
Bounce osc1_triSwitch = Bounce(OSC1_TRI, DEBOUNCE);
Bounce osc1_pulseSwitch = Bounce(OSC1_PULSE, DEBOUNCE);
Bounce singleSwitch = Bounce(SINGLE_TRIG, DEBOUNCE);
Bounce multiSwitch = Bounce(MULTIPLE_TRIG, DEBOUNCE);
Bounce lfoTriangleSwitch = Bounce(LFO_TRIANGLE, DEBOUNCE);
Bounce lfoSquareSwitch = Bounce(LFO_SQUARE, DEBOUNCE);
Bounce syncOffSwitch = Bounce(SYNC_OFF, DEBOUNCE);
Bounce syncOnSwitch = Bounce(SYNC_ON, DEBOUNCE);
Bounce octave0Switch = Bounce(OCTAVE_0, DEBOUNCE);
Bounce octave1Switch = Bounce(OCTAVE_1, DEBOUNCE);
Bounce kbOffSwitch = Bounce(KB_OFF, DEBOUNCE);
Bounce kbHalfSwitch = Bounce(KB_HALF, DEBOUNCE);
Bounce kbFullSwitch = Bounce(KB_FULL, DEBOUNCE);
Bounce osc2_8Switch = Bounce(OSC2_8, DEBOUNCE);
Bounce osc2_sawSwitch = Bounce(OSC2_SAW, DEBOUNCE);
Bounce osc2_triSwitch = Bounce(OSC2_TRI, DEBOUNCE);
Bounce osc2_pulseSwitch = Bounce(OSC2_PULSE, DEBOUNCE);
Bounce lfoOscOffSwitch = Bounce(LFO_OSC_OFF, DEBOUNCE);
Bounce lfoOscOnSwitch = Bounce(LFO_OSC_ON, DEBOUNCE);
Bounce osc2_32Switch = Bounce(OSC2_32, DEBOUNCE);
Bounce osc2_16Switch = Bounce(OSC2_16, DEBOUNCE);
Bounce level1Switch = Bounce(LEVEL1, DEBOUNCE);
Bounce level2Switch = Bounce(LEVEL2, DEBOUNCE);
Bounce lfoVCFOffSwitch = Bounce(LFO_VCF_OFF, DEBOUNCE);
Bounce lfoVCFOnSwitch = Bounce(LFO_VCF_ON, DEBOUNCE);

Bounce button1Switch = Bounce(BUTTON1, DEBOUNCE);
Bounce button2Switch = Bounce(BUTTON2, DEBOUNCE);
Bounce button3Switch = Bounce(BUTTON3, DEBOUNCE);
Bounce button4Switch = Bounce(BUTTON4, DEBOUNCE);
Bounce button5Switch = Bounce(BUTTON5, DEBOUNCE);
Bounce button6Switch = Bounce(BUTTON6, DEBOUNCE);
Bounce button7Switch = Bounce(BUTTON7, DEBOUNCE);
Bounce button8Switch = Bounce(BUTTON8, DEBOUNCE);

Bounce button9Switch = Bounce(BUTTON9, DEBOUNCE);
Bounce button10Switch = Bounce(BUTTON10, DEBOUNCE);
Bounce button11Switch = Bounce(BUTTON11, DEBOUNCE);
Bounce button12Switch = Bounce(BUTTON12, DEBOUNCE);
Bounce button13Switch = Bounce(BUTTON13, DEBOUNCE);
Bounce button14Switch = Bounce(BUTTON14, DEBOUNCE);
Bounce button15Switch = Bounce(BUTTON15, DEBOUNCE);
Bounce button16Switch = Bounce(BUTTON16, DEBOUNCE);

Bounce recallButton = Bounce(RECALL_SW, DEBOUNCE); //On encoder
boolean recall = true; //Hack for recall button
Bounce saveButton = Bounce(SAVE_SW, DEBOUNCE);
boolean del = true; //Hack for save button
TButton settingsButton{SETTINGS_SW, LOW, HOLD_DURATION, DEBOUNCE, CLICK_DURATION};
Bounce backButton = Bounce(BACK_SW, DEBOUNCE);
boolean panic = true; //Hack for back button
Encoder encoder(ENCODER_PINB, ENCODER_PINA);//This often needs the pins swapping depending on the encoder

#define QUANTISE_FACTOR 10

void setupHardware()
{
  adc->adc0->setAveraging(16); // set number of averages 0, 4, 8, 16 or 32.
  adc->adc0->setResolution(10); // set bits of resolution  8, 10, 12 or 16 bits.
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed

  //MUXs on ADC1
  adc->adc1->setAveraging(16); // set number of averages 0, 4, 8, 16 or 32.
  adc->adc1->setResolution(10); // set bits of resolution  8, 10, 12 or 16 bits.
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed


  //Mux address pins

  pinMode(DAC_NOTE1, OUTPUT);
  digitalWrite(DAC_NOTE1,HIGH);

  pinMode(GATE_NOTE1, OUTPUT);
  pinMode(TRIG_NOTE1, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  digitalWrite(GATE_NOTE1, LOW);
  digitalWrite(TRIG_NOTE1, LOW);
  digitalWrite(CLOCK, LOW);

  pinMode(MUX_0, OUTPUT);
  pinMode(MUX_1, OUTPUT);
  pinMode(MUX_2, OUTPUT);
  pinMode(MUX_3, OUTPUT);

  pinMode(DEMUX_0, OUTPUT);
  pinMode(DEMUX_1, OUTPUT);
  pinMode(DEMUX_2, OUTPUT);
  pinMode(DEMUX_3, OUTPUT);

  digitalWrite(MUX_0, LOW);
  digitalWrite(MUX_1, LOW);
  digitalWrite(MUX_2, LOW);
  digitalWrite(MUX_3, LOW);

  digitalWrite(DEMUX_0, LOW);
  digitalWrite(DEMUX_1, LOW);
  digitalWrite(DEMUX_2, LOW);
  digitalWrite(DEMUX_3, LOW);

  
  analogReadResolution(10);
  analogWriteResolution(12);
  //Switches
  
  pinMode(RECALL_SW, INPUT_PULLUP); //On encoder
  pinMode(SAVE_SW, INPUT_PULLUP);
  pinMode(SETTINGS_SW, INPUT_PULLUP);
  pinMode(BACK_SW, INPUT_PULLUP);

}
