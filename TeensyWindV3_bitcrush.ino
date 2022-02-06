// To create a Teensy 3.2-based electronic wind instrument. Instrument controls are an analog pressure sensor, 
// MPR121 capacitive touch sensor with 12 touch plates, and a joystick for control over filter resonance and bitcrushing

// Tuning for this instrument is fixed by waveform frequency parameters. These ratios can be changed to match any scale, experiment!

// Sources:
// Notes & Volts TS-1: https://www.youtube.com/watch?v=UJcZxyB5rVc&list=PL4_gPbvyebyHi4VRZEOG9RKOYq5Hre3a1
// Adafruit MPR121: https://www.adafruit.com/product/1982
// Joystick deadzones: https://forum.arduino.cc/t/problem-controlling-servos-with-joystick/252657/8

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Adafruit_MPR121.h>

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform8;      //xy=75,550
AudioSynthWaveform       waveform9;      //xy=76,611
AudioSynthWaveform       waveform7;      //xy=77,512
AudioSynthWaveform       waveform5;      //xy=78,432
AudioSynthWaveform       waveform6;      //xy=78,472
AudioSynthWaveform       waveform11;     //xy=78,694
AudioSynthWaveform       waveform12;     //xy=78,733
AudioSynthWaveform       waveform1;      //xy=80,250
AudioSynthWaveform       waveform2;      //xy=80,292
AudioSynthWaveform       waveform3;      //xy=80,334
AudioSynthWaveform       waveform4;      //xy=80,375
AudioSynthWaveform       waveform10;     //xy=79,651
AudioEffectEnvelope      envelope9;      //xy=247,612
AudioEffectEnvelope      envelope11;     //xy=249,695
AudioEffectEnvelope      envelope12;     //xy=251,733
AudioEffectEnvelope      envelope10;     //xy=252,651
AudioEffectEnvelope      envelope4;      //xy=259,373
AudioEffectEnvelope      envelope7;      //xy=259,512
AudioEffectEnvelope      envelope6;      //xy=260,472
AudioEffectEnvelope      envelope8;      //xy=260,549
AudioEffectEnvelope      envelope5;      //xy=261,433
AudioEffectEnvelope      envelope3;      //xy=262,335
AudioEffectEnvelope      envelope2;      //xy=263,291
AudioEffectEnvelope      envelope1;      //xy=265,249
AudioMixer4              mixer3;         //xy=465,680
AudioMixer4              mixer1;         //xy=467,320
AudioMixer4              mixer2;         //xy=467,491
AudioMixer4              mixer4;         //xy=670,496
AudioFilterStateVariable filter1;        //xy=825,503
AudioEffectBitcrusher    bitcrusher1;    //xy=982,489
AudioOutputI2S           i2s1;           //xy=1149,488
AudioConnection          patchCord1(waveform8, envelope8);
AudioConnection          patchCord2(waveform9, envelope9);
AudioConnection          patchCord3(waveform7, envelope7);
AudioConnection          patchCord4(waveform5, envelope5);
AudioConnection          patchCord5(waveform6, envelope6);
AudioConnection          patchCord6(waveform11, envelope11);
AudioConnection          patchCord7(waveform12, envelope12);
AudioConnection          patchCord8(waveform1, envelope1);
AudioConnection          patchCord9(waveform2, envelope2);
AudioConnection          patchCord10(waveform3, envelope3);
AudioConnection          patchCord11(waveform4, envelope4);
AudioConnection          patchCord12(waveform10, envelope10);
AudioConnection          patchCord13(envelope9, 0, mixer3, 0);
AudioConnection          patchCord14(envelope11, 0, mixer3, 2);
AudioConnection          patchCord15(envelope12, 0, mixer3, 3);
AudioConnection          patchCord16(envelope10, 0, mixer3, 1);
AudioConnection          patchCord17(envelope4, 0, mixer1, 3);
AudioConnection          patchCord18(envelope7, 0, mixer2, 2);
AudioConnection          patchCord19(envelope6, 0, mixer2, 1);
AudioConnection          patchCord20(envelope8, 0, mixer2, 3);
AudioConnection          patchCord21(envelope5, 0, mixer2, 0);
AudioConnection          patchCord22(envelope3, 0, mixer1, 2);
AudioConnection          patchCord23(envelope2, 0, mixer1, 1);
AudioConnection          patchCord24(envelope1, 0, mixer1, 0);
AudioConnection          patchCord25(mixer3, 0, mixer4, 2);
AudioConnection          patchCord26(mixer1, 0, mixer4, 0);
AudioConnection          patchCord27(mixer2, 0, mixer4, 1);
AudioConnection          patchCord28(mixer4, 0, filter1, 0);
AudioConnection          patchCord29(filter1, 0, bitcrusher1, 0);
AudioConnection          patchCord30(bitcrusher1, 0, i2s1, 0);
AudioConnection          patchCord31(bitcrusher1, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=465,154
// GUItool: end automatically generated code

#define Press A3
#define Joy1 A2
#define Joy2 A6

#define _BV(bit) (1 << (bit)) 

Adafruit_MPR121 cap = Adafruit_MPR121();

uint16_t lasttouched = 0;
uint16_t currtouched = 0;
float ampmain = 0.32;
float mainpitch = 220.00;
int Pressval;
float Pressfloat;
int Joy1val;
int Joy2val;
int Joy1valLow;
int Joy1valHi;
int Joy2valLow;
int Joy2valHi;
float Joy2valFloatLow;
float Joy2valFloatHi;
int thresh1Low = 2200; //Lower deadzone for joystick axis 1. Set these values such that when the joystick returns to center that the values are between these 2
int thresh1Hi = 2700;  //Upper deadzone for joystick axis 1
int thresh2Low = 1900; //Lower deadzone for joystick axis 2
int thresh2Hi = 2400;  //Upper deadzone for joystick axis 2
int envatk = 50;
int envdec = 1;
int envsus = 1;
int envrel = 250;

void setup() {
  analogReadResolution (12);
  AudioMemory (20);

  sgtl5000_1.enable();
  sgtl5000_1.volume(0.45);

  waveform1.begin (WAVEFORM_TRIANGLE);
  waveform1.amplitude (ampmain);
  waveform1.frequency (mainpitch);

  waveform2.begin (WAVEFORM_TRIANGLE);
  waveform2.amplitude (ampmain);
  waveform2.frequency (mainpitch * 13/12);

  waveform3.begin (WAVEFORM_TRIANGLE);
  waveform3.amplitude (ampmain);
  waveform3.frequency (mainpitch * 7/6);

  waveform4.begin (WAVEFORM_TRIANGLE);
  waveform4.amplitude (ampmain);
  waveform4.frequency (mainpitch * 4/3);

  waveform5.begin (WAVEFORM_TRIANGLE);
  waveform5.amplitude (ampmain);
  waveform5.frequency (mainpitch * 3/2);

  waveform6.begin (WAVEFORM_TRIANGLE);
  waveform6.amplitude (ampmain);
  waveform6.frequency (mainpitch * 5/3);

  waveform7.begin (WAVEFORM_TRIANGLE);
  waveform7.amplitude (ampmain);
  waveform7.frequency (mainpitch * 11/6);

  waveform8.begin (WAVEFORM_TRIANGLE);
  waveform8.amplitude (ampmain);
  waveform8.frequency (mainpitch * 2);

  waveform9.begin (WAVEFORM_TRIANGLE);
  waveform9.amplitude (ampmain);
  waveform9.frequency (mainpitch * 2 * 13/12);

  waveform10.begin (WAVEFORM_TRIANGLE);
  waveform10.amplitude (ampmain);
  waveform10.frequency (mainpitch * 2 * 7/6);

  waveform11.begin (WAVEFORM_TRIANGLE);
  waveform11.amplitude (ampmain);
  waveform11.frequency (mainpitch * 2 * 4/3);

  waveform12.begin (WAVEFORM_TRIANGLE);
  waveform12.amplitude (ampmain);
  waveform12.frequency (mainpitch * 2 * 3/2);

  mixer1.gain (0, 0.6);
  mixer1.gain (1, 0.6);
  mixer1.gain (2, 0.6);
  mixer1.gain (3, 0.6);
  
  mixer2.gain (0, 0.6);
  mixer2.gain (1, 0.6);
  mixer2.gain (2, 0.6);
  mixer2.gain (3, 0.6);

  mixer3.gain (0, 0.6);
  mixer3.gain (1, 0.6);
  mixer3.gain (2, 0.6);
  mixer3.gain (3, 0.6);

  mixer4.gain (0, 0.6);
  mixer4.gain (1, 0.6);
  mixer4.gain (2, 0.6);
  mixer4.gain (3, 0.0);

  envelope1.attack(envatk);
  envelope1.decay (envdec);
  envelope1.sustain (envsus);
  envelope1.release (envrel);
  
  envelope2.attack(envatk);
  envelope2.decay (envdec);
  envelope2.sustain (envsus);
  envelope2.release (envrel);
  
  envelope3.attack(envatk);
  envelope3.decay (envdec);
  envelope3.sustain (envsus);
  envelope3.release (envrel);
  
  envelope4.attack(envatk);
  envelope4.decay (envdec);
  envelope4.sustain (envsus);
  envelope4.release (envrel);

  envelope5.attack(envatk);
  envelope5.decay (envdec);
  envelope5.sustain (envsus);
  envelope5.release (envrel);
  
  envelope6.attack(envatk);
  envelope6.decay (envdec);
  envelope6.sustain (envsus);
  envelope6.release (envrel);
  
  envelope7.attack(envatk);
  envelope7.decay (envdec);
  envelope7.sustain (envsus);
  envelope7.release (envrel);
  
  envelope8.attack(envatk);
  envelope8.decay (envdec);
  envelope8.sustain (envsus);
  envelope8.release (envrel);

  envelope9.attack(envatk);
  envelope9.decay (envdec);
  envelope9.sustain (envsus);
  envelope9.release (envrel);
  
  envelope10.attack(envatk);
  envelope10.decay (envdec);
  envelope10.sustain (envsus);
  envelope10.release (envrel);
  
  envelope11.attack(envatk);
  envelope11.decay (envdec);
  envelope11.sustain (envsus);
  envelope11.release (envrel);
  
  envelope12.attack(envatk);
  envelope12.decay (envdec);
  envelope12.sustain (envsus);
  envelope12.release (envrel);

  filter1.frequency(0);
  filter1.resonance(0.7);
  bitcrusher1.bits(16);

  cap.begin(0x5A);
  cap.setThresholds (20, 10);

  //Serial.begin (9600);   //uncomment this line to see the joystick values
}

void loop() {
  // Maps pressure sensor to filter
  Pressval = analogRead (Press);
  Pressval = map (Pressval, 350, 4095, 0, 12000);
  filter1.frequency (Pressval);

  // Maps joystick to control bitcrusher in one axis and filter resonance in the other with null value at center position
  Joy1val = analogRead (Joy1); // for bitcrusher
  //Serial.print ("Axis 1:");  // uncomment these to set the values for Axis 1
  //Serial.print (Joy1val);
  //Serial.print ("\t");
  Joy1valLow = map (Joy1val, 900, thresh1Low, 4, 16); //replace the 900 here with the lowest value seen on this axis
  Joy1valHi = map (Joy1val, thresh1Hi, 4095, 16, 4);  //replace the 4095 here with the highest value seen on this axis
  if (Joy1val <=thresh1Low) {
    bitcrusher1.bits(Joy1valLow);
  }
  if (Joy1val >=thresh1Hi) {
    bitcrusher1.bits(Joy1valHi);
  }

  Joy2val = analogRead (Joy2); // for filter resonance
  //Serial.print ("Axis 2:");  // uncomment these to set the values for Axis 2
  //Serial.println (Joy1val);
  Joy2valLow = map (Joy2val, 700, thresh2Low, 1000, 100); //replace the 700 here with the lowest value seen on this axis
  Joy2valHi = map (Joy2val, thresh2Hi, 4095, 100, 1000);  //replace the 4095 here with the highest value seen on this axis
  if (Joy2val <=thresh2Low) {
    Joy2valFloatLow = Joy2valLow / 100;
    filter1.resonance (Joy2valFloatLow);
  }
  if (Joy2val >=thresh2Hi) {
    Joy2valFloatHi = Joy2valHi / 100;
    filter1.resonance (Joy2valFloatHi);
  }
  //This is the example from Adafruit's MPR121 library to handle capacitive touch stuff
  currtouched = cap.touched(); 

  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      if (i==0) {
        envelope1.noteOn();
      }
      if (i==1) {
        envelope2.noteOn();
      }
      if (i==2) {
        envelope3.noteOn();
      }
      if (i==3) {
        envelope4.noteOn();
      }
      if (i==4) {
        envelope5.noteOn();
      }
      if (i==5) {
        envelope6.noteOn();
      }
      if (i==6) {
        envelope7.noteOn();
      }
      if (i==7) {
        envelope8.noteOn();
      }
      if (i==8) {
        envelope9.noteOn();
      }
      if (i==9) {
        envelope10.noteOn();
      }
      if (i==10) {
        envelope11.noteOn();
      }
      if (i==11) {
        envelope12.noteOn();
      }
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      if (i==0) {
        envelope1.noteOff();
      }
      if (i==1) {
        envelope2.noteOff();
      }
      if (i==2) {
        envelope3.noteOff();
      }
      if (i==3) {
        envelope4.noteOff();
      }
      if (i==4) {
        envelope5.noteOff();
      }
      if (i==5) {
        envelope6.noteOff();
      }
      if (i==6) {
        envelope7.noteOff();
      }
      if (i==7) {
        envelope8.noteOff();
      }
      if (i==8) {
        envelope9.noteOff();
      }
      if (i==9) {
        envelope10.noteOff();
      }
      if (i==10) {
        envelope11.noteOff();
      }
      if (i==11) {
        envelope12.noteOff();
      }
    }
  }
  lasttouched = currtouched;
}
