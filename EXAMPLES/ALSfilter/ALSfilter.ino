//  ------------------------------------------------------------------------------------------
//    ALSfilter.ino:  A simple AudioSDR test-bench demo demonstrating the the performance of
//                    the ALS (adaptive-least-squares) automatic notch/peak filter in the
//                    AudioSDR ayatem.
//
//    This demo is self-contained and requires no external hardware or software beyond
//    the Teensy Audio Board.
//
//    An narrow band signal (sinusoid) in noise is simulated with Teensy Audio Library.
//    It is then passed through a Hilbert transformer to produce a quadrature
//    signal and demodulated as an SSB signal in AudioSDR.  The ALS filter is then cycled through
//    its three modes.
//
//    The code uses an AudioIQgenerator class object to create the quadrature waveform.
//    This is a non-standard Audio object, written by the author, and contained
//    in yhe AudioSDRlib library.
//   
//    Author:  Derek Rowell
//    Updated: August 23, 2019
//
//  ------------------------------------------------------------------------------------------

  #include <Wire.h>
  #include <Arduino.h>
  #include <Audio.h>
  #include "AudioSDRlib.h"
  //
  //Sources
  AudioSynthWaveformSine   SSB;
  AudioSynthNoiseWhite     noise;
  AudioMixer4              summer;
  AudioIQgenerator         IQgen;
  AudioSDR                 SDR;
  AudioOutputI2S           output;
  AudioControlSGTL5000     codec;
  //---
  // --- Synthesize a noisy sinusoidal SSB signal at the IF frequency (6890 Hz)
  AudioConnection  c1(SSB,0,     summer,0);
  AudioConnection  c2(noise,0,   summer,1);
  // --- Generate a quadrature (IQ) signal
  AudioConnection  c3(summer,0,  IQgen, 0);
  // --- Pass the quadrature signal to the SDR
  AudioConnection  c4(IQgen,0,   SDR,0);
  AudioConnection  c5(IQgen,1,   SDR,1);
  // 4) SSB demodulated output is on the Teensy I2S output 0
  AudioConnection  c6(SDR,0,     output,0);
  //  AudioConnection  c7(summer,0,  output,1);
  AudioConnection  c7(SDR,0,  output,1);
  //
/***************************************  SETUP ***********************************/
 
  void setup() {
    Serial.begin(115200);
    float audio_frequency = 2000.0;
    //---
    AudioMemory(20);
    codec.enable();
    codec.inputSelect(AUDIO_INPUT_LINEIN);
    codec.volume(0.7);
    codec.lineInLevel(15);               // Set codec input voltage level to most sensitive
    codec.lineOutLevel(13);              // Set codec output voltage level to most sensitive
    //---
    //
    // - Set up input signal parameters
    SSB.amplitude(0.5);
    SSB.frequency(6890.0);
    noise.amplitude(0.4);
    summer.gain(0, 0.5);
    summer.gain(1, 0.5);
    // - Set up audioSDR
    SDR.setInputGain(0.5);
    SDR.setOutputGain(0.3);
    SDR.disableNoiseBlanker();
    SDR.disableAGC();
    SDR.setMute(false);
    SDR.setAudioFilter(audio2700);
    SDR.enableALSfilter();
    float tuning_offset = SDR.setDemodMode(LSBmode);  // <---- Change between USB/LSB here
    if       (SDR.getDemodMode() == LSBmode) SSB.frequency(tuning_offset - audio_frequency);
    else  if (SDR.getDemodMode() == USBmode) SSB.frequency(tuning_offset + audio_frequency);
  }
//--------------------------------------------------------------------------------------------
//                                    ****** LOOP ******** 
void loop(){
  float interval = 3000.0;
  SDR.disableALSfilter();
  Serial.println("ALS filter: pass-through");
  delay(interval);
  SDR.enableALSfilter();
  SDR.setALSfilterNotch();
  Serial.println("ALS filter: notch mode");
  delay(interval);
  SDR.setALSfilterPeak();
  Serial.println("ALS filter: peak mode\n");
  delay(interval);
  SDR.setMute(true);
  delay(500);
  SDR.setMute(false);
}
