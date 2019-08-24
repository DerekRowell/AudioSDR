//  ------------------------------------------------------------------------------------------
//     SidebandRejection.ino:- A simple AudioSDR test-bench demo demonstrating the unwanted 
//        sideband rejection in SSB demodulation mode.
//
//    This demo is self-contained and requires no external hardware or software beyond
//    the Teensy Audio Board.
//
//    A single frequency SSB signal is generated within the SDR IF passband and then
//    switched in and out of the passband repeatedly simulating reception of the wanted
//    and unwanted sidebands,
//
//    Author:  Derek Rowell
//    Updated: April 4, 2019
//
//  ------------------------------------------------------------------------------------------

#include <Wire.h>
#include <Arduino.h>
#include <Audio.h>
#include <EEPROM.h>
#include "AudioSDRlib.h"
//
float tuning_offset;
float audio_frequency;
int   interval;
//
AudioSynthWaveformSine sine;
AudioIQgenerator       quadGen;
AudioSDR               SDR;
AudioOutputI2S         output;
AudioControlSGTL5000   codec;
//---
// Input processing
AudioConnection c1(sine,0,    quadGen, 0);
AudioConnection c2(quadGen,0, SDR, 0);
AudioConnection c3(quadGen,1, SDR, 1);
AudioConnection c4(SDR,0,     output, 0);
AudioConnection c5(SDR,0,     output, 1);

//***************************************     SETUP     *************************************************

void setup() {
  Serial.begin(57600);
  delay(2000);
  //
  //---
  AudioMemory(20);
  codec.enable();
  codec.inputSelect(AUDIO_INPUT_LINEIN);
  codec.volume(0.7);
  codec.lineInLevel(15);               // Set codec input voltage level to most sensitive
  codec.lineOutLevel(13);              // Set codec output voltage level to most sensitive
  //---
  AudioNoInterrupts();
  AudioInterrupts();
  //
  sine.amplitude(0.1);
  sine.frequency(7000);
  SDR.setInputGain(0.5);
  SDR.setOutputGain(0.5);
  SDR.disableNoiseBlanker();
  SDR.disableALSfilter();
  SDR.disableAGC();
  SDR.enableAudioFilter();
  SDR.setAudioFilter(audio2700);
  SDR.setMute(false);
  // Change the mode to beween USB and LSB mode to hear the rejection
  tuning_offset = SDR.setDemodMode(LSBmode);
  audio_frequency = 1000.0;
  interval = 2000;
  Serial.println(tuning_offset);
}
//--------------------------------------------------------------------------------------------
//                                    ****** LOOP ********
void loop() {
  sine.frequency(tuning_offset + audio_frequency); // A USB signal
  Serial.print("Simulating a USB Signal\n");
  delay(interval);
  sine.frequency(tuning_offset - audio_frequency);   // A LSB signal
  Serial.print("Simulating a LSB Signal\n\n");
  delay(interval);
}
