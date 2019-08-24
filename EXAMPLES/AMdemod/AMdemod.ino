//  ------------------------------------------------------------------------------------------
//    AMdemod.ino:  A simple AudioSDR test-bench demo demonstrating the the AM and synchronous
//                  AM (SAM) demodulation.
//
//    This demo is self-contained and requires no external hardware or software beyond
//    the Teensy Audio Board.
//
//    An AM signal at the center frequency of the AudioSDR IF passband (6890 Hz) is
//    generated and, passed through a Hilbert transforme to produce a quadrature
//    signal. It is then demodulated using the AM and SAM demodulators in AudioSDR.
//
//    The code uses an AudioIQgenerator class object to create the quadrature waveform.
//    This is a non-standard Audio object, written by the author, and contained
//    in the AudioSDRlib library.
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
float audio_frequency   = 1000.0;
float carrier_frequency = 6890; // Center of the IF passband
float modulation_depth;
//
//Sources
AudioSynthWaveformSine   carrier;
AudioSynthWaveformSine   audio;
AudioSynthWaveformDc     constant;
AudioMixer4              summer;
AudioEffectMultiply      modulator;
AudioIQgenerator         quadGen;
AudioSDR                 SDR;
AudioOutputI2S           output;
AudioControlSGTL5000     codec;
//---
// Synthesize an AM signal at the center of the IF passband:
//     f_AM(t) = carrier(t) * (constant + f_audio(t))
AudioConnection  c1(audio,0,     summer, 0);
AudioConnection  c2(constant,0,  summer, 1);
AudioConnection  c3(summer,0,    modulator, 0);
AudioConnection  c4(carrier,0,   modulator, 1);
// Use a Hilbert thansform to generate a quadrature (IQ) signal
AudioConnection  c5(modulator,0, quadGen, 0);
// Use the audioSDR convention I->channel 0,  Q->channel 1
AudioConnection  c6(quadGen,0,   SDR, 0);
AudioConnection  c7(quadGen,1,   SDR, 1);
//  AM demodulated output is on the Teensy I2S output 0
AudioConnection  c8(SDR,0,       output, 0);

// ----- SETUP -----
void setup() {
  Serial.begin(57600);
  delay(2000);
  //---
  AudioMemory(20);
  codec.enable();
  codec.volume(0.7);
  codec.lineOutLevel(13);
  // --- Set up the AM signal parameters
  carrier.amplitude(0.5);
  carrier.frequency(carrier_frequency);  // Always 6890 Hz
  audio.amplitude(0.2);
  audio.frequency(audio_frequency);      // Defined as a global
  summer.gain(0, 1.0);
  summer.gain(1, 1.0);
  constant.amplitude(0.5);
  // --- Set up the SDR
  SDR.setOutputGain(1.0);
  SDR.disableNoiseBlanker();
  SDR.disableALSfilter();
  SDR.disableAGC();
  SDR.setMute(false);
  SDR.enableAudioFilter();
}
// ----- LOOP -----
void loop() {
  Serial.print("Carrier frequency: ");
  Serial.print(carrier_frequency);
  Serial.print("   Audio frequency: ");
  Serial.println(audio_frequency);
  //  
  SDR.setMute(false);
  // tuning_offset is not used
  SDR.setDemodMode(AMmode);   // Set the demod mode to AM
  Serial.println("AM envelope detection");
  delay(2000);
  SDR.setMute(true);
  delay(500);

  // Switch to SAM mode
  SDR.setDemodMode(SAMmode);   // Set the demod mode to SAM
  Serial.println("Synchronous AM detection");
  SDR.setMute(false);
  delay(3000);
  if (SDR.getSAMphaseLockStatus() == true) {
    Serial.print("SAM PLL locked to the carrier \n \n");
  } else {
    Serial.print("SAM PLL failed to lock to the carrier - reverted to normal AM demodulation \n \n");
  }
  SDR.setMute(true);
  delay(500);
}
