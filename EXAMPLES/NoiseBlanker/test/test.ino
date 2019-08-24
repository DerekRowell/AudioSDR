//  ------------------------------------------------------------------------------------------
//    SDR - Teensy 3.6, using phase-shift method of SSB Demodulation.
//          Uses Hilbert Transformer designed using MATLAB.
//
//    Author:  D. Rowell
//    Updated: April 4, 2019
//        
//  ------------------------------------------------------------------------------------------

#include <Wire.h>
#include <Arduino.h>
#include <Audio.h>
#include "AudioSDR.h"
#include "AudioImpulse.h"
#include "AudioIQgenerator.h"
//
  //
  //Sources
  AudioSynthWaveformSine   signal;
  AudioSynthNoiseWhite     noise;
  AudioImpulse             impulse;
  AudioMixer4              summer;
  AudioIQgenerator         IQgen;
  AudioSDR                 SDR;
  AudioOutputI2S           output;
  AudioControlSGTL5000     codec;
//---
// --- Synthesize a noisy sinusoidal SSB signal at the IF frequency (6890 Hz)
  AudioConnection  c1(signal,0,  summer,0);
  AudioConnection  c2(impulse,0, summer,1);
  AudioConnection  c3(noise,0,   summer,2);
// --- Generate a quadrature (IQ) signal
  AudioConnection  c4(summer,0,  IQgen, 0);
// --- Pass the quadrature signal to the SDR
  AudioConnection  c5(IQgen,0,   SDR,0);
  AudioConnection  c6(IQgen,1,   SDR,1);
// 4) SSB demodulated output is on the Teensy I2S output 0
  AudioConnection  c7(SDR,0,     output,0);
  AudioConnection  c8(SDR,0,     output,1);
  //
/***************************************  SETUP ***********************************/
 void setup() {
 Serial.begin(115200);
  float audio_frequency = 500.0;
  delay(2000);
  //
  //---
  AudioMemory(30);
  Serial.println("a");
  codec.enable();
  Serial.println("b");
  codec.inputSelect(AUDIO_INPUT_LINEIN);
  codec.volume(0.7);
  codec.lineInLevel(15);               // Set codec input voltage level to most sensitive
  codec.lineOutLevel(13);              // Set codec output voltage level to most sensitive
  //---
  signal.amplitude(.05);
  signal.frequency(6890);
  noise.amplitude(0.01);
  impulse.enable();
  impulse.frequency(2.0); 
  impulse.amplitude(0.5);               
  impulse.width(5);
  //
  summer.gain(1, 0.5);
  summer.gain(2, 0.5);
  summer.gain(3, 0.5);
// - Set up audioSDR
  SDR.setInputGain(1.0);
  SDR.setOutputGain(1.0);
  SDR.disableNoiseBlanker();
  SDR.setNoiseBlankerThreshold(1.3);
  SDR.disableAGC();
  SDR.setAudioFilter(audio2700);
  SDR.enableALSfilter();
  SDR.setALSfilterBandpass();
  SDR.setMute(false);
  float tuning_offset = SDR.setDemodMode(LSBmode);
  if       (SDR.getDemodMode() == LSBmode) signal.frequency(tuning_offset + audio_frequency);
  else  if (SDR.getDemodMode() == USBmode) signal.frequency(tuning_offset - audio_frequency);

  Serial.println(tuning_offset); 
  Serial.println(tuning_offset + audio_frequency); 
  Serial.println(SDR.getDemodMode()); 
  delay(500);
 }
//--------------------------------------------------------------------------------------------
//                                    ****** LOOP ******** 
void loop(){
  float interval = 3000.0;
  Serial.println("Noise blanker: ON");
  SDR.enableNoiseBlanker();
  delay(3000);
  SDR.setMute(true);
  delay(500);
  SDR.setMute(false);
  //
  SDR.disableNoiseBlanker();
  Serial.println("Noise blanker: OFF");
  delay(interval);
  SDR.setMute(true);
  delay(500);
 SDR.setMute(false);
}
