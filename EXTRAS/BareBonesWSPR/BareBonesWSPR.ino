//  ------------------------------------------------------------------------------------------
//    BareBonesWSPR example - A simple fixed frequency WSPR (Weak Signal
//        Propagation Reporter) receiver.
//    Author:  D. Rowell
//    Updated: March 23, 2021
// 
//     This fully functional and tested code that has been in constant unattended
//     use for several weeks.   It is designed to have no user controls, or displays,
//     and is a framework for a more sophisticated SDR
//     
//     What you will need:
//         1) A quadrature output SDR front-end, for example the QRP Labs receiver
//            module.
//         2) A quadrature high-stability rf oscillator (for example an SI5351 module 
//         from Adafruit) and a suitable library to allow you to tune to the WSPR
//         frequencies.
//         3) A computer with a WSPR software decoder (I recommend WSJT-X from
//         http://physics.princeton.edu/pulsar/K1JT/), and a WSPR
//         browser based database interrogator and mapping package (I recommend
//         VK7JJ.com/wspr).
//
// A good introduction to WSPR is at http://www.g4ilo.com/wspr.html
//  ------------------------------------------------------------------------------------------

/*-----------------------------------------------------------------------*
 *  Note: Lines ending in // are particular to my hardware rf front-end *
 *        and should be deleted or modified                              *
 *-----------------------------------------------------------------------*/
#include <Wire.h>
#include <Arduino.h>
#include <Audio.h>
#include "AudioSDRlib.h"
#include "SI5351quad.h"    // My own SI5351 direct quadrature library, substitute your own //*
// SI5351quad is included in "EXTRAS"
//
#define RF_ATTEN_6dB   25  // Multi-RX RF attenuator select  //*                   
#define RF_ATTEN_10dB  26  // Multi-RX RF attenuator select  //*
#define ABPF0          44  // Multi-RX BPF select            //*
#define ABPF1          24  // Multi-RX BPF select            //*
//
#define SI5351_XTAL_ERROR  -538  //Error in SI5351 xtal frequency at 25MHz

// --- Tuning
uint32_t frequency;
uint32_t TuningOffset;
//
// Class instance definitions
SI5351quad  Tuner;                                          //*
// --- Audio library classes
AudioInputI2S          IQinput;
AudioSDRpreProcessor   preProcessor;
AudioSDR               SDR;
AudioOutputI2S         audio_out;
AudioControlSGTL5000   codec;
//---
// Audio Block connections
AudioConnection c1(IQinput, 0,       preProcessor, 0);
AudioConnection c2(IQinput, 1,       preProcessor, 1);
AudioConnection c3(preProcessor, 0,  SDR, 0);
AudioConnection c4(preProcessor, 1,  SDR, 1);
AudioConnection c5(SDR, 0,           audio_out, 0);
AudioConnection c6(SDR, 1,           audio_out, 1);
//
// --------------- SETUP ----------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(2000);
  //
  // --- Set up my personal (home-brew) RF board - ignore!!!                    //*
  // Set up the RF attenuators and band-pass filters                            //*
  pinMode(RF_ATTEN_6dB,  OUTPUT);                                               //*
  pinMode(RF_ATTEN_10dB, OUTPUT);                                               //*
  pinMode(ABPF0,         OUTPUT);                                               //* 
  pinMode(ABPF1,         OUTPUT);                                               //*
  digitalWrite(RF_ATTEN_6dB,  false); // set RF attenuators to off              //*
  digitalWrite(RF_ATTEN_10dB, false);                                           //*
  digitalWrite(ABPF0, LOW);           // Select RF bandpass filter for 4-8 MHz  //*
  digitalWrite(ABPF1, HIGH);                                                    //*
  Tuner.reset();                                                                //*
  delay(20);                                                                    //*
  //  This value must be found for each SI5351 unit:                            //*
  Tuner.Correction(SI5351_XTAL_ERROR)  //Error in SI5351 xtal frequency         //*
  // --- End of personal hardware setup.                                        //*
  //
  // --- Initialize AudioSDR
  preProcessor.startAutoI2SerrorDetection();    // IQ error compensation
  //
  SDR.enableAGC();                      // There's a bit of debate whether AGC should be on for WSPR
  SDR.setAGCmode(AGCmedium);            // 0->OFF; 1->FAST; 2->MEDIUM; 3->SLOW
  SDR.disableALSfilter();
  //
  SDR.disableNoiseBlanker();            // You can choose whether this is necessary
  SDR.setNoiseBlankerThresholdDb(10.0); // This works on my setup
  //
  SDR.setInputGain(1.0);                // You mave have to experiment with these
  SDR.setOutputGain(0.5);
  SDR.setIQgainBalance(1.020);          // This was foumd by experimentation
  //
  // --- WSPR set up
  SDR.setAudioFilter(audioWSPR);  // WSPR output is 200Hz wide, centered on 1500 Hz
  TuningOffset = SDR.setDemodMode(WSPRmode);  // WSPR is always USB
  // Some sample WSPR frequencies - select one or add another
  // frequency = 1836600;                   // WSPR frequency for 160m 
  // frequency = 3592600;                   // WSPR frequency for 80m 
  // frequency = 5287200;                   // WSPR frequency for 60m
     frequency = 7038600;                   // WSPR frequency for 40m
  // frequency = 10138700;                  // WSPR frequency for 30m
  // frequency = 14095600;                  // WSPR frequency for 20m
  // ---
  // Note: My system uses a library that generates the IQ rf signal directly
  //   at the desired frequency.  For use with the common x4 oscillator and a
  //   a Johnson divider to generate the IQ, the following must be modified to
  //   Tuner.SetFrequency((frequency - int(TuningOffset))*4);
  // ---
  Tuner.SetFrequency(frequency - int(TuningOffset));  // see above note      //*
  //
  // --- Set up the Audio board
  AudioMemory(20);
  AudioNoInterrupts();
  codec.inputSelect(AUDIO_INPUT_LINEIN);
  codec.volume(0.7);
  codec.lineInLevel(15);  // Set codec input voltage level to most sensitive
  codec.lineOutLevel(13); // Set codec output voltage level to most sensitive
  codec.enable();
  AudioInterrupts();
  delay(500);
  //
  SDR.setMute(false);
  Serial.print("AudioSDR WSPR Receiver\n  Frequency:  ");
  Serial.println(frequency);
}
//
//---------------------------  LOOP  ------------------------
void loop() {}
