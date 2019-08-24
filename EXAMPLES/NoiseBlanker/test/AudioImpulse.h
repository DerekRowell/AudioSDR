//---------------------------------------------------------------------------------
// File:    AudioWaveform.h
//          Helper file for ''Teensy'' Audio Object AudioWaveform 
//
// Author:  Derek Rowell
//
// Date:    Jan. 8, 2018
//
// Copyright 2018, Derek Rowell
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
//  1) The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 2) THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN 
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------
//
#ifndef audio_impulse_h_
  #define audio_impulse_h_
  #include "Arduino.h"
  #include "AudioStream.h"
  
  class AudioImpulse : public AudioStream {
    public:
      AudioImpulse() : AudioStream(0,NULL){ }       // corresponds to phase shift of pi/2
  //---  Select/change Impulse amplitude
      void amplitude(float amp) {
        amp = abs(amp);
        if (amp>1.0) amp = 1.0;
        _amplitude = int(amp*32767.0);
      }
  //--- Select/change Impulse frequency (impulses/sec)
  void frequency(float freq) {
      _blockCount = (int)(AUDIO_SAMPLE_RATE_EXACT/(128.0)/freq);
  }
  //--- Select/change Impulse width (samples)
  void width(int width){
      _width = width;
  }
  //--- Enable Impulse generation
      void enable(void) {
        _enabled = true;
      }
  //--- Disable Impulse generation
      void disable(void) {
        _enabled = false;
      }
  //---
       virtual void update(void);
       
  //------------------------------------------------------------
    private:
      uint16_t _blockCounter;
      uint16_t _blockCount = 1;
      uint16_t _width      = 1;
      uint16_t _amplitude  = 0;
      boolean  _enabled    = false;
  };
#endif
