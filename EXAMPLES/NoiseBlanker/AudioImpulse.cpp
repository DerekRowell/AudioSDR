//---------------------------------------------------------------------------------
// File:    AudioWaveform.cpp
//          A ''Teensy'' Audio Object function for the generation of waveforms
//
// Author:  Derek Rowell
//
// Date:    January 8, 2018
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
#include "AudioImpulse.h"
void AudioImpulse::update(void) {
//
  uint16_t n_block = 128;
  audio_block_t *blockOut;
  //
  blockOut = allocate();
  if (!blockOut) return;
  //
 // Start with an empty buffer
  for (int i=0; i<n_block; i++) blockOut->data[i] = 0;
  // Return an empty block unless...
  if( (_enabled) && (_width > 0) && (_blockCount > 0) ) {
    if  (_blockCounter > 0) _blockCounter--;
    else {
      for (int i=0; i<_width; i++) blockOut->data[i] = _amplitude;
      _blockCounter = _blockCount;
    }
  }
  transmit(blockOut, 0); 
  release(blockOut);
	return;
}
