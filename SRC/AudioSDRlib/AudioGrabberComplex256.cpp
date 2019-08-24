/*------------------------------------------------------------------------------------
  AudioGrabberComplex256.cpp:   An AudioSDRlib  component for AudioSDR
  
  Author:    Derek Rowell
  Modified:  August 22, 2019

  Function:  Retrieve a buffer of 256 complex (quadrature) samples (total of 512 samples) 
             from the audio data stream (in arm_math interleaved complex sequence
             {..., real, complex, real, ...} for external processing.  The data is
             returned in Q15.1 format (int16_t).  This intended for occasional data
             retrieval, not for extracting contiguous blocks.
  Usage:      if(myGrabber. newDataAvailable) {
                   myGrabber.grab(myBuffer);
                }
  where myBuffer is length 512 int16_t buffer.
  ---
  Copyright (c) 2019 Derek Rowell
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  ------------------------------------------------------------------------------- */

#include "AudioGrabberComplex256.h"

static void copy_to_buffer(void *destination, const void *sourceRe, const void *sourceIm){  
  const int16_t *srcRe = (const int16_t *)sourceRe;
  const int16_t *srcIm = (const int16_t *)sourceIm;
  int16_t *dst = (int16_t *)destination;
  for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++) {  // Complex data points interleaved in buffer:
    *dst++ = *srcRe++;                           // real sample 
    *dst++ = *srcIm++;                           // imag sample 
  }
}

//--------------------------------------------------------------------------------
void AudioGrabberComplex256::update(void) { 
  audio_block_t *blockI, *blockQ;
  blockI = receiveReadOnly(0);
  blockQ = receiveReadOnly(1);
  if (!blockI &&  blockQ) {release(blockQ); return;}
  if ( blockI && !blockQ) {release(blockI); return;}
  if (!blockI && !blockQ)  return;
  //
  if(!_transferringData) {
    copy_to_buffer(_buffer+_buffStart, blockI->data, blockQ->data);   // Add the current block
    _buffStart = (_buffStart+256)%512;
    if (_buffStart == 0) {
      while(_transferringData){};   // wait for grab to complete
      _dataBufferValid    = false;
      for (int i=0; i<512; i++){
        _outBuffer[i] = _buffer[i];
      }
      _newDataIsAvailable = true;
      _dataBufferValid    = true;
    }
  }
  release(blockI);
  release(blockQ);
}
//
//--------------------------------------------------------------------------------
  bool AudioGrabberComplex256::AudioGrabberComplex256::newDataAvailable(void) {
    return _newDataIsAvailable; 
  }
//
//--------------------------------------------------------------------------------
  void AudioGrabberComplex256::grab(int16_t *destination) {
    if (_dataBufferValid){
      _transferringData = true;
      for (int i=0; i<2*BUFFLENGTH; i++){             // real & imaginary
        *destination++ = _outBuffer[i];
      }
    }
    _transferringData   = false;
    _newDataIsAvailable = false;     
  }


