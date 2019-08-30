/*-------------------------------------------------------------------------------
  AudioGrabberComplex256.h:   An AudioSDRlib  component for AudioSDR
  
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
//--------------------------------------------------------------------------------*/

#ifndef audiograbbercomplex256_h_
#define audiograbbercomplex256_h_
//
#include "core_pins.h"
#include "AudioStream.h"
#include "Arduino.h"
//
#define BUFFLENGTH 256
//
class AudioGrabberComplex256 : public AudioStream {
  public:
    AudioGrabberComplex256() : AudioStream(2, inputQueueArray) { }
    virtual void update(void);
  //---------------------- Public Functions --------------------------
    bool newDataAvailable(void);
    void grab(int16_t *);
//------------------------------------------------------------------
  private:
    audio_block_t   *inputQueueArray[2];
    audio_block_t   *prevblocki;
    int16_t         _buffer[512] __attribute__ ((aligned (4)));
    int16_t         _outBuffer[512];
    uint16_t        _buffStart = 0;
    bool            _dataBufferValid;
    bool            _transferringData   = false;
    bool            _newDataIsAvailable = false;
};

#endif

