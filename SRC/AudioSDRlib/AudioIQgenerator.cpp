/*------------------------------------------------------------------------------------
   AudioIQgenerator.cpp

   Function: A single Teensy 3.6 Audio block for generation of a quadrature data stream
             from a single (real) audio input stream.

   Author:   Derek Rowell

   Date:     July 16, 2019
  --
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

#include "AudioIQgenerator.h"
//----
void AudioIQgenerator::update(void)
{
  audio_block_t *blockI;
  audio_block_t *blockQ;
  static float32_t bufferI[3*n_block]; // Larger than necessary to allow for experimentation
  static float32_t bufferQ[3*n_block]; // Larger than necessary to allow for experimentation
  static float32_t Idata[n_block];
  static float32_t Qdata[n_block];
  //
  // uint32_t t0 = micros();;
  blockI = receiveWritable(0);
  if (!blockI)
    return;
  //
  blockQ = allocate();
  if (!blockQ)
    return;
  //
  // Create a long (3 block) buffer for the length 257 FIR hilbert-transform FIR filter:
  for (int i = 0; i < n_block; i++)
  {
    bufferI[i]             = bufferI[n_block + i];
    bufferI[n_block + i]   = bufferI[2*n_block + i];
    bufferI[2*n_block + i] = ((float)blockI->data[i]) / 32767.0;
    bufferQ[i]             = bufferQ[n_block + i];
    bufferQ[n_block + i]   = bufferQ[2*n_block + i];
    bufferQ[2*n_block + i] = ((float)blockI->data[i]) / 32767.0;
  }
  // --- Apply a length 257 FIR hilbert-transform filter to the Q channel
  //     Note: This compact FIR algorithm recognizes that the Hilbert coefficients
  //           are odd symmetric about the mid-point and non-zero for even indices,
  //           to reduce the required number of multiplications by a factor of four!
  for (int i = 0; i < n_block; i++)
  {
    Qdata[i] = 0.0;
    for (int k = 0; k < hilbertFilterLength / 4; k++)
    {
      int indx1 = (2*n_block + i) - (2*k + 1);
      int indx2 = (2*n_block + i) - hilbertFilterLength + 2*(k + 1);
      Qdata[i] += hilbert_coeffs[k]*(bufferQ[indx1] - bufferQ[indx2]); // Convolution
    }
    // Compensate the I channel output for the group delay introduced by the FIR filter:
    Idata[i] = bufferI[2*n_block + i - hilbertDelay];
  }
  //
  for (int i = 0; i < n_block; i++)
  {
    blockI->data[i] = ((int16_t)(Idata[i]*32767.0*gainI));
    blockQ->data[i] = ((int16_t)(Qdata[i]*32767.0*gainQ));
  }
  //  Serial.println( micros() -t0);
  transmit(blockI, 0);
  release(blockI);
  transmit(blockQ, 1);
  release(blockQ);
}
