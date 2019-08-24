/**************************************************************************************************
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

****************************************************************************************************/
#include "AudioIQgenerator.h"
//----
void AudioIQgenerator::update(void) {
  audio_block_t *blockI;
  audio_block_t *blockQ;
  static  float32_t buffer[4*n_block];   // Larger than necessary to allow for experimentation
  static  float32_t Idata[n_block];
  static  float32_t Qdata[n_block];
  //
  // uint32_t t0 = micros();;
  blockI = receiveWritable(0);                          
  if (!blockI) return;
  //  
  blockQ = allocate();
  if (!blockQ) return;  
  //
  // --- Load the data into a length 256 buffer for the Hilbert FIR algorithm
  for (int i = 0; i<n_block; i++){
    buffer[i]           = buffer[i+n_block];
    buffer[i+n_block]   = buffer[i+2*n_block];
    buffer[i+2*n_block] = buffer[i+3*n_block];
    buffer[i+3*n_block] = float(blockI->data[i])/32767.0;

 }
  // 
  // --- Pre-filter The current block with 8th-order IIR band-pass filter (300 - 2900 Hz)
//  arm_biquad_cascade_df1_f32(&inFilt, &buffer[3*n_block], &buffer[3*n_block], n_block);
  //
  // --- Apply FIR Hilbert transform filter to the Q channel
  //     Note: This compact FIR algorithm recognizes that the Hilbert coefficients
  //           are odd symmetric about the mid-point and non-zero for even indices
  //           to reduce the required number of multiplications by a factor of four!
  for (int i=0; i<n_block; i++){
    Qdata[i] = 0.0;
    for (int k=0; k<filterLength/4; k++) {
      int indx1 = (3*n_block+i) - filterLength + 2*(k+1);
      int indx2 = (3*n_block+i) - (2*k+1);
      Qdata[i] += (buffer[indx1]-buffer[indx2])*hilbert_coeffs[k];
    }
    Idata[i] = buffer[3*n_block+i-hilbertDelay];  // Compensate for the FIR filter delay
  }
  //
  // --- Post-filter the I and Q output streams: (Not strictly necessary - comment out if desired)
//  arm_biquad_cascade_df1_f32(&outFiltI, Idata, Idata, n_block);
//  arm_biquad_cascade_df1_f32(&outFiltQ, Qdata, Qdata, n_block);
  //
  for (int i=0; i<n_block; i++){
    blockI->data[i] = ((int16_t)(Idata[i]*32767.0*gainI));
    blockQ->data[i] = ((int16_t)(Qdata[i]*32767.0*gainQ));
  }
  //  Serial.println( micros() -t0);
  transmit(blockI, 0);
  transmit(blockQ, 1);
  release(blockI);
  release(blockQ);
}
