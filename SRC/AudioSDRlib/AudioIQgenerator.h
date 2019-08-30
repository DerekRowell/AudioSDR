/*------------------------------------------------------------------------------------
   AudioIQgenerator.h

   Function: A header file for a single Teensy 3.6 Audio block for generation of a
             quadrature data stream from a single (real) audio input stream.
             Uses a length 257 FIR Hilbert transform filter.

   Author:   Derek Rowell
   Date:     August 20, 2019

   Input:   A singie AudioStream on input 0 
   Outputs: The in-phase (I) component on output 0,
            the quadrature (Q) component on output 1.
 
    Public function:
      myIQ.setGainBalance(float balance) - Sets a compensation factor for any gain imbalance
                                           between the I & Q output channels. The I channel
                                           is set to a gain of balance, and the Q channel
                                           gain is set to 1/balance.  The default is
                                           balance = 1.0.
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

#ifndef Audio_IQ_gen_h_
#define Audio_IQ_gen_h_
//
#include "core_pins.h"
#include "AudioStream.h"
#include "Arduino.h"
#include "arm_math.h"
class AudioIQgenerator : public AudioStream
{
#define n_block 128
public:
  AudioIQgenerator() : AudioStream(1, inputQueueArray) {}
  //----
  virtual void update(void);
  //---
  void setGainBalance(float balance)
  {
    gainI = balance;
    gainQ = 1.0 / balance;
  }

private:
  audio_block_t *inputQueueArray[1];
  arm_biquad_casd_df1_inst_f32 inFilt;
  arm_biquad_casd_df1_inst_f32 outFiltI;
  arm_biquad_casd_df1_inst_f32 outFiltQ;
  //
  float32_t inFilt_state[16] = {0.0};
  float32_t outFiltI_state[16] = {0.0};
  float32_t outFiltQ_state[16] = {0.0};
  //
  float gainI = 1.0;
  float gainQ = 1.0;
  // ----------------
  // Hilbert Transform FIR filter coefficients
  // Note this ia a length 257 FIR filter.   The number ofcoefficients has been reduced by a
  //   factor of four by recognizing that the Hilbert coefficients are
  //   1) odd symetric about the mid-point, and
  //   2) odd coefficients are zero.
  //
  // The filter was designed in MATLAB with the command:
  //   Hd = designfilt('hilbertfir','FilterOrder',256,...
  //                   'TransitionWidth',0.02,...
  //                   'DesignMethod','equiripple');
  // ----------------
  int16_t hilbertFilterLength = 257;
  int16_t hilbertDelay = (hilbertFilterLength - 1) / 2;
  float32_t hilbert_coeffs[64] = {
    -0.003780058, -0.0007677196, -0.0008456816, -0.0009285885,
    -0.001016253, -0.00110868, -0.001206127, -0.001309284,
    -0.001418976, -0.001535649, -0.001658795, -0.001787153,
    -0.001920324, -0.002062003, -0.002218711, -0.002372268,
    -0.00254078, -0.002716246, -0.002901201, -0.003095788,
    -0.003300195, -0.003514823, -0.003740456, -0.00397813,
    -0.004228542, -0.004491779, -0.004768153, -0.005059852,
    -0.00536908, -0.005692769, -0.006036516, -0.00639912,
    -0.006783208, -0.00719045, -0.007622755, -0.008082277,
    -0.008571692, -0.009094321, -0.009653805, -0.01025394,
    -0.01089927, 0.01159615, -0.01235175, -0.01317265,
    -0.01407044, -0.01505581, -0.01614401, -0.01735329,
    -0.01870661, -0.02023316, -0.021971, -0.02397055,
    -0.02629965, -0.02905173, -0.03235971, -0.03641963,
    -0.04153193, -0.04818082, -0.05720523, -0.07018713,
    -0.09051862, -0.1270187, -0.2120234, -0.6365587};
};
#endif
