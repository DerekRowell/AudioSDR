/**************************************************************************************************
   AudioSDR.cpp

   Function: A single Teensy 3.6 Audio object for demodulation of direct conversion SDR receivers
             Based on the Teensy 3.x Audio library function structure.

   Author:   Derek Rowell
   Date:     August 17, 2019
   Version:  1.0

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

****************************************************************************************************/
//
#include "AudioSDR.h"
// ---
/*----------------------*
     ---  UPDATE----
  ----------------------*/
void AudioSDR::update(void) {
  audio_block_t *blockI, *blockQ;
  static float32_t bufferI[4*n_block] = {0.0};
  static float32_t bufferQ[4*n_block] = {0.0};
  static float32_t phase_SSB;
  static float32_t phase_AM;
  //
  blockI = receiveWritable(0);
  blockQ = receiveWritable(1);
  if (!blockI &&  blockQ) {
    release(blockQ);
    return;
  }
  if ( blockI && !blockQ) {
    release(blockI);
    return;
  }
  if (!blockI && !blockQ) return;
          /*********************************************************************
          *                         --- IMPORTANT ---                          * 
          *  The convention adopted for all SDR quadrature blocks is that      *
          *  the in-phase (I) channel is  on I2S input 0  (left), and the     *
          *  quadrature (Q) channel is on I2S input 1 (right).                 *
          *  Demodulation will not be successful if this condition is not met. *
          *********************************************************************/
  //  -----------------------------------------------
  //   --- Common processing for all demod modes ---
  //  -----------------------------------------------
  for (int i = 0; i < n_block; i++) {
    _Idata[i] = ((float)(blockI->data[i]) / 32767.0) * _inGainI;
    _Qdata[i] = ((float)(blockQ->data[i]) / 32767.0) * _inGainQ;
  }
  // --- Impulse Noise Blanker ---
  // Do noise blanking before any filtering
  if (NoiseBlankerisEnabled()) impulse_noise_blanker(_Idata, _Qdata);
  //
  // --- IF Pre-filter with (IIR) elliptical BPF
  // The bandwidth is determined by the current demod mode
  arm_biquad_cascade_df1_f32(&_IFfilterI, _Idata, _Idata, n_block);
  arm_biquad_cascade_df1_f32(&_IFfilterQ, _Qdata, _Qdata, n_block);
  //
  //  ----------------------
  //   --- DEMODULATION ---
  //  ----------------------
  // --- Hartley ("phase-shift") SSB/CW demodulation:
  if ((_mode==USBmode) || (_mode==LSBmode) || (_mode==CW_USBmode) || (_mode==CW_LSBmode)) {
    // --- Shift the IF band down to baseband
    phase_SSB = freq_shifter(_Idata, _Qdata, -_freq_shift, phase_SSB);
    //
    // Create a long (4 block) buffer for the length 257 FIR hilbert-transform FIR filter:
    for (int i = 0; i < n_block; i++) {
      bufferI[i]             = bufferI[  n_block + i];
      bufferI[  n_block + i] = bufferI[2*n_block + i];
      bufferI[2*n_block + i] = bufferI[3*n_block + i];
      bufferI[3*n_block + i] = _Idata[i];
      bufferQ[i]             = bufferQ[  n_block + i];
      bufferQ[  n_block + i] = bufferQ[2*n_block + i];
      bufferQ[2*n_block + i] = bufferQ[3*n_block + i];
      bufferQ[3*n_block + i] = _Qdata[i];
    }
    // --- Apply a length 257 FIR hilbert-transform filter to the Q channel
    //     Note: This compact FIR algorithm recognizes that the Hilbert coefficients
    //           are odd symmetric about the mid-point and non-zero for even indices,
    //           to reduce the required number of multiplications by a factor of four!
    for (int i = 0; i < n_block; i++) {
      _Qdata[i] = 0.0;
      for (int k = 0; k < hilbertFilterLength / 4; k++) {
        int indx1 = (3 * n_block + i) - (2 * k + 1);
        int indx2 = (3 * n_block + i) - hilbertFilterLength + 2 * (k + 1);
        _Qdata[i] += hilbert_coeffs[k] * (bufferQ[indx1] - bufferQ[indx2]); // Convolution
      }
      // Compensate the I channel output for the group delay introduced by the FIR filter:
      _Idata[i] = bufferI[3 * n_block + i - hilbertDelay];
    }
    //
    // --- Combine the I and Q channels to form the SSB demodulated audio:
    for (int i = 0; i < n_block; i++) {
      if      ((_mode == USBmode) || (_mode == CW_USBmode)) _audioOut[i] = _Idata[i] - _Qdata[i];
      else if ((_mode == LSBmode) || (_mode == CW_LSBmode)) _audioOut[i] = _Idata[i] + _Qdata[i];
    }
  }
  //
  // --- AM demodulation: ---
  else if (_mode == AMmode || _mode == SAMmode) {
    // --- Pre-filter with (IIR) elliptical BPF 8 kHz bandwidth
    arm_biquad_cascade_df1_f32(&_IFfilterI, _Idata, _Idata, n_block);
    arm_biquad_cascade_df1_f32(&_IFfilterQ, _Qdata, _Qdata, n_block);
    //            --- Synchronous AM (SAM) demodulation ---
    if (_mode == SAMmode) {
      SAMdemod(_Idata, _Qdata);
      for (int i = 0; i < n_block; i++) {
        _audioOut[i] = _Idata[i];
      }
    }
    // If the SAM PLL fails to lock, switch back to standard envelope AM demodulation.
    // --- Complex envelope AM demodulation:
    if ((_mode == AMmode) || ((_mode == SAMmode) && !_SAM_PLL_isLocked)) {
      // --- Shift IF signal down to baseband
      phase_AM = freq_shifter(_Idata, _Qdata, -_IF_center_freq, phase_AM);
      // --- Image rejection filter (IIR elliptical LPF):
      arm_biquad_cascade_df1_f32(&_AMimageI, _Idata, _Idata, n_block);
      arm_biquad_cascade_df1_f32(&_AMimageQ, _Qdata, _Qdata, n_block);
      // --- complex envelope AM demodulation:
      for (int i = 0; i < n_block; i++) {
        _audioOut[i] = fast_sqrt_f32(_Idata[i] * _Idata[i] + _Qdata[i] * _Qdata[i], 2);
      }
    }
  }
  // --------------------------------------------
  //  --- POST DEMODULATION AUDIO PROCESSING ---
  // --------------------------------------------
  // --- IIR Audio bandpass filter ---
  if (_audio_filter_is_enabled) audioFilter(_audioOut);
  //
  // --- Audio AGC0 ---
  if (_agc_is_enabled) agcProcessor(_audioOut);
  //
  // --- Adaptive-Least-Squares (ALS) notch/peaking filter ---
  if (_als_is_enabled) ALSfilter(_audioOut);
  //
  // --- Check for mute and apply output gain ---
  if (_isMuted) for (int i = 0; i < n_block; i++)  blockI->data[i] = 0;
  else          for (int i = 0; i < n_block; i++)  {
      blockI->data[i] = (int)(_output_gain * _audioOut[i] * 32767.0);
    }
  //
  // --- Transmit the processed mono audio on both output channels ---
  transmit(blockI, 0);
  transmit(blockI, 1);
  release(blockI);
  release(blockQ);
}
//               -------------------------------------------------

/*-----------------------------------------------*
*    --- AudioSDR GENERAL PUBLIC FUNCTIONS ---   *
*-----------------------------------------------*/
void AudioSDR::init(void) {
  arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw2700_coefs,  _audio_filter_state);
  arm_biquad_cascade_df1_init_f32(&_IFfilterI,    4, SSBpre_coefs,  _IFfilterStateI);
  arm_biquad_cascade_df1_init_f32(&_IFfilterQ,    4, SSBpre_coefs,  _IFfilterStateQ);
  arm_biquad_cascade_df1_init_f32(&_AMimageI,     4, AMimage_coefs, _AMimage_stateI);
  arm_biquad_cascade_df1_init_f32(&_AMimageQ,     4, AMimage_coefs, _AMimage_stateQ);
  agc_init();
  PLL_design_filter();
  initBlanker();
  setDemodMode(LSBmode);
  _isMuted = false;
}
// ---
float32_t AudioSDR::setDemodMode(int newMode) {
  _mode = newMode;
  if (_mode == USBmode) {
    _freq_shift = _IF_center_freq - _IF_bandwidth_SSB / 2.0;
    arm_biquad_cascade_df1_init_f32(&_IFfilterI, 4, SSBpre_coefs, _IFfilterStateI);
    arm_biquad_cascade_df1_init_f32(&_IFfilterQ, 4, SSBpre_coefs, _IFfilterStateQ);
  }
  else if (_mode == LSBmode) {
    _freq_shift = _IF_center_freq + _IF_bandwidth_SSB / 2.0;
    arm_biquad_cascade_df1_init_f32(&_IFfilterI, 4, SSBpre_coefs, _IFfilterStateI);
    arm_biquad_cascade_df1_init_f32(&_IFfilterQ, 4, SSBpre_coefs, _IFfilterStateQ);
  }
  else if (_mode == CW_USBmode) {
    _freq_shift = _IF_center_freq - _IF_bandwidth_CW / 2.0;
    arm_biquad_cascade_df1_init_f32(&_IFfilterI, 4, CWpre_coefs, _IFfilterStateI);
    arm_biquad_cascade_df1_init_f32(&_IFfilterQ, 4, CWpre_coefs, _IFfilterStateQ);
  }
  else if (_mode == CW_LSBmode) {
    _freq_shift = _IF_center_freq + _IF_bandwidth_CW / 2.0;
    arm_biquad_cascade_df1_init_f32(&_IFfilterI, 4, CWpre_coefs, _IFfilterStateI);
    arm_biquad_cascade_df1_init_f32(&_IFfilterQ, 4, CWpre_coefs, _IFfilterStateQ);
  }
  else if ((_mode == AMmode)  || (_mode == SAMmode)) {
    _freq_shift = _IF_center_freq;
    arm_biquad_cascade_df1_init_f32(&_IFfilterI, 4, AMpre_coefs, _IFfilterStateI);
    arm_biquad_cascade_df1_init_f32(&_IFfilterQ, 4, AMpre_coefs, _IFfilterStateQ);
  }
  // Return the mode dependent frequency offset to be applied to the rf quadrature oscillator
  return _freq_shift;
};
// ---
float32_t AudioSDR::getTuningOffset(void) {
  return _freq_shift;
}
//
int16_t   AudioSDR::getDemodMode(void)    {
  return _mode;
}
//--- Input gain and balance
void     AudioSDR::setInputGain( float32_t ingain) {
  if (ingain > 10.0) ingain = 10.0;
  if (ingain < 0.0)  ingain = 0.0;
  _inGain  = ingain;
  _inGainI = _inGain * _gainBalance;
  _inGainQ = _inGain;
}
//
void     AudioSDR::setIQgainBalance( float32_t balance) {
  _gainBalance = balance;
  _inGainI = _inGain * _gainBalance;
  _inGainQ = _inGain;
}//
void AudioSDR::setOutputGain(float32_t outGain) {
  _output_gain = outGain;
}
// --- Audio muting
void      AudioSDR::setMute(bool muted) {
  _isMuted = muted;
  if (_isMuted) _currentOutGain = 0.0f;
  else          _currentOutGain = _outGain;
}
//
bool     AudioSDR::getMute(void) {
  return _isMuted;
}
// --- IF tuning band limits
float32_t  AudioSDR::getBPFlower(void) {             // returns tuning-band lower cut-off frequency
  if      (_mode == USBmode || _mode == LSBmode)       return (_IF_center_freq - _IF_bandwidth_SSB / 2.0);
  else if (_mode == CW_USBmode || _mode == CW_LSBmode) return (_IF_center_freq - _IF_bandwidth_CW / 2.0);
  else if (_mode == AMmode || _mode == SAMmode)        return (_IF_center_freq - _IF_bandwidth_AM / 2.0);
  return 0.0;
}
//
float32_t  AudioSDR::getBPFupper(void) {             // returns tuning-band upper cut-off frequency
  if      (_mode == USBmode    || _mode == LSBmode)    return (_IF_center_freq + _IF_bandwidth_SSB / 2.0);
  else if (_mode == CW_USBmode || _mode == CW_LSBmode) return (_IF_center_freq + _IF_bandwidth_CW / 2.0);
  else if (_mode == AMmode     || _mode == SAMmode)    return (_IF_center_freq + _IF_bandwidth_AM / 2.0);
  return 0.0;
}
//               -------------------------------------------------

/*------------------------------------------------------*
*    --- IIR AUDIO OUTPUT FILTER PUBLIC FUNCTIONS----   *
*------------------------------------------------------*/
// --- Filtering operation, assumes that a filter has been selected
void AudioSDR::audioFilter(float32_t * buff) {
  float32_t tempBuff[n_block];
  for (int i = 0; i < n_block; i++) {
    tempBuff[i] = buff[i];
  }
  arm_biquad_cascade_df1_f32(&_audio_filter, tempBuff, buff, n_block);
}
//
// ---  Auxilliary functions for the Audio filter
void AudioSDR::enableAudioFilter(void)    {
  _audio_filter_is_enabled = true;
}
void AudioSDR::disableAudioFilter(void)   {
  _audio_filter_is_enabled = false;
}
int  AudioSDR::getAudioFilter(void)       {
  return _currentFilter;
}
void AudioSDR::setAudioFilter(int filter) {
  if      (filter == audioBypass) _audio_filter_is_enabled = false;
  else if (filter == audioAM)   arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw3900_coefs, _audio_filter_state);
  else if (filter == audioCW)   arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw470_coefs,  _audio_filter_state);
  else if (filter == audioWSPR) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, wspr_coefs,   _audio_filter_state);
  else if (filter == audio2100) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw2100_coefs, _audio_filter_state);
  else if (filter == audio2300) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw2300_coefs, _audio_filter_state);
  else if (filter == audio2500) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw2500_coefs, _audio_filter_state);
  else if (filter == audio2700) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw2700_coefs, _audio_filter_state);
  else if (filter == audio2900) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw2900_coefs, _audio_filter_state);
  else if (filter == audio3100) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw3100_coefs, _audio_filter_state);
  else if (filter == audio3300) arm_biquad_cascade_df1_init_f32(&_audio_filter, 4, bw3300_coefs, _audio_filter_state);
  _currentFilter = filter;
}
//               -------------------------------------------------

/*----------------------------------------------*
*         --- ALS NOTCH/PEAKING FILTER ----     *
*----------------------------------------------*/

/*-------------------------------------------------------------------------*
*  Note: In order to save execution time this implementation only updates  * 
*  the ALS filter coefficients every "iteration_count" iterations. This   * 
*  will slow convergence of the filter, but with iteration count = 4 the   * 
*  execution time is reduced from 730 usecs to 368 usecs.                  *
*-------------------------------------------------------------------------*/
void AudioSDR::ALSfilter( float32_t * buff) {
  const uint16_t  iteration_count = 4;
  uint16_t  count = 0;
  float32_t grad_j, als_out, err_out;
  // --- Prepare the length 256 FIR buffer
  for (int i = 0; i < n_block; i++) {
    _als_in[i]         = _als_in[n_block + i];
    _als_in[n_block + i] = buff[i];
  }
  // --- ALS FIR filter convolution with the current coefficients
  for (int i = n_block; i < 2 * n_block; i++) {
    als_out = 0.0;
    for (int j = 0; j < _M; j++)  als_out += _als_coeffs[j] * _als_in[(i - _delay) - j];
    err_out = _als_in[i] - als_out;                                // The filter "error"
    //
    // --- Update the filter coefficients every iteration_count iterations
    if (_als_is_adaptive) {
      if (count == 0) {
        for (int j = 0; j < _M; j++) {
          grad_j = err_out * _als_in[i - _delay - j];    // The optimization gradient
          _als_coeffs[j] += _lambda * grad_j;            // Compute new filter coefficients
        }
      }
      count = (++count) % iteration_count;
    }
    if (_als_is_notch) buff[i - n_block] = err_out; // For a notch filter the output is the "error"
    else               buff[i - n_block] = als_out; // For a bandpass (peak) filter the output is the response
  }
}
//

// ---  Auxilliary functions for the ALS filter
void AudioSDR::disableALSfilter(void)       {
  _als_is_enabled  = false; // Disable the notch filter and set in passthrough mode
}
void AudioSDR::setALSfilterNotch(void)      {
  _als_is_notch    = true; // Set the ALS filter in "notch" mode
}
void AudioSDR::setALSfilterPeak(void)   {
  _als_is_notch    = false; // Set the ALS filter in "noise reduction" mode
}
void AudioSDR::setALSfilterAdaptive(void)   {
  _als_is_adaptive = true; // Enable adaptive updating of filter coefficients
}
void AudioSDR::setALSfilterStatic(void)     {
  _als_is_adaptive = false; // Disable adaptive updating of filter coefficients
}
boolean AudioSDR::ALSfilterIsEnabled(void)  {
  return _als_is_enabled; // Return true if ALS filter is running
}
boolean AudioSDR::ALSfilterIsNotch(void)    {
  return _als_is_notch; // Return true if the ALS filter in "notch" mode
}
boolean AudioSDR::ALSfilterIsPeak(void) {
  return !(_als_is_notch); // Return true if the ALS filter in "bandpass" mode
}
boolean AudioSDR::ALSfilterIsAdaptive(void) {
  return _als_is_adaptive; // Return true if the ALS filter in adaptive mode
}
// ---
void AudioSDR::enableALSfilter(void) {       // Enable and initialize the notch filter
  _als_is_enabled = true;
  for (int i = 0; i < n_block; i++) {
    _als_coeffs[i]     = 0.0;
    _als_in[i]         = 0.0;
    _als_in[i + n_block] = 0.0;
  }
}
// ---
void AudioSDR::setALSfilterParams(unsigned int m, float Lambda, float Delay) {  // Modify ALS parameters
  _M = m;
  if (_M >= n_block) _M = n_block; // FIR filter length
  _lambda = Lambda;                // Filter coefficient update gain
  _delay  = Delay;                 // "Decorrelation" delay
}
//               -------------------------------------------------

/*------------------------------------*
*         --- AGC PROCESSOR ----      *
*------------------------------------*/
void AudioSDR::agcProcessor( float32_t * buff) {   //  Execution time: 33 usecs
  float32_t output;
  for (int i = 0; i < 128; i++) {
    // Look for a new peak value
    _absVal = fabs(buff[i]);
    if (_absVal > 1.0) _absVal = 1.0;
    if (_absVal > _old_absVal) {                                  // Attack
      _absVal = _agc_alphaAttack * _old_absVal + _agc_betaAttack * _absVal;
      _old_absVal = _absVal;
      _agc_hang_counter = _agc_hangCount;                       // Reset the counter
      _agc_gain = agc_staticCompressor((int)(_absVal * 32767.0));
    }
    else {
      if (_agc_hang_counter > 0) _agc_hang_counter--;             // Do nothing, we are "hanging"
      else {                                                      // Release
        _absVal = _agc_alphaRelease * _old_absVal + _agc_betaRelease * _absVal;
        _old_absVal = _absVal;
        _agc_gain = agc_staticCompressor((int)(_absVal * 32767.0));
      }
    }
    _agc_is_active = (_agc_gain < 0.99);
    // --- Apply make-up gain
    output = _agc_gain * _agc_staticGain * buff[i];
    output = (output >  1.0) ?  1.0 : output;
    output = (output < -1.0) ? -1.0 : output;
    buff[i] = output;
  }
}
//
//   --- Auxilliary functions for the AGC processor-
void AudioSDR::agc_init(void) {
  // Explanation:   make-up gain = - threshold*(1 - slope) dB
  _agc_threshold     = -60.0;                      // set AGC threshold at -60 dB
  _agc_slope         =  0.1;                       // set slope of the active region
  _agc_kneeWidth     =  2.0;                       // width of the soft knee region (dB)
  _agc_attackTime    =  5.0;                       // Time-constant (ms) for the attack
  _agc_releaseTime   =  500.0;                     // Time-constant (ms) for the release
  _agc_hangTime      =  100.0;                     // Time (ms) to dwell after a decrease in signal level
  _agc_hangCount     =  AUDIO_SAMPLE_RATE_EXACT * (_agc_hangTime / 1000.0);;
  _agc_alphaAttack   =  exp(log(0.1) / (AUDIO_SAMPLE_RATE_EXACT * _agc_attackTime / 1000.0));
  _agc_betaAttack    =  1.0 - _agc_alphaAttack;
  _agc_alphaRelease  =  exp(log(0.1) / (AUDIO_SAMPLE_RATE_EXACT * _agc_releaseTime / 1000.0));
  _agc_betaRelease   =  1.0 - _agc_alphaRelease;
  _agc_is_enabled    =  true;
  //
  agc_createLookupTable();
  //    _agc_staticGain = 1.0/agc_staticCompressor(32767);
  for (int i = 0; i < 3; i++) _agc_history[i] = 0.0;
}
//                            ---------------------
void AudioSDR::agc_createLookupTable() {
  int16_t  agc_tableSize = 129;
  float32_t input;
  float32_t inDB;
  float32_t outDB;
  float32_t linLowerKnee = expf(2.3025 * (_agc_threshold - _agc_kneeWidth / 2.0) / 20.0);
  float32_t linUpperKnee = expf(2.3025 * (_agc_threshold + _agc_kneeWidth / 2.0) / 20.0);
  // Create a table of length (tableSize + 1) for interpolation in the final segment
  for (int i = 0; i < agc_tableSize + 1; i++) {
    input = float(i) / 128.0;
    inDB = 6.026 * log2_approx_f32(input);
    if      (input < linLowerKnee) _agc_gainLookup[i] = 1.0;             // below knee region                                                             // below knee region
    else if (input > linUpperKnee) {
      outDB = (_agc_threshold + (inDB - _agc_threshold) * _agc_slope);   // above knee region
      _agc_gainLookup[i] = expf(2.3025 * (outDB - inDB) / 20.0);
    } else {                                                            // within knee region
      outDB = inDB + ((_agc_slope - 1.0) * (inDB - _agc_threshold + _agc_kneeWidth / 2.0)
                      * (inDB - _agc_threshold + _agc_kneeWidth / 2.0)) / (2.0 * _agc_kneeWidth);
      _agc_gainLookup[i] = expf(2.3025 * (outDB - inDB) / 20.0);
    }
  }
}

// ---
float32_t AudioSDR::agc_staticCompressor(uint16_t input) {
  float32_t delta;
  float32_t output;
  uint16_t indx, frac;
  indx  = input >> 8;
  if (indx > 127) indx = 127;
  frac  = input & 0xFF;
  delta = float(frac) / 256.0;
  // Linear interpolation
  output = (_agc_gainLookup[indx] + (_agc_gainLookup[indx + 1] - _agc_gainLookup[indx]) * delta);
  return output;
}
void  AudioSDR::enableAGC(void)    {
  _agc_is_enabled = true;
}
void  AudioSDR::disableAGC(void)   {
  _agc_is_enabled = false;
}
void  AudioSDR::setAGCstaticGain( float32_t outputgain) {
  _agc_staticGain = outputgain;
}
bool  AudioSDR::AGCisEnabled(void) {
  return _agc_is_enabled;
}
bool  AudioSDR::AGCisActive(void)  {
  return _agc_is_active;
}
// ---
void  AudioSDR::setAGCthreshold(float32_t threshold) {
  _agc_threshold = threshold;
  agc_createLookupTable();
}
// ---
void  AudioSDR::setAGCslope( float32_t slope) {
  _agc_slope = slope;
  agc_createLookupTable();
}
// ---
void AudioSDR::setAGCmode(int16_t mode) {
  if      (mode == AGCoff) disableAGC();
  else if (mode == AGCfast) {
    setAGCattackTime(2.0);  // "FAST"
    setAGCreleaseTime(100.0);
    setAGChangTime(100.0);
    enableAGC();
  }
  else if (mode == AGCmedium) {
    setAGCattackTime(5.0);  // "MEDIUM"
    setAGCreleaseTime(250.0);
    setAGChangTime(500.0);
    enableAGC();
  }
  else if (mode == AGCslow) {
    setAGCattackTime(10.0);  // "SLOW"
    setAGCreleaseTime(500.0);
    setAGChangTime(2000.0);
    enableAGC();
  }
}
// ---
void  AudioSDR::setAGCkneeWidth( float32_t kneewidth) {
  _agc_kneeWidth = kneewidth;
  agc_createLookupTable();
}
//
void  AudioSDR::setAGCattackTime( float32_t attack) {
  _agc_attackTime   = attack;                     // msec
  _agc_alphaAttack  = exp(log(0.1) / (AUDIO_SAMPLE_RATE_EXACT * _agc_attackTime / 1000.0));
  _agc_betaAttack   = 1.0 - _agc_alphaAttack;
}
//
void  AudioSDR::setAGCreleaseTime(  float32_t release) {
  _agc_releaseTime  = release;                     // msec
  _agc_alphaRelease = exp(log(0.1) / (AUDIO_SAMPLE_RATE_EXACT * _agc_releaseTime / 1000.0));
  _agc_betaRelease  = 1.0 - _agc_alphaRelease;
}
//
void  AudioSDR::setAGChangTime( float32_t hang) {
  _agc_hangTime  = hang;                           // msec
  _agc_hangCount = _agc_hangTime * AUDIO_SAMPLE_RATE_EXACT / 1000.0;
}
//
float32_t  AudioSDR::getAGCthreshold(void)    {
  return _agc_threshold;
}
float32_t  AudioSDR::getAGCslope(void)        {
  return _agc_slope;
}
float32_t  AudioSDR::getAGCkneeWidth(void)    {
  return _agc_kneeWidth;
}
float32_t  AudioSDR::getAGCattack(void)       {
  return _agc_attackTime;
}
float32_t  AudioSDR::getAGCrelease(void)      {
  return _agc_releaseTime;
}
float32_t  AudioSDR::getAAGalphaAttack(void)  {
  return _agc_alphaAttack;
}
float32_t  AudioSDR::getAGCbetaAttack(void)   {
  return _agc_betaAttack;
}
float32_t  AudioSDR::getAGCalphaRelease(void) {
  return _agc_alphaRelease;
}
float32_t  AudioSDR::getAGCbetaRelease(void)  {
  return _agc_betaRelease;
}
float32_t  AudioSDR::getAGClookup(int i)      {
  return _agc_gainLookup[i];
}
float32_t  AudioSDR::getAGCstaticGain(void)   {
  return _agc_staticGain;
}
//               -------------------------------------------------

/*----------------------------------------*
*        --- IMPULSE NOISE BLANKER  ----  *
*----------------------------------------*/
void AudioSDR::impulse_noise_blanker(float * Ibuff, float * Qbuff) {  //  Execution time: 76 usecs
  const uint16_t nTrans = 7;
  const float transition_dn[nTrans] = {0.933, 0.750, 0.500, 0.250, 0.067, 0.000, 0.000};
  const float transition_up[nTrans] = {0.000, 0.000, 0.067, 0.250, 0.500, 0.750, 0.933};
  // ---
  _nb_impulseDetected = false;
  for (int i = 0; i < n_block; i++) {
    _BufferI[i]           = _BufferI[n_block + i];
    _BufferI[n_block + i]   = _BufferI[2 * n_block + i];
    _BufferI[2 * n_block + i] = Ibuff[i];
    //
    _BufferQ[i]           = _BufferQ[n_block + i];
    _BufferQ[n_block + i]   = _BufferQ[2 * n_block + i];
    _BufferQ[2 * n_block + i] = Qbuff[i];
    //
    _mask[i]              = _mask[n_block + i];
    _mask[n_block + i]      = _mask[2 * n_block + i];
    _mask[2 * n_block + i]    = 1.0;
  }
  // --- Work on the second block (the first and third blocks are to allow for transients at the block ends)
  //     Create a "mask"  (1 or 0) - (set to 0 for the duration of an impulse)
  for (int i = n_block - 50; i < 2 * n_block; i++) {
    _nb_Mag    = fast_sqrt_f32(_BufferI[i] * _BufferI[i] + _BufferQ[i] * _BufferQ[i], 1); // envelope
    if (_nb_Mag > _nb_AvgMag * _nb_Threshold) {                              // An impulse is detected
      for (int j = -(_pre_mask); j < (_post_mask + 1); j++) _mask[i + j] = 0.0; // Zero out the mask
      _nb_impulseDetected = true;
    }
    _nb_AvgMag = _nb_Alpha * _nb_AvgMag + _nb_Beta * _nb_Mag;                // Update (smooth) the signal average
  }
  // --- Scan the second buffer again, adding raised cosine transitions at each pulse edge in the mask
  for (int i = n_block; i < 2 * n_block; i++) {
    if ((_mask[i] == 1.0) && (_mask[i - 1] == 0.0)) {                       // Transition down to blanking
      for (int j = 0; j < nTrans; j++)  _mask[i - nTrans + j] = transition_dn[j];
    }
    else if ((_mask[i - 1] == 0.0) && (_mask[i] == 1.0)) {                  // Transition up from blanking
      for (int j = 0; j < nTrans; j++)  _mask[i + j - 1] = transition_up[j];
    }
  }
  // --- Use the mask to blank the signal and return the previous buffer
  for (int i = 0; i < n_block; i++) {
    Ibuff[i] = _mask[i] * _BufferI[i];
    Qbuff[i] = _mask[i] * _BufferQ[i];
  }
}
// ---
// --- Auxilliary functions for the impulse noise balanker
void AudioSDR::enableNoiseBlanker(void)  {
  _nb_is_enabled = true;
  initBlanker();
}
void AudioSDR::disableNoiseBlanker(void) {
  _nb_is_enabled = false;
}
bool AudioSDR::NoiseBlankerisEnabled()   {
  return _nb_is_enabled;
}
bool AudioSDR::NoiseBlankerDetection()   {
  return _nb_impulseDetected;
}
void AudioSDR::setNoiseBlankerThreshold   (float32_t thresh) {
  _nb_Threshold = thresh;
  initBlanker();
}
//
void AudioSDR::setNoiseBlankerThresholdDb (float32_t thresh) {  // dB to ratio
  _nb_Threshold = powf(10.0, (thresh / 20.0));
  initBlanker();
}
//
void AudioSDR::initBlanker(void) {
  for (int i = 0; i < 3 * n_block; i++) {
    _BufferI[i] = 0.0;
    _BufferQ[i] = 0.0;
    _mask[i]    = 1.0;
  }
}
//               -------------------------------------------------

/*---------------------------------------------------*
*      --- SYNCHRONOUS AM DEMODULATOR (SAM) ----     *
*---------------------------------------------------*/
void AudioSDR::SAMdemod(float* buffI, float* buffQ) {  // Execution time: 112 usecs
  
  const  float32_t twoPI = 2.0*PI;
  static float32_t yReal = 0.0;
  static float32_t yImag = 0.0;
  static float32_t prev_phase_error = 0.0;
  static float32_t prev_phase_err_filt = 0.0;
  float32_t phase_error = 0.0;;
  float32_t phase_err_filt;
  float32_t deltaReal, deltaImag;
  float32_t tempI, tempQ;
  float32_t xReal, xImag;
  // --- Quadrature Phase-locked loop:
  for (int i = 0; i < 128; i++) {
    xReal = buffI[i];
    xImag = buffQ[i];
    /*----------------------------------------------------------------------------------*
    *  Phase-error detector - let the quadrature input waveform be                      *
    *               x(t) = A*e^(j(phiX(t))   =   A*cos(phiX(t)) + jA*sin(phiX(t))       *
    *   and let the feedback estimated waveform be                                      *
     *              y(t) = B*e^(j(phiY(t))   =   B*cos(phiY(t)) + jB*sin(phiX(t)),      *
    *   then form the complex product                                                   *
    *       delta = x(t)*conj(y(t)) = AB*e^(j*phiX(t))*e^(-j*phiY(t))                   *
    *             = AB*e^(j*(phiX(t)-phiY(t)))                                          *
    *             = AB*cos(phiX(t)-phiY(t)) + j*AB*sin(phiX(t)-phiY(t))                 *
    *   The phase error we seek is found from the ratio of the real and imaginary parts *
    *                           imag           [sin(phiX(t)-phiY(t))]                   *
    *     phi_error(t) = arctan(----) = arctan[ -------------------- ]                  *
    *                           real           [cos(phiX(t)-phiY(t)]                    *
    *----------------------------------------------------------------------------------*/
    deltaReal   = xReal * yReal + xImag * yImag;           // Complex conjugate product
    deltaImag   = xImag * yReal - xReal * yImag;           // to form complex delta.
    // Note: WE don't need an accurate atan2 value since this is just the error signal
    // in a feedback loop and will be driven to zero.
    phase_error = approx_atan2_f32(deltaImag, deltaReal);  // radians
    /*----------------------------------------------------------------------------------*
    *    Let the PLL filter be in the form of a PI (proportional+integral) controller   *
    *    ie    y_out = Kp*error + Ki*integral(error)                                    *
    *    where Kp and Ki are the proportional and integral gains.                       *
    * ---------------------------------------------------------------------------------*/
    phase_err_filt   = _K_1 * phase_error + _K_2 * prev_phase_error;
    _PLLfreq         = _alpha_freq * _PLLfreq + _beta_freq * (phase_err_filt * _f_conversion);
    prev_phase_error = phase_error;
    /*---------------------------------------------------------*
    *    Use Tustin's (trapezoidal) integration to estimate    *
    *    the phase from the estimated frequency                *
    *--------------------------------------------------- -----*/
    _phase_est += (phase_err_filt + prev_phase_err_filt) / 2.0;
    prev_phase_err_filt  = phase_err_filt;
    // --- Compute the outputs
    if (_phase_est >=  PI) _phase_est -= twoPI;    // Wrap phase
    if (_phase_est <  -PI) _phase_est += twoPI;
    _SAM_PLL_isLocked = ((_PLLfreq > _lock_freq_low) && (_PLLfreq < _lock_freq_high));
    yReal =  cos_f32(_phase_est);
    yImag =  sin_f32(_phase_est);
    if (_SAM_PLL_isLocked) {
      // --- Shift data buffere to baseband
      tempI     =  _Idata[i];
      tempQ     =  _Qdata[i];
      _Idata[i] =  tempI * yReal + tempQ * yImag;
      _Qdata[i] = -tempI * yImag + tempQ * yReal;
    }
  }
}

// --- Auxilliary functions for the SAM demodulator
float32_t AudioSDR::getSAMfrequency(void) {
  return _PLLfreq;
}
boolean   AudioSDR::getSAMphaseLockStatus(void) {
  return _SAM_PLL_isLocked;
}
void AudioSDR::PLL_design_filter(void) {
  _Kp  = (1 / (_Kd * _Ko)) * 4.0 * _zeta * _Bn / (_zeta + 1 / (4.0 * _zeta));
  _Ki  = (1 / (_Kd * _Ko)) * 4.0 * _Bn * _Bn / ((_zeta + 1 / (4.0 * _zeta)) * (_zeta + 1 / (4.0 * _zeta)));
  _K_1 = _Kp + _Ki;
  _K_2 = _Ki;
}
//------------------------------------------------------------------------------------------------------------

 
    
    
