/**************************************************************************************************
  AudioSDR.h

  Function: A single Teensy 3.6 Audio block for demodulation of direct conversion SDR receivers
            Based on Teensy 3.x Audio library functions Copyright(c) 2014, Paul Stoffregen

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
#ifndef audio_sdr_h_
#define audio_sdr_h_
#include "core_pins.h"
#include "AudioStream.h"
#include "utility/dspinst.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "Arduino.h"
//#include "sdrMath.h"
#include "Streaming.h"

// -- Demodulation mode definitions
#define LSBmode      0
#define USBmode      1
#define CW_LSBmode   2
#define CW_USBmode   3
#define AMmode       4
#define SAMmode      5
#define WSPRmode     6
// -- arm_math definitions
#define FORWARD      0
#define INVERSE      1
#define BIT_REVERSE  1
// --  IIR biquad audio filters
#define audioAM      0
#define audioCW      1
#define audioWSPR    2
#define audio2100    3
#define audio2300    4
#define audio2500    5
#define audio2700    6
#define audio2900    7
#define audio3100    8
#define audio3300    9
#define audioBypass  10
// ---
#define AGCoff       0
#define AGCfast      1
#define AGCmedium    2
#define AGCslow      3
// ---
#define n_block     128
// ---
class AudioSDR : public AudioStream {
  public:
    AudioSDR() : AudioStream(2, inputQueueArray) {
      init();
    }
    //---
    virtual void update(void);
    //
    /*--------------------------------------------*
             ---- FUNCTION TEMPLATES ----
      --------------------------------------------*/
    // ---
    // General public functions
    void      init(void);
    void      setMute(bool);
    void      setInputGain(float);
    void      setIQgainBalance(float);
    int16_t   getDemodMode(void);
    float32_t setDemodMode(int);
    float32_t getBPFlower(void);
    float32_t getBPFupper(void);
    float32_t getTuningOffset(void);
    boolean   getMute(void);
    //
    // --- IIR Audio Output Filters
    void      enableAudioFilter(void);
    void      disableAudioFilter(void);
    int       getAudioFilter(void);
    void      setOutputGain(float);
    void      setAudioFilter(int);
    //
    // --- ALS notch/peaking filter
    void      enableALSfilter(void);
    void      disableALSfilter(void);
    void      setALSfilterNotch(void);
    void      setALSfilterPeak(void);
    void      setALSfilterAdaptive(void);
    void      setALSfilterStatic(void);
    void      setALSfilterParams(unsigned int, float, float);
    boolean   ALSfilterIsEnabled(void);
    boolean   ALSfilterIsNotch(void);
    boolean   ALSfilterIsPeak(void);
    boolean   ALSfilterIsAdaptive(void);
    //
    // --- AGC processor
    void      enableAGC(void);
    void      disableAGC(void);
    bool      AGCisEnabled(void);
    bool      AGCisActive(void);
    void      setAGCthreshold(float);
    void      setAGCslope(float);
    void      setAGCmode(int16_t);
    void      setAGCkneeWidth(float);
    void      setAGCattackTime(float);
    void      setAGCreleaseTime(float);
    void      setAGChangTime( float);
    void      setAGCstaticGain(float);
    float32_t getAGCthreshold(void);
    float32_t getAGCslope(void);
    float32_t getAGCkneeWidth(void);
    float32_t getAGCattack(void);
    float32_t getAGCrelease(void);
    float32_t getAAGalphaAttack(void);
    float32_t getAGCbetaAttack(void);
    float32_t getAGCalphaRelease(void);
    float32_t getAGCbetaRelease(void);
    float32_t getAGClookup(int);
    float32_t getAGCstaticGain(void);
    boolean   getAGCmakeUpMode(void);
    //
    // --- Impulse Noise Blanker
    void      enableNoiseBlanker(void);
    void      disableNoiseBlanker(void);
    void      setNoiseBlankerThreshold(float);
    void      setNoiseBlankerThresholdDb(float);
    bool      NoiseBlankerisEnabled(void);
    bool      NoiseBlankerDetection(void);
    //
    // --- Synchronous AM detector (SAM)
    float32_t getSAMfrequency(void);
    boolean   getSAMphaseLockStatus(void);
    //
    /*--------------------------------------------------*
               ---- Private Variables ----
      --------------------------------------------------*/
  private:
    // --- General private variables ---
    audio_block_t *inputQueueArray[2];
    const float32_t _IF_center_freq   = 6890.0;
    const float32_t _IF_bandwidth_SSB = 3000.0;
    const float32_t _IF_bandwidth_CW  = 1000.0;
    const float32_t _IF_bandwidth_AM  = 8500.0;
    float32_t _audioOut[128];
    float32_t _Idata[128];
    float32_t _Qdata[128];
    float32_t _inGain          = 1.0f;
    float32_t _inGainI         = 1.0f;
    float32_t _inGainQ         = 1.0f;
    float32_t _gainBalance     = 1.0f;
    float32_t _output_gain     = 1.0;
    float32_t _outGain         = 1.0f;
    float32_t _currentOutGain  = 1.0f;
    float32_t _dc_offset       = 0.0;
    float32_t _freq_shift;
    uint16_t  _mode            = 0;
    bool      _isMuted = true;
    // --- IIR Filters
    arm_biquad_casd_df1_inst_f32   _IFfilterI;
    arm_biquad_casd_df1_inst_f32   _IFfilterQ;
    arm_biquad_casd_df1_inst_f32   _AMimageI;
    arm_biquad_casd_df1_inst_f32   _AMimageQ;
    arm_biquad_casd_df1_inst_f32   _audio_filter;
    //
    //   --- arm_math IIR filter state vectors
    float32_t _IFfilterStateI[16]     = {0.0};
    float32_t _IFfilterStateQ[16]     = {0.0};
    float32_t _AMimage_stateI[16]     = {0.0};
    float32_t _AMimage_stateQ[16]     = {0.0};
    float32_t _audio_filter_state[16] = {0.0};
    //
    //--- Private variables for the ALS filter
    int16_t     _M       = 55;                     // FIR filter length
    int16_t     _delay   = 3;                      // "Decorrelation" delay
    float32_t   _lambda  = 0.5;                    // Coefficient update gain
    float32_t   _als_in[2 * n_block];              // Input buffer
    float32_t   _als_coeffs[n_block];              // FIR filter coefficients
    boolean     _als_is_enabled  = false;
    boolean     _als_is_notch    = true;           // Default operation is a notch filter
    boolean     _als_is_adaptive = true;
    //
    //--- Private variables for the AGC processor
    float32_t    _twoPI = 2.0 * PI;
    float32_t    _agc_history[3];
    float32_t    _agc_alphaAttack;
    float32_t    _agc_alphaRelease;
    float32_t    _agc_attackTime;
    float32_t    _agc_betaAttack;
    float32_t    _agc_betaRelease;
    float32_t    _agc_control;
    float32_t    _agc_gain;
    float32_t    _agc_prevGain;
    float32_t    _agc_gainLookup[129];
    float32_t    _agc_hangTime = 100.0;
    float32_t    _agc_kneeWidth;
    float32_t    _agc_staticGain = 10.0;
    float32_t    _agc_slope;
    float32_t    _agc_releaseTime;
    float32_t    _agc_threshold;
    float32_t    _absVal;
    float32_t    _old_absVal;
    uint32_t     _agc_hangCount;
    uint32_t     _agc_hang_counter;
    boolean      _agc_is_active   = true;
    boolean      _agc_is_enabled  = true;
    boolean      _agc_makeUp      = false;
    //
    // --- Private variables for the impulse noise Blanker
    float32_t   _BufferI[3 * n_block];
    float32_t   _BufferQ[3 * n_block];
    float32_t   _mask[3 * n_block];
    float32_t   _nb_Alpha      = 0.995;
    float32_t   _nb_Beta       = (1.0 - _nb_Alpha);
    float32_t   _nb_Threshold  = 1.2;
    float32_t   _nb_Mag        = 0.0;
    float32_t   _nb_AvgMag     = 10.0;
    int16_t     _pre_mask      = 10;
    int16_t     _post_mask     = 10;
    boolean     _nb_is_enabled = true;
    boolean     _nb_impulseDetected = false;
    //
    // ------ Private variables for the SAM demodulator
    const float32_t _alpha_freq       = 0.995;
    const float32_t _beta_freq        = 1.0 - _alpha_freq;
    const float32_t _f_conversion     = (AUDIO_SAMPLE_RATE_EXACT / _twoPI);
    float32_t _phase_est        = 0.0;        // PLL estimated output phase
    float32_t _lock_freq_low    = _IF_center_freq - 3000.0;
    float32_t _lock_freq_high   = _IF_center_freq + 3000.0;
    float32_t _lock_phase_low   = _lock_freq_low / _f_conversion;
    float32_t _lock_phase_high  = _lock_freq_high / _f_conversion;
    float32_t _PLLfreq          = _lock_freq_low - 2000.0;
    boolean   _PLLisEnabled     = true;
    boolean   _SAM_PLL_isLocked = false;
    // --- PLL P-I (proportional + integral) filter design parameters
    float32_t _Bn    = 9000.0 / AUDIO_SAMPLE_RATE_EXACT; // closed-loop PLL "noise-bandwidth"
    float32_t _zeta  = 2.0;                             // PLL damping factor - 0.707
    float32_t _Kd    = 1.0f;                            // phase detector gain
    float32_t _Ko    = 1.0f;                            // oscillator gain
    // --- Derived parameters
    float32_t _Kp    = 0.0;                             // proportional gain
    float32_t _Ki    = 0.0;                             // integral gain
    float32_t _K_1   = _Kp + _Ki;;                        // proportional gain
    float32_t _K_2   = _Ki;                              // integral gain
    //
    // Private variables for the audio output filters
    int16_t _currentFilter;
    boolean _audio_filter_is_enabled;
    // ---------------- End of private variables ----------

    // ---------------- Private function templates --------
    // --- Initialize the SDR
    void initSDR(void);
    // --- ALS notch/peaking filter
    void ALSfilter(float*);
    // --- AGC processor
    void agcProcessor(float*);//
    void agc_init(void);
    void agc_createLookupTable(void);
    float32_t agc_staticCompressor(uint16_t);
    // --- IIR audio output filter
    void audioFilter(float*);
    // --- Impulse noise blanker
    void impulse_noise_blanker(float*, float*);
    void initBlanker(void);
    // --- SAM demodulation
    void SAMdemod(float*, float*);
    void PLL_design_filter(void);
    // --------------- END OF SDR PRIVATE STUFF **********


    /* --------------------------------------------------------------------*
        The following are a set of fast and/or approximate math function.
        They usually reside in a separate library but are included here to
        make the audioSDR package compact.
      ---------------------------------------------------------------------*/
    /*---------------------------------------------------*
                       --- SINE ---
      ---------------------------------------------------*/
    float32_t sin_f32(float Phase) {
      const float32_t twoPI = 2.0 * PI;
      float32_t val1, val2;
      uint16_t  index, delta, intPhase;
      if (Phase >= twoPI) Phase -= twoPI;
      if (Phase <  0.0)   Phase += twoPI;
      intPhase = (long)(Phase * 65535.0 / twoPI);
      index    = intPhase >> 8;                             // index into lookup table
      delta    = intPhase & 0xFF;                           // remainder
      val1     = sine_table_f32[index];
      val2     = sine_table_f32[index + 1];                 // table has 257 entries
      return val1 + (((val2 - val1) * (float)delta) / 256.0); // linear interpolation
    }

    /*--------------------------------------------------*
                        --- COSINE ---
      --------------------------------------------------*/
    float32_t cos_f32(float Phase) {
      return sin_f32(Phase + PI / 2.0);
    }

    /*---------------------------------------------------*
                       --- ATAN2 ---
      ---------------------------------------------------*/
    // Polynomial approximating arctangenet over the range -1,1.
    // Max error < 0.005 (or 0.29 degrees)
    float32_t approx_atan_f32(float32_t z) {
      const float32_t n1 =  0.97239411f;
      const float32_t n2 = -0.19194795f;
      return (n1 + n2 * z * z) * z;
    }
    // ---
    float32_t approx_atan2_f32(float y, float x) {
      const float32_t halfPI = 0.5 * PI;
      if (x != 0.0)  {                                               // Untangle the quadrants
        if (fabsf(x) > fabsf(y)) {
          float32_t z = y / x;
          if (x > 0.0)        return approx_atan_f32(z);           // atan2(y,x) = atan(y/x) if x > 0
          else if (y >= 0.0)  return approx_atan_f32(z) + PI;      // atan2(y,x) = atan(y/x) + PI if x < 0, y >= 0
          else                return approx_atan_f32(z) - PI;      // atan2(y,x) = atan(y/x) - PI if x < 0, y < 0
        } else {                                                   // Use property atan(y/x) = PI/2 - atan(x/y) if |y/x| > 1.
          float32_t z = x / y;
          if (y > 0.0)        return -approx_atan_f32(z) + halfPI; // atan2(y,x) = PI/2 - atan(x/y) if |y/x| > 1, y > 0
          else                return -approx_atan_f32(z) - halfPI; // atan2(y,x) = -PI/2 - atan(x/y) if |y/x| > 1, y < 0
        }
      } else {
        if (y > 0.0)         return  halfPI;                       // x = 0, y > 0
        else if (y < 0.0)    return -halfPI;                       // x = 0, y < 0
      }
      return 0.0f;                                                 // x,y = 0. Could return NaN instead.
    }

    /*---------------------------------------------------------------------------------------------*
                             --- FAST SQUARE ROOT (f32 only)  ---
       Interprets the structure of the IEEE 754 single precision floating point format to
       approximate an integer square root, which is then used to seed a small numder of
       Newton-Raphson iterations.
       The basic algorithm is described on Wikipedia (search for "fast square root approximation")
     *                                                                                             *
       To justify the following code, prove that
       ((((val_int/2^m) - b)/2) + b)*2^m = ((val_int - 2^m)/2) + ((b + 1)/2)*2^m)
       where b = exponent bias
             m = number of mantissa bits
             a is an empirically found constant to reduce the average error
     *                                                                                             *
        Suggestion: Let n_iter = 1, or 2
     *                                                                                             *
     *    ***********************************************************************************      *                                                                                            *
     *    *   IMPORTANT NOTE: This algorithm breaks down the IEEE standard f_32 data word   *      *
     *    *   format to extract the mantissa and exponent directly from the 32 bits.        *      *
     *    *   The bit manipulation code to do this, while widely used, does not obey the    *      *
     *    *   strict parsing rules of C++ and generates a compiler warning.                 *      *
     *    *   It may not work on all systems, but it does work on the Teensy 3.6 (with the  *      *
     *    *   compiter warning.                                                             *      *
     *    ***********************************************************************************      *
      ---------------------------------------------------------------------------------------------*/
    float32_t fast_sqrt_f32(float x, int n_iter) {
      float32_t out;
      const int32_t a = -0x4B0D2;
      uint32_t val_int = *(int*)&x;   // Same bits as in the float, but as an int
      val_int = (1 << 29) + (val_int >> 1) - (1 << 22) + a;
      out = *(float*)&val_int;
      // Use n_iter stages of Newton-Raphson to improve the estimate
      for (int i = 0; i < n_iter; i++)  out = 0.5 * (out + x / out);
      return out;
    }

    /*-----------------------------------------------------------------------------*
                    ---- FAST (APPROXIMATE) COMPLEX MAGNITUDE  ---
       Uses the "alpha max plus beta min"  algorithm:
             http://dspguru.com/dsp/tricks/magnitude-estimator/
       as an approximate envelope detector for quadrature waveforms, where the
       true magnitude is defined:
                  magnitude = sqrt(I^2 + Q^2)
     *                                                                             *
       The algorithm uses a pair of constants (alpha and beta) to approximate
       the magnitude:
            approx_magnitude ~= alpha*max(|I|,|Q|) + beta*min(|I|,|Q|),
       where alpha and beta have been chosen to minimize the error according
       to some error criterion.
     *                                                                             *
       Note: This algorithm is only accurate to within a few percent of the
              true value and should be used with caution.
     *                                                                             *
       ----------------------------------------------------------------------------*/
    float32_t approx_magnitude(float inphase, float quadrature, int crit) {
      //         Criteria: - 0 - "Minimum RMS Error"
      //                   - 1 - "Minimum Peak Error"
      //                   - 2 - "Minimum RMS Error with Avg=0"
      float32_t alpha[3] =  {0.947543636291,  0.960433870103,  0.948059448969};
      float32_t beta[3]  =  {0.3924854250920, 0.3978247347593, 0.3926990816987};
      // magnitude ~= alpha * max(|I|, |Q|) + beta * min(|I|, |Q|)
      float32_t abs_inphase    = fabs(inphase);
      float32_t abs_quadrature = fabs(quadrature);
      if (abs_inphase > abs_quadrature) return alpha[crit] * abs_inphase    + beta[crit] * abs_quadrature;
      else                              return alpha[crit] * abs_quadrature + beta[crit] * abs_inphase;
    }

    // ---
    /*--------------------------------------------------*
                 --- LOG2_APPROXIMATE ---
      --------------------------------------------------*/
    float32_t log2_approx_f32(float input) {
      float32_t mantissa;
      int   exponent;
      // Calculate mantissa and exponent
      mantissa = frexpf(fabsf(input), &exponent);
      // Expand using Horner's method
      return (((1.23149591368684f * mantissa  - 4.11852516267426f) * mantissa +
               6.02197014179219f) * mantissa - 3.13396450166353f) + exponent;
    }

    // ---
    /*---------------------------------------------------------------*
                        --- FREQUENCY SHIFTER ---
        Performs a complex frequency shift by complex multiplication
        of a time domain signal by a complex exonential.
        From Fourier theory a spectral shift:
           F(j(w + w0)) = Fourier[f(t) * e^(j(w_0*t))]
                        = Fourier[f(t) * cos(w_0]*t) * jsin(w_0*t)]
           where f(t) is complex, and j is the sqrt(-1),
                                                                    
           Note:  In the discrete-time case this results in a
           spectral ROTATION, ie spectral components at either end
           of the complex spectrum (near the Nyquist frequency)
           will appear at the other end!
    ---------------------------------------------------------------*/
    float32_t freq_shifter(float * _Idata, float * _Qdata, float freq_shift,  float initial_phase) {
      const float32_t twoPI = 2.0 * PI;
      float32_t phase_inc   = freq_shift * (twoPI / AUDIO_SAMPLE_RATE_EXACT);
      float32_t phase       = initial_phase;
      // Frequency shifting is a complex multiplication in the time domain:
      for (int i = 0; i < n_block; i++) {
        float32_t cosine = cos_f32(phase);
        float32_t sine   = sin_f32(phase);
        float32_t tempI  = _Idata[i];
        float32_t tempQ  = _Qdata[i];
        _Idata[i]     = tempI * cosine - tempQ * sine;
        _Qdata[i]     = tempQ * cosine + tempI * sine;
        phase       += phase_inc;
        if (phase > twoPI)   phase -= twoPI;
        else if (phase < 0.0) phase += twoPI;
      }
      // Return the updated phase for the next iteration
      return phase;
    }

    // ---
    /*---------------------------------------------------*
               --- QUADRARURE FREQUENCY MIXER ---
        Computes the products
          f_I_out(t) = f_I_in(t) * cos(w_0*t)
          f_Q_out(t) = f_Q_in(t) * sin(w_0*t)
        which produces sum and difference frequencies
        for sinusoidal inputs, ie this an unbalanced
        modulator,
      ---------------------------------------------------*/
    float32_t freq_mixer(float * _Idata, float * _Qdata, float freq_shift, float initial_phase) {
      const float32_t twoPI = 2.0 * PI;
      float32_t phase_inc   = freq_shift * (twoPI / AUDIO_SAMPLE_RATE_EXACT);
      float32_t phase       = initial_phase;
      for (int i = 0; i < n_block; i++) {
        float32_t cosine  = cos_f32(phase);
        float32_t sine    = sin_f32(phase);
        float32_t tempI   = _Idata[i];
        float32_t tempQ   = _Qdata[i];
        _Idata[i]         = tempI * cosine;
        _Qdata[i]         = tempQ * sine;
        phase            += phase_inc;
        if (phase > twoPI)   phase -= twoPI;
        else if (phase < 0.0) phase += twoPI;
      }
      return phase;
    }
    // -------------------------------------------------------------------------------------------------------

    /*---------------------------------------------------*
             --- AudioSDR FILTER COEFFICIENTS ---
      ---------------------------------------------------*/
  private:
    // ---
    // --- Filter coefficients for Audio IIR Filters ---
    //
    // Notes on IOWA Hills IIR band-pass Designs:
    //   1)  The center frequency Fc is the geometric mean of the lower and upper
    //       cut-off frequencies, ie Fc = sqrt(F_lower * F_upper)
    //   2)  Iowa Hills use the 6dB bandwidth, whereas I use the 3dB (half-power)
    //       bandwidth, therefore their design bandwidths are larger than mine.
    //   3)  Iowa Hills (wrongly) define the filter order as the number of bi-quad
    //       (SOS) sections instead of the number of poles in the filter design.
    //       Therefore the filters designed here are actually 8th order Elliptic
    //       filters, not 4th order as indicated in the design GUI.
    //
    // ---
    //   Also note that the arm_math biquad IIR filters require the Iowa Hills coefficients a1 and a2 have their
    //   signs reversed
    // ---
    // Audio post-demod 2.1kHz BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 2100Hz, therefore F_center = sqrt(150*2100) = 561.2.4Hz (0.0255)
    // BW = 0.10715
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw2100_coefs[20] = {
      0.281254361701984257, -0.562493993644119561, 0.281254361701984257, 1.952997193705192870, -0.953922052090953376,
      0.214243744065813785, -0.287431528423247040, 0.214243744065813813, 1.689417435117400500, -0.726644133624002264,
      0.189970210897357356, -0.379938650995254257, 0.189970210897357356, 1.989604869713017310, -0.990091299861694751,
      0.168761125859136579,  0.017139310878815404, 0.168761125859136579, 1.801854712954994660, -0.879553281952586086
    };
    // ---
    // Audio post-demod 2.3kHz BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 2300Hz, therefore F_center = sqrt(150*2300) = 587.4.4Hz (0.0266)
    // BW = 0.1288 (2840 Hz)
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw2300_coefs[20] = {
      0.289406501341733458, -0.578798339911709481, 0.289406501341733458, 1.953232475600815920, -0.954124045571293711,
      0.211875468977264531, -0.259061566136226606, 0.211875468977264531, 1.653635753875417390, -0.698517409528392030,
      0.201256837579942033, -0.402511863005069415, 0.201256837579942061, 1.989571769409478110, -0.990048461507118227,
      0.176269945845154902,  0.053417595207295554, 0.176269945845154902, 1.774257073808950480, -0.867129737267279155
    };
    // ---
    // Audio post-demod 2.5 kHz BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 2500Hz, therefore F_center = sqrt(150*2500) = 612.4Hz (0.0278)
    // BW = 0.1288 (2840 Hz)
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw2500_coefs[20] = {
      0.296978038380352749, -0.593940863296862798, 0.296978038380352749, 1.952683062902163200, -0.953581600345526748,
      0.210146283787265964, -0.234036554647055833, 0.210146283787265992, 1.622512137998870060, -0.674769187288778660,
      0.211726357044414920, -0.423450788213088014, 0.211726357044414920, 1.989381137419197420, -0.989866850535885967,
      0.183177808559878280,  0.084882582012851746, 0.183177808559878308, 1.748723792073403340, -0.856396166157033512
    };
    // ---
    // Audio post-demod 2.7 kHz BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 2700Hz, therefore F_center = sqrt(150*2700) = 636.4Hz (0.289)
    // BW = 0.1288 (2840 Hz)
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw2700_coefs[20] = {
      0.305591661255011304, -0.611167754953983788, 0.305591661255011304, 1.952546934210232840, -0.953437233552230312,
      0.208432872466726010, -0.207942395873919245, 0.208432872466726010, 1.589768552841398510, -0.650304594339650710,
      0.223126293055299657, -0.446250569629344340, 0.223126293055299657, 1.989279256333149610, -0.989766034841996434,
      0.190552107607214360,  0.116894861471134343, 0.190552107607214360, 1.721121730875193380, -0.845270189338173883
    };
    // ---
    // Audio post-demod 2.9 kHz BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 2900Hz, therefore F_center = sqrt(150*2900) = 659.5Hz (0.0299)
    // BW = 0.15035
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw2900_coefs[20] = {
      0.315412512142470847, -0.630809297604115748,  0.315412512142470847, 1.952811291243868870, -0.953679254879347016,
      0.206728256955276607, -0.180800996268307895,  0.206728256955276607, 1.555208191810562020, -0.625062235852461234,
      0.235560068080949259, -0.471118054335721126,  0.235560068080949286, 1.989265491121462890, -0.989745656232921767,
      0.198422211506727897,  0.149497687517384709,  0.198422211506727897, 1.691154207879723930, -0.833704640768851313
    };
    // ---
    // Audio post-demod 3.1 kHz BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 3100Hz, therefore F_center = sqrt(150*3100) = 682.0Hz (0.0309)
    // BW = 0.16115
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw3100_coefs[20] = {
      0.325343126545082517, -0.650670182564146327, 0.325343126545082517, 1.952860963619506670, -0.953717821081684458,
      0.205294213585258095, -0.155403592463629892, 0.205294213585258095, 1.522495491989495960, -0.601803862095975362,
      0.247786247692179523, -0.495570327397973631, 0.247786247692179579, 1.989213168681239540, -0.989691741581839590,
      0.206027242248543296,  0.179610697868997388, 0.206027242248543296, 1.661600598539905830, -0.822900633482976818
    };
    // ---
    // Audio post-demod 3.3 kHz) BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 3300Hz, therefore F_center = sqrt(150*3300) = 703.56 Hz (0.0319)
    // BW = 0.17325
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw3300_coefs[20] = {
      0.337168066644705333, -0.674319801927098617, 0.337168066644705333, 1.953159366591916020, -0.953996407999300966,
      0.203797815247538400, -0.127880526851349763, 0.203797815247538400, 1.486198965250347780, -0.576633444676513784,
      0.261745716252479721, -0.523489188386493720, 0.261745716252479721, 1.989214444617858120, -0.989686676984118718,
      0.214516293297874777,  0.211863602270518092, 0.214516293297874777, 1.627822179373723980, -0.811107515005655699
    };
    // ---
    // Audio post-demod AM (3.9 kHz) BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_cl = 150Hz, f_cu = 3900Hz, therefore F_center = sqrt(150*3900) = 764.8 Hz ( 0.0347)
    // BW = 0.2071
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw3900_coefs[20] = {
      0.373595243140580457, -0.747172867264591645, 0.373595243140580513, 1.953498342031101180, -0.954306129294667182,
      0.200355752945732990, -0.056823132076542736, 0.200355752945732990, 1.387493286008134600, -0.511785753662859055,
      0.301862290603145733, -0.603722064882966314, 0.301862290603145733, 1.989138866602465190, -0.989604616247514612,
      0.237955827945548121,  0.294690213886068364, 0.237955827945548121, 1.529548307358015610, -0.780098055068093932
    };
    // ---
    // Audio post-demod CW (400 Hz) BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 0.0351 (774 Hz), BW = 0.02160 (476 Hz)
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t bw470_coefs[20] = {
      0.325442765141308932,  -0.649673806026101697,   0.325442765141308932,  1.956983741609901410, -0.966519662697379989,
      0.322666195558872548,  -0.632605985353747791,   0.322666195558872548,  1.943379016382100530, -0.958273637946739143,
      0.171725132774581513,  -0.343255124580149118,   0.171725132774581513,  1.982041643396188710, -0.989487595794544705,
      0.170604083645742644,  -0.319631344958988373,  -1.963497179540541810,  0.983028062531475788, -0.170604083645742671
    };
   // ---
    // Audio post-demod WSPR (400 Hz) BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 0.067 1448 Hz), BW = 0.0200 (441 Hz)
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t wspr_coefs[20] = {
      0.242638324423997687,  -0.480428333378961681,  0.242638324423997687,  1.926844057734943270,  -0.966186506661544597,
      0.241709618592532510,  -0.459606722362008779,  0.241709618592532510, -1.913861406136361690,  -0.962488397365951376,
      0.130924094058145229,  -0.260824216410256715,  0.130924094058145229,  1.953127143628212940,  -0.987823169143460245,
      0.130488762801277719,  -0.229254725978934398,  0.130488762801277719,  1.928196919157067590,  -0.984538591886083281
    };
    // ---
    //                           ----------------------------
    // CW 1kHz IF IIR BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 6.89kHz,
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t CWpre_coefs[20] = {
      0.241142396487104477,  -0.352462660588158228,  0.241142396487104477,  1.112420382052906160,  -0.902001144700291135,
      0.240017816062385786,  -0.152853073056203187,  0.240017816062385786,  1.006369266833312400,  -0.897794614263582758,
      0.147481431883840497,  -0.254567821325217380,  0.147481431883840497,  1.211465849939156620,  -0.962515990875300154,
      0.146884997799233058,   0.013154639293845102,  0.146884997799233058,  0.965185183191514917,  -0.958623451071442756
    };
    // ---
    //                           ----------------------------
    // WSPR 450Hz IF IIR BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 6.89kHz,
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t WSPRpre_coefs[20] = {
      0.246084591699641864,  -0.317335335403629515,  0.246084591699641891,  1.107861958121652220,  -0.951304250547790442,
      0.245839229249974839,  -0.221965241940777630,  0.245839229249974839,  1.059321085368752910,  -0.950355737925846378,
      0.129614440769847961,  -0.192468008344885394,  0.129614440769847961,  1.155151819152469010,  -0.981250483665057627,
      0.129501160070902038,  -0.074705318268108167,  0.129501160070902038,  1.043329796116461820,  -0.980392888323285749
    };
    // ---
    //                           ----------------------------
    // SSB 3kHz IF IIR BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 6.89kHz,
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t SSBpre_coefs[20] = {
      0.431339311114742496, -0.713470542898471116,  0.431339311114742552, 1.101645528027215050, -0.713152957996962655,
      0.405431891178868020,  0.036265531287101561,  0.405431891178868076, 0.718505860710959787, -0.670319038886762364,
      0.321020863623668884, -0.604037184977245345,  0.321020863623668884, 1.377588402853656020, -0.909128745501107338,
      0.309679288357254057,  0.296618587153241187,  0.309679288357254057, 0.568224414953491297, -0.877009487028082901
    };
    // ---
    //                           ----------------------------
    // AM Image rejection IIR LPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 6.89kHz,
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t AMimage_coefs[20] = {
      0.209325293095768195, -0.303897781729920602, 0.209325293095768195, 1.384519679637237430, -0.499272484098853164,
      0.332033631248614625, -0.446312550268681052, 0.332033631248614625, 1.442704167088042720, -0.660458879316590974,
      0.301307464075802278, -0.280053123017876382, 0.301307464075802278, 1.509178471911276190, -0.831740277045004417,
      0.132730945079106794,  0.116733341653715780, 0.132730945079106794, 1.568977735857043590, -0.951172967668973080
    };
    // ---
    //                           ----------------------------
    // AM 8kHz IIR IF BPF filter coefficients  (Iowa Hills, 8th-order (4 SOS sections), Elliptic,
    // f_c = 6.89kHz,
    // (each row is one filter section)  b0, b1, b2, -a1, -a2
    float32_t AMpre_coefs[20] = {
      0.5800818617383324271, -1.097910701730202550,  0.580081861738332427,  1.178717262640705870, -0.521879755590131889,
      0.385739840901379194,   0.381919343603990658,  0.385739840901379194,  0.293317925971320503, -0.347036904908080812,
      0.552110184864637921,  -1.091109512736456510,  0.552110184864637921,  1.617498852028246280, -0.868774552360621288,
      0.479480610202897395,   0.828203078425222006,  0.479480610202897395, -0.109012279931153116, -0.754488078492427716
    };
    // ---
    //                         ------------------------------------
    // Hilbert transform FIR length 257 filter
    // Note: This set of coefficients (65 in total) recognizes tha Hilbert coeffs are odd-symmetric about the mid-point of
    // the impulse response and that the odd coefficients are all zero.
    int16_t   hilbertFilterLength = 257;
    int16_t   hilbertDelay = (hilbertFilterLength - 1) / 2;
    float32_t hilbert_coeffs[64] = {
      -0.003780058,  -0.0007677196, -0.0008456816, -0.0009285885,
      -0.001016253,  -0.00110868,   -0.001206127,  -0.001309284,
      -0.001418976,  -0.001535649,  -0.001658795,  -0.001787153,
      -0.001920324,  -0.002062003,  -0.002218711,  -0.002372268,
      -0.00254078,   -0.002716246,  -0.002901201,  -0.003095788,
      -0.003300195,  -0.003514823,  -0.003740456,  -0.00397813,
      -0.004228542,  -0.004491779,  -0.004768153,  -0.005059852,
      -0.00536908,   -0.005692769,  -0.006036516,  -0.00639912,
      -0.006783208,  -0.00719045,   -0.007622755,  -0.008082277,
      -0.008571692,  -0.009094321,  -0.009653805,  -0.01025394,
      -0.01089927,   -0.01159615,   -0.01235175,   -0.01317265,
      -0.01407044,   -0.01505581,   -0.01614401,   -0.01735329,
      -0.01870661,   -0.02023316,   -0.021971,     -0.02397055,
      -0.02629965,   -0.02905173,   -0.03235971,   -0.03641963,
      -0.04153193,   -0.04818082,   -0.05720523,   -0.07018713,
      -0.09051862,   -0.1270187,    -0.2120234,    -0.6365587
    };
    //  --------------------END OF FILTER COEFFICIENTS ----------
    //
    /*------------------------------------*
               --- SINE TABLE ---
      ------------------------------------*/
    float32_t sine_table_f32[257] = {
      0.00000000, 0.02454123, 0.04906767, 0.07356456, 0.09801714, 0.12241068, 0.14673047, 0.17096189,
      0.19509032, 0.21910124, 0.24298018, 0.26671276, 0.29028468, 0.31368174, 0.33688985, 0.35989504,
      0.38268343, 0.40524131, 0.42755509, 0.44961133, 0.47139674, 0.49289819, 0.51410274, 0.53499762,
      0.55557023, 0.57580819, 0.59569930, 0.61523159, 0.63439328, 0.65317284, 0.67155895, 0.68954054,
      0.70710678, 0.72424708, 0.74095113, 0.75720885, 0.77301045, 0.78834643, 0.80320753, 0.81758481,
      0.83146961, 0.84485357, 0.85772861, 0.87008699, 0.88192126, 0.89322430, 0.90398929, 0.91420976,
      0.92387953, 0.93299280, 0.94154407, 0.94952818, 0.95694034, 0.96377607, 0.97003125, 0.97570213,
      0.98078528, 0.98527764, 0.98917651, 0.99247953, 0.99518473, 0.99729046, 0.99879546, 0.99969882,
      1.00000000, 0.99969882, 0.99879546, 0.99729046, 0.99518473, 0.99247953, 0.98917651, 0.98527764,
      0.98078528, 0.97570213, 0.97003125, 0.96377607, 0.95694034, 0.94952818, 0.94154407, 0.93299280,
      0.92387953, 0.91420976, 0.90398929, 0.89322430, 0.88192126, 0.87008699, 0.85772861, 0.84485357,
      0.83146961, 0.81758481, 0.80320753, 0.78834643, 0.77301045, 0.75720885, 0.74095113, 0.72424708,
      0.70710678, 0.68954054, 0.67155895, 0.65317284, 0.63439328, 0.61523159, 0.59569930, 0.57580819,
      0.55557023, 0.53499762, 0.51410274, 0.49289819, 0.47139674, 0.44961133, 0.42755509, 0.40524131,
      0.38268343, 0.35989504, 0.33688985, 0.31368174, 0.29028468, 0.26671276, 0.24298018, 0.21910124,
      0.19509032, 0.17096189, 0.14673047, 0.12241068, 0.09801714, 0.07356456, 0.04906767, 0.02454123,
      0.00000000, -0.02454123, -0.04906767, -0.07356456, -0.09801714, -0.12241068, -0.14673047, -0.17096189,
      -0.19509032, -0.21910124, -0.24298018, -0.26671276, -0.29028468, -0.31368174, -0.33688985, -0.35989504,
      -0.38268343, -0.40524131, -0.42755509, -0.44961133, -0.47139674, -0.49289819, -0.51410274, -0.53499762,
      -0.55557023, -0.57580819, -0.59569930, -0.61523159, -0.63439328, -0.65317284, -0.67155895, -0.68954054,
      -0.70710678, -0.72424708, -0.74095113, -0.75720885, -0.77301045, -0.78834643, -0.80320753, -0.81758481,
      -0.83146961, -0.84485357, -0.85772861, -0.87008699, -0.88192126, -0.89322430, -0.90398929, -0.91420976,
      -0.92387953, -0.93299280, -0.94154407, -0.94952818, -0.95694034, -0.96377607, -0.97003125, -0.97570213,
      -0.98078528, -0.98527764, -0.98917651, -0.99247953, -0.99518473, -0.99729046, -0.99879546, -0.99969882,
      -1.00000000, -0.99969882, -0.99879546, -0.99729046, -0.99518473, -0.99247953, -0.98917651, -0.98527764,
      -0.98078528, -0.97570213, -0.97003125, -0.96377607, -0.95694034, -0.94952818, -0.94154407, -0.93299280,
      -0.92387953, -0.91420976, -0.90398929, -0.89322430, -0.88192126, -0.87008699, -0.85772861, -0.84485357,
      -0.83146961, -0.81758481, -0.80320753, -0.78834643, -0.77301045, -0.75720885, -0.74095113, -0.72424708,
      -0.70710678, -0.68954054, -0.67155895, -0.65317284, -0.63439328, -0.61523159, -0.59569930, -0.57580819,
      -0.55557023, -0.53499762, -0.51410274, -0.49289819, -0.47139674, -0.44961133, -0.42755509, -0.40524131,
      -0.38268343, -0.35989504, -0.33688985, -0.31368174, -0.29028468, -0.26671276, -0.24298018, -0.21910124,
      -0.19509032, -0.17096189, -0.14673047, -0.12241068, -0.09801714, -0.07356456, -0.04906767, -0.02454123,
      -0.00000000
    };
};
#endif
