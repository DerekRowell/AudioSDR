# AudioSDR
Version 1.10, uploaded March 29, 2021

 A single Teensy Audio Library class object for a complete SDR (software-defined-radio)
 demodulator/processor.
   - Creates a dual-conversion receiver with an IF at approx. 7kHz
   - Contains demodulators for CW/SSB/AM/SAM and a WSPR mode
   - Contains an impulse noise blanker, AGC, ALS (adaptive-least-squares)
     automatic notch/peak filters
   - Contains demod mode dependent IF and audio band-pass filters
   - Uses float32 processing throughout.
   - Uses the I2S input and output on the Teensy Audio Board.
   
  See the detailed description in the supplied pdf file

  Changes from Version 1.01:
  1) Fixed bug in AM & SAM demodulators that was causing severe audio distortion.
  2) Modified PLL in SAM demodulator to improve carrier lock.
  
