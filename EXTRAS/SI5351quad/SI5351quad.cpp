#include <Wire.h>
#include "SI5351quad.h"
#include "Streaming.h"
// ---
void SI5351quad::SetFrequency(uint32_t frequency) {
  uint32_t pllFreq;
  uint32_t xtalFreq = XTAL_FREQ - xtal_correction;
  uint32_t l;
  float    f;
  uint8_t  mult;
  uint32_t num;
  uint32_t denom;
  //
  if (frequency < 3500000) frequency = 3500000; // Set lower limit
  // Look for a change in the multisynth divider 
  if ((frequency>freq_max) || (frequency < freq_min) ){
    n_div = PLL_mid_freq/frequency;
    if (n_div % 2) n_div--;        // Ensure an even integer division ratio
    if (n_div > 126) {             // Special case for low frequency limit
      n_div = 126;                 // where the PLL allosed to run below the
      freq_min = 3500000;          // specified lower limit of 600 MHz.
      freq_max = 6950000;
     } else {                      // New operating frequency limits
       freq_min = PLL_min_freq/n_div;
       freq_max = PLL_max_freq/n_div;
     }
  }
  //
  pllFreq = n_div*frequency;       // Calculate the pllFrequency: the divider * desired output frequency
  mult    = pllFreq/xtalFreq;      // Determine the multiplier to get to the required pllFrequency
  l       = pllFreq%xtalFreq;      // It has three parts:
  f       = l;                     // mult is an integer that must be in the range 15..90
  f      *= 1048575;               // num and denom are the fractional parts, the numerator and denominator
  f      /= xtalFreq;              // each is 20 bits (range 0..1048575)
  num     = f;                     // the actual multiplier is  mult + num / denom
  denom   = 1048575;               // For simplicity we set the denominator to the maximum 1048575
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);
  //
  // Determine if n_div has changed and if so configure the multisynth divider
  if((n_div != n_prev) || (n_prev == 0) ){
    setupMultisynth(SI_SYNTH_MS_0, n_div, SI_R_DIV_1);
    setupMultisynth(SI_SYNTH_MS_1, n_div, SI_R_DIV_1);
    // Reset the PLL. This causes a glitch in the output. When operating within the
    // constraints set by n_div you don't need to reset the PLL, and there is no glitch
    writeRegister(SI_PLL_RESET, 0x20); 
    // Finally switch on the outputs (0x4F)
    // and set the MultiSynth0 input to be PLL A
    writeRegister(SI_CLK0_CONTROL, 0x0C | SI_CLK_STRENGTH | SI_CLK_SRC_PLL_A);
    writeRegister(SI_CLK1_CONTROL, 0x0C | SI_CLK_STRENGTH | SI_CLK_SRC_PLL_A);
    // Set the phase difference if thye divider has changed
    if (n_div != n_prev){
      writeRegister(SI_CLK0_PHASE, 0);
      delay(5);                              // Delays found to be essential for reliable operation
      writeRegister(SI_CLK1_PHASE, n_div);
      delay(5);
      writeRegister(SI_PLL_RESET, 0x20);
      delay(5);
    } 
    n_prev = n_div;
  }
}

// ---
void SI5351quad::setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom) {
  // Set up specified PLL with mult, num and denom
  // mult is 15..90
  // num is 0..1,048,575 (0xFFFFF)
  // denom is 0..1,048,575 (0xFFFFF)
	uint32_t P1;					// PLL config register P1
	uint32_t P2;					// PLL config register P2
	uint32_t P3;					// PLL config register P3
  //
	P1 = (uint32_t)(128 * ((float)num / (float)denom));
	P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
	P2 = (uint32_t)(128 * ((float)num / (float)denom));
	P2 = (uint32_t)(128 * num - denom * P2);
	P3 = denom;
  //
	writeRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
	writeRegister(pll + 1, (P3 & 0x000000FF));
	writeRegister(pll + 2, (P1 & 0x00030000) >> 16);
	writeRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
	writeRegister(pll + 4, (P1 & 0x000000FF));
	writeRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
	writeRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
	writeRegister(pll + 7, (P2 & 0x000000FF));
}

// ---
void SI5351quad::setupMultisynth(uint8_t synth, uint32_t divider, uint8_t rDiv) {
  // Set up MultiSynth with integer divider and R divider
  // R divider is the bit value which is OR'ed onto the appropriate register, it is a #define in si5351a.h
	uint32_t P1;					// Synth config register P1
	uint32_t P2;					// Synth config register P2
	uint32_t P3;					// Synth config register P3
  //
	P1 = 128 * divider - 512;
	P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
	P3 = 1;
  //
	writeRegister(synth + 0,   (P3  & 0x0000FF00) >> 8);
	writeRegister(synth + 1,   (P3  & 0x000000FF));
	writeRegister(synth + 2,   ((P1 & 0x00030000) >> 16) | rDiv);
	writeRegister(synth + 3,   (P1  & 0x0000FF00) >> 8);
	writeRegister(synth + 4,   (P1  & 0x000000FF));
	writeRegister(synth + 5,   ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
	writeRegister(synth + 6,   (P2  & 0x0000FF00) >> 8);
	writeRegister(synth + 7,   (P2  & 0x000000FF));
}

// ---
void SI5351quad::swapIQ(bool swap){
  // Define which clock (0 or 1) leads the phase in the quadrature
  // IQ output.  Assume the I channel is the leading phase,
  IQswap = swap;
  IQswap_has_changed = true;
  /*if (IQswap) {
    writeRegister(SI_CLK0_PHASE, (uint8_t)n_div);
    writeRegister(SI_CLK1_PHASE, (uint8_t)0);
} else {
    writeRegister(SI_CLK0_PHASE, (uint8_t)0);
    writeRegister(SI_CLK1_PHASE, (uint8_t)n_div);
  }
  writeRegister(SI_PLL_RESET, 0x20);
  */
}

// ---
void SI5351quad::reset(void) {
  // Reset PLL A & and enable its output
  // This soft-resets PLL A & and enables its output
  // This soft-resets PLL B & and enables its output
  writeRegister(SI_PLL_RESET, 0x20);
  writeRegister(SI_CLK0_CONTROL, 0x0C | SI_CLK_STRENGTH | SI_CLK_SRC_PLL_A);
  writeRegister(SI_PLL_RESET, 0x20);
  writeRegister(SI_CLK1_CONTROL, 0x0C | SI_CLK_STRENGTH | SI_CLK_SRC_PLL_A);
  freq_max = 0;
  SetFrequency(30000000);
  delay(50);
}

// ---
void SI5351quad::OutputOff(uint8_t clk) {
  // Switch off Si5351a output
  // Example: si5351aOutputOff(SI_CLK0_CONTROL);
  // will switch off output CLK0
  //
	writeRegister(clk, 0x80);		// Refer to SiLabs AN619 to see bit values - 0x80 turns off the output stage
}

// --
void SI5351quad::Correction(int32_t measuredErrorAt25MHz){
  // update the correction to be appled to the xtal frequency
  // Note: the correction must be done at the xtal frequency, ie 25000000 Hz for Adafruit
  xtal_correction = - measuredErrorAt25MHz;
}

// ---
void SI5351quad::writeRegister(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(SI_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
//    if (uint8_t status = Wire.endTransmission() != 0){
//    Serial.print("SI5351quad: ");
//     Serial.print(status); Serial.print("\t");
 //    Serial.print(reg); Serial.print("\t");
//     Serial.println(value);
//     }
}
