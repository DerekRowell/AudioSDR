#ifndef SI5351A_H
  #define SI5351A_H

  #include <inttypes.h>
  #define SI_I2C_ADDR     0x60
  #define SI_CLK0_CONTROL	16			        // Register definitions
  #define SI_CLK1_CONTROL	17
  #define SI_CLK2_CONTROL	18
  #define SI_SYNTH_PLL_A	26
  #define SI_SYNTH_PLL_B	34
  #define SI_SYNTH_MS_0		42
  #define SI_SYNTH_MS_1		50
  #define SI_SYNTH_MS_2		58
  #define SI_PLL_RESET		177
  #define SI_CLK0_PHASE	  165
  #define SI_CLK1_PHASE	  166
   
  #define SI_R_DIV_1		0b00000000			// R-division ratio definitions
  #define SI_R_DIV_2		0b00010000
  #define SI_R_DIV_4		0b00100000
  #define SI_R_DIV_8		0b00110000
  #define SI_R_DIV_16		0b01000000
  #define SI_R_DIV_32		0b01010000
  #define SI_R_DIV_64		0b01100000
  #define SI_R_DIV_128	0b01110000

  #define SI_CLK_SRC_PLL_A	0b00000000
  #define SI_CLK_SRC_PLL_B	0b00100000
  #define SI_CLK_STRENGTH   0b00000011       // 0x03 = 8mA, ...  0x0 = 2mA
  #define SI_CLK_MSINT      0b00000000       //

  #define XTAL_FREQ	25000000			          // Crystal frequency
  //
  class SI5351quad {
    public:
      void SetFrequency(uint32_t);
      void reset(void);
      void swapIQ(bool);
      void Correction(int32_t);
    private:
      void setupPLL(uint8_t, uint8_t, uint32_t, uint32_t);
      void setupMultisynth(uint8_t, uint32_t, uint8_t);
      void OutputOff(uint8_t);
      void writeRegister(uint16_t, uint16_t);
      //
      uint8_t   n_div;
      uint32_t  freq_min = 0;
      uint32_t  freq_max = 0;
      uint32_t  PLL_mid_freq = 750000000;
      uint32_t  PLL_max_freq = 900000000;
      uint32_t  PLL_min_freq = 600000000;
      uint32_t  n_prev = 0;
      bool      IQswap = false;
      bool      IQswap_has_changed = false;
      int32_t   xtal_correction = 0;
  };
#endif //SI5351A_H
