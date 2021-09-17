#include <stdint.h>
#include <stm32f1xx_hal_conf.h>
#include <stm32f1xx_it.h>
#include <string.h>
#include "math.h"

I2C_HandleTypeDef hi2c1;
uint8_t SensorAddressWrite = 0x76 << 1 | 0x00;
uint8_t SensorAddressRead = 0x76 << 1 | 0x01;
float PAR_T1, PAR_T2, PAR_T3, PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6, PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11; //Float vrednosti koriscene za trimovanje P i T

void initialisation(char operationalMode, uint8_t PEnable, uint8_t TEnable, uint8_t T_oversampling, uint8_t P_oversampling, uint8_t OdrRate, uint8_t IIRfilter)
{
    /*      Initialisation funkcija služi podešavanju parametra senzora:
     *      - T oversampling (vrednosti moraju biti 2^n, gde n može biti 0 - 5)
     *      - P oversampling (vrednosti moraju biti 2^n, gde n može biti 0 - 5)
     *      - Temperature enable (omogućiti merenje temperature)
     *      - Pressure enable (omogućiti merenje pritiska)
     *      - Operational mode ('n'- normal mode (ciklično merenje), 'f' - forced mode (jedno merenje potom sleep), 's' - sleep mode (nema merenja))
     *      - Output data rate (ODR)
     *      - Koeficijent IIRfilter-a (vrednosti mogu biti 2^n - 1, gde n može biti 0-7)
     */

        uint8_t dataBuffer[5];
        uint8_t pwr_Ctrl5_4, pwr_Ctrl1, pwr_Ctrl0, pwr_Ctrl;

        switch(operationalMode)
        {
            case 'n': pwr_Ctrl5_4 = 48; break;       // Normal mode
            case 'f': pwr_Ctrl5_4 = 32; break;       // Forced mode
            case 's': pwr_Ctrl5_4 = 0; break;        // Sleep mode
            default: pwr_Ctrl5_4 = 48;               //Normal mode
        }

        switch(PEnable)
        {
            case 0: pwr_Ctrl1 = 0; break;         // Merenje pritiska onemogućeno
                default: pwr_Ctrl1 = 2;           // Merenje pritiska omogućeno
        }

        switch(TEnable)
        {
            case 0: pwr_Ctrl0 = 0; break;         // Merenje temperature onemogućeno
                default: pwr_Ctrl0 = 1;           // Merenje temperature omogućeno
        }

        pwr_Ctrl = pwr_Ctrl5_4 | pwr_Ctrl1 | pwr_Ctrl0;      // Kombinovanje pređašnjih vrednosti u vrednost koju je potrebno zapisati u registru

        uint8_t pwr_Ctrl_Set[2] = {0x1B, pwr_Ctrl};              //  adresa registra, pwrl_Ctrl - vrednost koju želimo upisati
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, pwr_Ctrl_Set, 2, HAL_MAX_DELAY);     // Upis u registar
        
        uint8_t osr5_3, osr2_0, osr;

        switch(T_oversampling)
        {
            case 0: osr5_3 = 0; break;        // x1 no oversampling
            case 1: osr5_3 = 0; break;        // x1 no oversampling
            case 2: osr5_3 = 8; break;        // x2 oversampling
            case 4: osr5_3 = 16; break;       // x4 oversampling
            case 8: osr5_3 = 24; break;       // x8 oversampling
            case 16: osr5_3 = 32; break;      // x16 oversampling
            case 32: osr5_3 = 40; break;      // x32 oversampling
                default: osr5_3 = 40;         // x32 oversampling
        }

        switch(P_oversampling)
        {
            case 0: osr2_0 = 0; break;        // x1 no oversampling
            case 1: osr2_0 = 0; break;        // x1 no oversampling
            case 2: osr2_0 = 1; break;        // x2 oversampling
            case 4: osr2_0 = 2; break;        // x4 oversampling
            case 8: osr2_0 = 3; break;        // x8 oversampling
            case 16: osr2_0 = 4; break;       // x16 oversampling
            case 32: osr2_0 = 5; break;       // x32 oversampling
                default: osr2_0 = 5;          // x32 oversampling
        }

        
        osr = osr5_3 | osr2_0;

        uint8_t osr_Set[2] = {0x1C, osr};
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, osr_Set, 2, HAL_MAX_DELAY);

        uint8_t odr;

        if(OdrRate<=5)  //5ms output data rate
          odr = 0;
          else if(OdrRate<=10)  //10ms output data rate
              odr = 1;
              else if(OdrRate<=20)  //20ms output data rate
                  odr = 2;
                  else if(OdrRate<=40)  //40ms output data rate
                      odr = 3;
                      else if(OdrRate<=80)  //80ms output data rate
                        odr = 4;
                        else if(OdrRate<=160)   //160ms output data rate
                            odr = 5;
                            else if(OdrRate<=320)   //320ms output data rate
                              odr = 6;
                              else if(OdrRate<=640) //640ms output data rate
                                odr = 7;
                                else if(OdrRate<=1280)  //1280ms output data rate
                                    odr = 8;
                                    else
                                      odr = 9;      //2560ms output data rate

        uint8_t odr_Set[2] = {0x1D, odr}; 
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, odr_Set, 2, HAL_MAX_DELAY);

        uint8_t config;
        switch(IIRfilter)
        {
            case 0: config = 0; break;        // bypass-mode
            case 1: config = 2; break;        // Koef = 1
            case 3: config = 4; break;        // Koef = 3
            case 7: config = 6; break;        // Koef = 7
            case 15: config = 8; break;       // Koef = 15
            case 31: config = 10; break;      // Koef = 31
            case 63: config = 12; break;      // Koef = 63
            case 127: config = 14; break;     // Koef = 127
                default: config = 14;         // Koef = 127
        }

        uint8_t config_Set[2] = {0x1F, config};
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, config_Set, 2, HAL_MAX_DELAY);
}

void write1(int8_t *NVM_Par, uint8_t NVM_par_Address)      // Funkcija za čitanje unsigned short vrednosti za trimovanje
{
    uint8_t dataBuffer[5];
    HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &NVM_par_Address, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, SensorAddressWrite  , dataBuffer, 1, HAL_MAX_DELAY);
    *NVM_Par = dataBuffer[0]; 

}

void write2(uint16_t *NVM_Par, uint8_t NVM_par_Address[])      // Funkcija za čitanje signed short vrednosti za trimovanje
{
    uint8_t niz[5], dataBuffer[5];
    HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &NVM_par_Address[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead , dataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = dataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &NVM_par_Address[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead  , dataBuffer, 1, HAL_MAX_DELAY);  
    niz[1] = dataBuffer[0];
    memcpy(NVM_Par, &niz, sizeof(*NVM_Par));
}

void write3(int16_t *NVM_Par, uint8_t NVM_par_Address[])      // Funkcija za čitanje signed short vrednosti za trimovanje
{
    uint8_t niz[5], dataBuffer[5];
    HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &NVM_par_Address[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead  , dataBuffer, 1, HAL_MAX_DELAY);
    niz[0] = dataBuffer[0];
    HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &NVM_par_Address[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead  , dataBuffer, 1, HAL_MAX_DELAY);  
    niz[1] = dataBuffer[0];
    memcpy(NVM_Par, &niz, sizeof(*NVM_Par));
}

void trimming(void)
{

      //Adrese registra vrednosti za trimovanje T
      uint8_t NVM_par_T3_Address = 0x35;
      int8_t NVM_par_T3;
      uint8_t NVM_par_T2_Address[2] = {0x34, 0x33};
      uint16_t NVM_par_T2;
      uint8_t NVM_par_T1_Address[2] = {0x32, 0x31};
      uint16_t NVM_par_T1;


      //Adrese registra vrednosti za trimovanje P
      uint8_t NVM_par_P1_Address[2] = {0x37, 0x36};
      int16_t NVM_par_P1;
      uint8_t NVM_par_P2_Address[2] = {0x39, 0x38};
      int16_t NVM_par_P2;
      uint8_t NVM_par_P3_Address = 0x3A;
      int8_t NVM_par_P3;
      uint8_t NVM_par_P4_Address = 0x3B;
      int8_t NVM_par_P4;
      uint8_t NVM_par_P5_Address[2] = {0x3D, 0x3C};
      uint16_t NVM_par_P5;
      uint8_t NVM_par_P6_Address[2] = {0x3F, 0x3E};
      uint16_t NVM_par_P6;
      uint8_t NVM_par_P7_Address = 0x40;
      int8_t NVM_par_P7;
      uint8_t NVM_par_P8_Address = 0x41;
      int8_t NVM_par_P8;
      uint8_t NVM_par_P9_Address[2] = {0x43, 0x42};
      int16_t NVM_par_P9;
      uint8_t NVM_par_P10_Address = 0x44;
      int8_t NVM_par_P10;
      uint8_t NVM_par_P11_Address = 0x45;
      int8_t NVM_par_P11;

      write1(&NVM_par_T3, NVM_par_T3_Address);
      write2(&NVM_par_T2, NVM_par_T2_Address);
      write2(&NVM_par_T1, NVM_par_T1_Address);

      write3(&NVM_par_P1, NVM_par_P1_Address);
      write3(&NVM_par_P2, NVM_par_P2_Address);
      write1(&NVM_par_P3, NVM_par_P3_Address);
      write1(&NVM_par_P4, NVM_par_P4_Address);
      write2(&NVM_par_P5, NVM_par_P5_Address);
      write2(&NVM_par_P6, NVM_par_P6_Address);
      write1(&NVM_par_P7, NVM_par_P7_Address);
      write1(&NVM_par_P8, NVM_par_P8_Address);
      write3(&NVM_par_P9, NVM_par_P9_Address);
      write1(&NVM_par_P10, NVM_par_P10_Address);
      write1(&NVM_par_P11, NVM_par_P11_Address);

      PAR_T3 = NVM_par_T3 / (pow(2,48));
      PAR_T2 = NVM_par_T2 / (pow(2,30));
      PAR_T1 = NVM_par_T1 / (pow(2,-8));

      PAR_P1 = (NVM_par_P1 - pow(2,14))/ (pow(2,20));
      PAR_P2 = (NVM_par_P2 - pow(2,14)) / (pow(2,29));
      PAR_P3 = NVM_par_P3 / (pow(2,32));
      PAR_P4 = NVM_par_P4 / (pow(2,37));
      PAR_P5 = NVM_par_P5 / (pow(2,-3));
      PAR_P6 = NVM_par_P6 / (pow(2,6));
      PAR_P7 = NVM_par_P7 / (pow(2,8));
      PAR_P8 = NVM_par_P8 / (pow(2,15));
      PAR_P9 = NVM_par_P9 / (pow(2,48));
      PAR_P10 = NVM_par_P10 / (pow(2,48));
      PAR_P11 = NVM_par_P11 / (pow(2,65));
}

void readValue(float *comp_press, float *t_lin)
{

        uint8_t dataBuffer[5], niz[5];
        uint8_t P1 = 0x06;
        uint8_t P2 = 0x05;
        uint8_t P3 = 0x04;
        uint8_t T1 = 0x09;
        uint8_t T2 = 0x08;
        uint8_t T3 = 0x07;

        uint32_t uncomp_temp, uncomp_p;                                       //Raw vrednosti P i T dobijene sa senzora

        //Citanje P iz registra
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &P1, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead , dataBuffer, 1, HAL_MAX_DELAY);
        niz[0] = dataBuffer[0];
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &P2, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead , dataBuffer, 1, HAL_MAX_DELAY);
        niz[1] = dataBuffer[0];
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &P3, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead  , dataBuffer, 1, HAL_MAX_DELAY);
        niz[2] = dataBuffer[0];

        uncomp_p = 0 | ((uint32_t)niz[0] << 16 | (uint32_t)niz[1]<<8 | (uint32_t)niz[2]);               //Raw P

        //Citanje P iz registra
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &T1, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead , dataBuffer, 1, HAL_MAX_DELAY);
        niz[0] = dataBuffer[0];
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &T2, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead , dataBuffer, 1, HAL_MAX_DELAY);
        niz[1] = dataBuffer[0];
        HAL_I2C_Master_Transmit(&hi2c1, SensorAddressWrite, &T3, 1, HAL_MAX_DELAY);
        HAL_I2C_Master_Receive(&hi2c1, SensorAddressRead  , dataBuffer, 1, HAL_MAX_DELAY);
        niz[2] = dataBuffer[0];
        HAL_Delay(50);

        uncomp_temp = 0 | ((uint32_t)niz[0] << 16 | (uint32_t)niz[1]<<8 | (uint32_t)niz[2]);              //Raw T

        //Trimovanje T
        float partdata1, partdata2, partdata3, partdata4, partout1, partout2;

        partdata1 = (float)(uncomp_temp - PAR_T1);
        partdata2 = (float)(partdata1 * PAR_T2);
        *t_lin = partdata2 + (partdata1 * partdata1)*PAR_T3;                                               //Stvarna temp data u decimalnom zapisu

        
        //Trimovanje P
        partdata1 = PAR_P6 * *t_lin;
        partdata2 = PAR_P7 * (*t_lin * *t_lin);
        partdata3 = PAR_P8 * (*t_lin * *t_lin * *t_lin);
        partout1 = PAR_P5 + partdata1 + partdata2 + partdata3;

        partdata1 = PAR_P2 * *t_lin;
        partdata2 = PAR_P3 * (*t_lin * *t_lin);
        partdata3 = PAR_P4 * (*t_lin * *t_lin* *t_lin);
        partout2 = (float)uncomp_p * (PAR_P1 + partdata1 + partdata2 +partdata3);

        partdata1 = (float)uncomp_p * (float) uncomp_p;
        partdata2 = PAR_P9 + PAR_P10 * *t_lin;
        partdata3 = partdata1 * partdata2;
        partdata4 = partdata3 + ((float)uncomp_p * (float)uncomp_p * (float)uncomp_p) * PAR_P11;
        *comp_press = partout1 + partout2 + partdata4;                                                     //Stvaran pritisak dat u decimalnom zapisu
}
