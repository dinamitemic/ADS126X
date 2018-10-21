/*
 * ADS126x.h
 *
 *  Created on: 		Mar 26, 2014
 *  Last Updated on: 	Sep 13, 2018
 *  Author: 			Michele Gazzarri
 *
 *  NOTES:
 *  	TODO: Add & correct comments
 *
 */

#ifndef ADS126X_H_
#define ADS126X_H_


// C Standard Libraries
#include <assert.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#define ADS126x_DRDY_Pin GPIO_PIN_11
#define ADS1263_CS_Pin GPIO_PIN_12
#define ADS1263_CS_GPIO_Port GPIOB
//this depends on how many bytes is composed the incoming data
//(1 status byte + 4 byte of data)
#define ADS126x_DATA_BUF_SIZE 5

// SELECT A DEVICE
//#define ADS1262	//Standard definitions for both ADS1262 and ADS1263 devices
#define ADS1263		//Additional definitions to support ADS1263 additional features

typedef struct {
	long int Value;
	uint8_t Status;
	uint8_t Checksum;
} ADS126xData;

//BEGIN ADC DEFINITIONS

#ifdef ADS1262
	#define ADS126x_NUM_REG 					(0x15)			/* ADS1262 has 21 registers */
#endif
#ifdef ADS1263
	#define ADS126x_NUM_REG 					(0x1B)			/* ADS1263 has 27 registers */
#endif


/* SPI Commands */
	#define NOP									(0x00)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define RESET1								(0x06)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define START1								(0x08)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define STOP1								(0x0B)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define RDATA1								(0x12)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define SYOCAL1								(0x16)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define SYGCAL1								(0x17)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define SFOCAL1								(0x19)			/* ID/CFG REGISTER (ADDRESS 00h) */
	//Multi-Byte Commands
		#define RREG							(0x20)			/* ID/CFG REGISTER (ADDRESS 00h) */
		#define WREG							(0x40)			/* ID/CFG REGISTER (ADDRESS 00h) */
	/* Additional ADS1263 Commands */
	#ifdef ADS1263
		#define START2							(0x0C)			/* ID/CFG REGISTER (ADDRESS 00h) */
		#define STOP2							(0x0E)			/* ID/CFG REGISTER (ADDRESS 00h) */
		#define RDATA2							(0x14)			/* ID/CFG REGISTER (ADDRESS 00h) */
		#define SYOCAL2							(0x1B)			/* ID/CFG REGISTER (ADDRESS 00h) */
		#define SYGCAL2							(0x1C)			/* ID/CFG REGISTER (ADDRESS 00h) */
		#define SFOCAL2							(0x1E)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#endif



/* STATUS Byte Masks */
	#define	ADC2_NEW							(0x80)			/* Indicates new ADC2 data */
	#define	ADC1_NEW							(0x40)			/* Indicates new ADC1 data */
	#define	EXTCLK								(0x20)			/* Indicates ADC clock source */
	#define	REF_ALM								(0x10)			/* Low Reference Alarm   - Only used with ADC1 */
	#define	PGAL_ALM							(0x08)			/* PGA Output Low Alarm  - Only used with ADC1 */
	#define	PGAH_ALM							(0x04)			/* PGA Output High Alarm - Only used with ADC1 */
	#define	PGAD_ALM							(0x02)			/* PGA Diff Output Alarm - Only used with ADC1 */
	#define	RST_ALM								(0x01)			/* Indicates device reset (re-named to avoid conflict) */


/* Register Addresses */
	#define ID									(0x00)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define POWER1 								(0x01)			/* CONFIGURATION REGISTER 0 (ADDRESS 01h) */
	#define INTERFACE							(0x02)			/* CONFIGURATION REGISTER 1 (ADDRESS 02h) */
	#define MODE0								(0x03)			/* High-Pass Filter Corner Frequency, Low Byte (Address 03h) */
	#define MODE1								(0x04)			/* High-Pass Filter Corner Frequency, High Byte (Address 04h) */
	#define MODE2								(0x05)			/* Offset Calibration, Low Byte (Address 05h) */
	#define INPMUX								(0x06)			/* Offset Calibration, Mid Byte (Address 06h) */
	#define OFCAL0								(0x07)			/* Offset Calibration, High Byte (Address 07h) */
	#define OFCAL1								(0x08)			/* Full-Scale Calibration, Low Byte (Address 08h) */
	#define OFCAL2								(0x09)			/* Full-Scale Calibration, Mid Byte (Address 09h) */
	#define FSCAL0								(0x0A)			/* Full-Scale Calibration, High Byte (Address 0Ah) */
	#define FSCAL1								(0x0B)
	#define FSCAL2								(0x0C)
	#define IDACMUX								(0x0D)
	#define IDACMAG								(0x0E)
	#define REFMUX								(0x0F)
	#define TDACP								(0x10)
	#define TDACN								(0x11)
	#define GPIOCON								(0x12)
	#define GPIODIR								(0x13)
	#define GPIODAT								(0x14)
	/* Additional ADS1263 Registers */
	#ifdef ADS1263
		#define ADC2CFG							(0x15)
		#define ADC2MUX							(0x16)
		#define ADC2OFC0						(0x17)
		#define ADC2OFC1						(0x18)
		#define ADC2FSC0						(0x19)
		#define ADC2FSC1						(0x1A)
	#endif /* ADS1263 */


/* Default Register Values */
//	#define ID_DEFAULT_VALUE					(0x00)			/* ID/CFG REGISTER (ADDRESS 00h) */
	#define POWER_DEFAULT_VALUE					(0x19)			/* CONFIGURATION REGISTER 0 (ADDRESS 01h) */
	#define INTERFACE_DEFAULT_VALUE				(0x05)			/* CONFIGURATION REGISTER 1 (ADDRESS 02h) */
	#define MODE0_DEFAULT_VALUE					(0x00)			/* High-Pass Filter Corner Frequency, Low Byte (Address 03h) */
	#define MODE1_DEFAULT_VALUE					(0x80)			/* High-Pass Filter Corner Frequency, High Byte (Address 04h) */
	#define MODE2_DEFAULT_VALUE					(0x04)			/* Offset Calibration, Low Byte (Address 05h) */
	#define INPMUX_DEFAULT_VALUE				(0x01)			/* Offset Calibration, Mid Byte (Address 06h) */
	#define OFCAL0_DEFAULT_VALUE				(0x00)			/* Offset Calibration, High Byte (Address 07h) */
	#define OFCAL1_DEFAULT_VALUE				(0x00)			/* Full-Scale Calibration, Low Byte (Address 08h) */
	#define OFCAL2_DEFAULT_VALUE				(0x00)			/* Full-Scale Calibration, Mid Byte (Address 09h) */
	#define FSCAL0_DEFAULT_VALUE				(0x00)			/* Full-Scale Calibration, High Byte (Address 0Ah) */
	#define FSCAL1_DEFAULT_VALUE				(0x00)
	#define FSCAL2_DEFAULT_VALUE				(0x40)
	#define IDACMUX_DEFAULT_VALUE				(0xBB)
	#define IDACMAG_DEFAULT_VALUE				(0x00)
	#define REFMUX_DEFAULT_VALUE				(0x00)
	#define TDACP_DEFAULT_VALUE					(0x00)
	#define TDACN_DEFAULT_VALUE					(0x00)
	#define GPIOCON_DEFAULT_VALUE				(0x00)
	#define GPIODIR_DEFAULT_VALUE				(0x00)
	#define GPIODAT_DEFAULT_VALUE				(0x00)

	#ifdef ADS1263
		#define ADC2CFG_DEFAULT_VALUE			(0x00)
		#define ADC2MUX_DEFAULT_VALUE			(0x01)
		#define ADC2OFC0_DEFAULT_VALUE			(0x00)
		#define ADC2OFC1_DEFAULT_VALUE			(0x00)
		#define ADC2FSC0_DEFAULT_VALUE			(0x00)
		#define ADC2FSC1_DEFAULT_VALUE			(0x40)
	#endif /* ADS1263 */


/* Register Field Definitions */

	/* POWER Register Fields */
		#define RST 							(0x10)
		#define VBIAS 							(0x02)
		#define INTREF 							(0x01)

	/* INTERFACE Register Fields */
		#define TIMEOUT 						(0x08)
		#define STATUS 							(0x04)
//		#define CRC1 							(0x02)
//		#define CRC0 							(0x01)
		#define CRC_MASK						(0x03)
			//CRC Field
			#define CRC_OFF						(0x00)
			#define CRC_CHKSUM					(0x01)
			#define CRC_ON						(0x02)

	/* MODE0 Register Fields */
		#define REFREV							(0x80)
		#define RUNMODE							(0x40)
//		#define CHOP1 							(0x20)
//		#define CHOP0 							(0x10)
		#define CHOP_MASK						(0x30)
			//CHOP Field
			#define CHOP_OFF					(0x00)
			#define CHOP_ON						(0x10)
			#define CHOP_IDAC					(0x20)
			#define CHOP_ON_IDAC				(0x30)
//		#define DELAY3 							(0x08)
//		#define DELAY2 							(0x04)
//		#define DELAY1 							(0x02)
//		#define DELAY0 							(0x01)
		#define DELAY_MASK						(0x0F)			/* Additional Settling Delay Field */
			//DELAY Field
			#define DELAY_0us					(0x00)
			#define DELAY_8_7us					(0x01)
			#define DELAY_17us					(0x02)
			#define DELAY_35us					(0x03)
			#define DELAY_69us					(0x04)
			#define DELAY_139us					(0x05)
			#define DELAY_278us					(0x06)
			#define DELAY_555us					(0x07)
			#define DELAY_1100us				(0x08)
			#define DELAY_2200us				(0x09)
			#define DELAY_4400us				(0x0A)
			#define DELAY_8800us				(0x0B)

	/* MODE1 Register Fields */
//		#define FILTER_2 						(0x80)
//		#define FILTER_1 						(0x40)
//		#define FILTER_0 						(0x20)
		#define FILTER_MASK						(0xE0)
			//DELAY Field
			#define FILTER_SINC1				(0x00)
			#define FILTER_SINC2				(0x20)
			#define FILTER_SINC3				(0x40)
			#define FILTER_SINC4				(0x60)
			#define FILTER_FIR					(0x80)
		#define SBADC 							(0x10)
		#define SBPOL 							(0x08)
//		#define SBMAG2 							(0x04)
//		#define SBMAG1 							(0x02)
//		#define SBMAG0 							(0x01)
		#define SBMAG_MASK						(0x07)
			//SBMAG Field
			#define SBMAG_0uA					(0x00)
			#define SBMAG_0_5uA					(0x01)
			#define SBMAG_2uA					(0x02)
			#define SBMAG_10uA					(0x03)
			#define SBMAG_50uA					(0x04)
			#define SBMAG_200uA					(0x05)
			#define SBMAG_10MOhm				(0x06)

	/* MODE2 Register Fields */
		#define BYPASS							(0x80)
//		#define GAIN2 							(0x40)
//		#define GAIN1 							(0x20)
//		#define GAIN0 							(0x10)
		#define GAIN_MASK						(0x70)
			//GAIN Field
			#define GAIN_1						(0x00)
			#define GAIN_2						(0x10)
			#define GAIN_4						(0x20)
			#define GAIN_8						(0x30)
			#define GAIN_16						(0x40)
			#define GAIN_32						(0x50)
//		#define DR3 							(0x08)
//		#define DR2 							(0x04)
//		#define DR1 							(0x02)
//		#define DR0 							(0x01)
		#define DR_MASK							(0x0F)
			//DR Field
			#define DR_2_5_SPS					(0x00)
			#define DR_5_SPS					(0x01)
			#define DR_10_SPS					(0x02)
			#define DR_16_6_SPS					(0x03)
			#define DR_20_SPS					(0x04)
			#define DR_50_SPS					(0x05)
			#define DR_60_SPS					(0x06)
			#define DR_100_SPS					(0x07)
			#define DR_400_SPS					(0x08)
			#define DR_1200_SPS					(0x09)
			#define DR_2400_SPS					(0x0A)
			#define DR_4800_SPS					(0x0B)
			#define DR_7200_SPS					(0x0C)
			#define DR_14400_SPS				(0x0D)
			#define DR_19200_SPS				(0x0E)
			#define DR_38400_SPS				(0x0F)


	/* INPMUX Register Fields */
//		#define MUXP_3 							(0x80)
//		#define MUXP_2 							(0x40)
//		#define MUXP_1 							(0x20)
//		#define MUXP_0 							(0x10)
		#define MUXP_MASK						(0xF0)
			//MUXP Field
			#define MUXP_AIN0					(0x00)
			#define MUXP_AIN1					(0x10)
			#define MUXP_AIN2					(0x20)
			#define MUXP_AIN3					(0x30)
			#define MUXP_AIN4					(0x40)
			#define MUXP_AIN5					(0x50)
			#define MUXP_AIN6					(0x60)
			#define MUXP_AIN7					(0x70)
			#define MUXP_AIN8					(0x80)
			#define MUXP_AIN9					(0x90)
			#define MUXP_AINCOM					(0xA0)
			#define MUXP_TEMP					(0xB0)
			#define MUXP_AVDD					(0xC0)
			#define MUXP_DVDD					(0xD0)
			#define MUXP_TEST					(0xE0)
			#define MUXP_NO_CONN				(0xF0)
//		#define MUXN_3 							(0x08)
//		#define MUXN_2 							(0x04)
//		#define MUXN_1 							(0x02)
//		#define MUXN_0 							(0x01)
		#define MUXN_MASK						(0x0F)
			//MUXN Field
			#define MUXN_AIN0					(0x00)
			#define MUXN_AIN1					(0x01)
			#define MUXN_AIN2					(0x02)
			#define MUXN_AIN3					(0x03)
			#define MUXN_AIN4					(0x04)
			#define MUXN_AIN5					(0x05)
			#define MUXN_AIN6					(0x06)
			#define MUXN_AIN7					(0x07)
			#define MUXN_AIN8					(0x08)
			#define MUXN_AIN9					(0x09)
			#define MUXN_AINCOM					(0x0A)
			#define MUXN_TEMP					(0x0B)
			#define MUXN_AVSS					(0x0C)
			#define MUXN_DVDD					(0x0D)
			#define MUXN_TEST					(0x0E)
			#define MUXN_NO_CONN				(0x0F)

	//SKIP OFFSET & GAIN CAL REGISTERS

	/* IDACMUX Register Fields */
//		#define MUX2_3 							(0x80)
//		#define MUX2_2 							(0x40)
//		#define MUX2_1  						(0x20)
//		#define MUX2_0 							(0x10)
		#define MUX2_MASK						(0xF0)
			//MUX2 Field
			#define MUX2_AIN0					(0x00)
			#define MUX2_AIN1					(0x10)
			#define MUX2_AIN2					(0x20)
			#define MUX2_AIN3					(0x30)
			#define MUX2_AIN4					(0x40)
			#define MUX2_AIN5					(0x50)
			#define MUX2_AIN6					(0x60)
			#define MUX2_AIN7					(0x70)
			#define MUX2_AIN8					(0x80)
			#define MUX2_AIN9					(0x90)
			#define MUX2_AINCOM					(0xA0)
			#define MUX2_NO_CONM				(0xB0)
//		#define MUX1_3 							(0x08)
//		#define MUX1_2 							(0x04)
//		#define MUX1_1 							(0x02)
//		#define MUX1_0 							(0x01)
		#define MUX1_MASK						(0x0F)
			//MUX1 Field
			#define MUX1_AIN0					(0x00)
			#define MUX1_AIN1					(0x01)
			#define MUX1_AIN2					(0x02)
			#define MUX1_AIN3					(0x03)
			#define MUX1_AIN4					(0x04)
			#define MUX1_AIN5					(0x05)
			#define MUX1_AIN6					(0x06)
			#define MUX1_AIN7					(0x07)
			#define MUX1_AIN8					(0x08)
			#define MUX1_AIN9					(0x09)
			#define MUX1_AINCOM					(0x0A)
			#define MUX1_NO_CONM				(0x0B)

	/* IDACMAG Register Fields */
//		#define MAG2_3 							(0x80)
//		#define MAG2_2 							(0x40)
//		#define MAG2_1 							(0x20)
//		#define MAG2_0 							(0x10)
		#define MAG2_MASK						(0xF0)
			//MAG2 Field
			#define MAG2_OFF					(0x00)
			#define MAG2_50uA					(0x10)
			#define MAG2_100uA					(0x20)
			#define MAG2_250uA					(0x30)
			#define MAG2_500uA					(0x40)
			#define MAG2_750uA					(0x50)
			#define MAG2_1000uA					(0x60)
			#define MAG2_1500uA					(0x70)
			#define MAG2_2000uA					(0x80)
			#define MAG2_2500uA					(0x90)
			#define MAG2_3000uA					(0xA0)
//		#define MAG1_3							(0x08)
//		#define MAG1_2 							(0x04)
//		#define MAG1_1 							(0x02)
//		#define MAG1_0 							(0x01)
		#define MAG1_MASK						(0xF0)
			//MAG1 Field
			#define MAG1_OFF					(0x00)
			#define MAG1_50uA					(0x01)
			#define MAG1_100uA					(0x02)
			#define MAG1_250uA					(0x03)
			#define MAG1_500uA					(0x04)
			#define MAG1_750uA					(0x05)
			#define MAG1_1000uA					(0x06)
			#define MAG1_1500uA					(0x07)
			#define MAG1_2000uA					(0x08)
			#define MAG1_2500uA					(0x09)
			#define MAG1_3000uA					(0x0A)

	/* REFMUX Register Fields */
//		#define RMUXP_2	 						(0x20)
//		#define RMUXP_1	 						(0x10)
//		#define RMUXP_0	 						(0x08)
		#define RMUXP_MASK						(0x38)
			//MUXP Field
			#define RMUXP_INTP					(0x00)
			#define RMUXP_AIN0					(0x08)
			#define RMUXP_AIN2					(0x10)
			#define RMUXP_AIN4					(0x18)
			#define RMUXP_AVDD					(0x20)
//		#define RMUXN_2	 						(0x04)
//		#define RMUXN_1	 						(0x02)
//		#define RMUXN_0	 						(0x01)
		#define RMUXN_MASK						(0x07)
			//MUXN Field
			#define RMUXN_INTN					(0x00)
			#define RMUXN_AIN1					(0x01)
			#define RMUXN_AIN3					(0x02)
			#define RMUXN_AIN5					(0x03)
			#define RMUXN_AVSS					(0x04)

	/* TDACP Register Fields */
		#define OUTP	 						(0x80)
		//OUTP Field
			#define OUTP_NO_CONN				(0x00)
			#define OUTP_AIN6					(0x80)
//		#define MAGP4	 						(0x10)
//		#define MAGP3	 						(0x08)
//		#define MAGP2	 						(0x04)
//		#define MAGP1	 						(0x02)
//		#define MAGP0	 						(0x01)
		#define MAGP_MASK						(0x1F)
			//MAGP Field
			#define MAGP_0_9_AVDD				(0x09)
			#define MAGP_0_7_AVDD				(0x08)
			#define MAGP_0_6_AVDD				(0x07)
			#define MAGP_0_55_AVDD				(0x06)
			#define MAGP_0_525_AVDD				(0x05)
			#define MAGP_0_5125_AVDD			(0x04)
			#define MAGP_0_50625_AVDD			(0x03)
			#define MAGP_0_503125_AVDD			(0x02)
			#define MAGP_0_5015625_AVDD			(0x01)
			#define MAGP_0_5_AVDD				(0x00)
			#define MAGP_0_4984375_AVDD			(0x11)
			#define MAGP_0_496875_AVDD			(0x12)
			#define MAGP_0_49375_AVDD			(0x13)
			#define MAGP_0_4875_AVDD			(0x14)
			#define MAGP_0_475_AVDD				(0x15)
			#define MAGP_0_45_AVDD				(0x16)
			#define MAGP_0_4_AVDD				(0x17)
			#define MAGP_0_3_AVDD				(0x18)
			#define MAGP_0_1_AVDD				(0x19)

	/* TDACN Register Fields */
		#define OUTN	 						(0x80)
		//OUTN Field
			#define OUTN_NO_CONN				(0x00)
			#define OUTN_AIN7					(0x80)
//		#define MAGN4	 						(0x10)
//		#define MAGN3	 						(0x08)
//		#define MAGN2	 						(0x04)
//		#define MAGN1	 						(0x02)
//		#define MAGN0	 						(0x01)
		#define MAGN_MASK						(0x1F)
			//MAGN Field
			#define MAGN_0_9_AVSS				(0x09)
			#define MAGN_0_7_AVSS				(0x08)
			#define MAGN_0_6_AVSS				(0x07)
			#define MAGN_0_55_AVSS				(0x06)
			#define MAGN_0_525_AVSS				(0x05)
			#define MAGN_0_5125_AVSS			(0x04)
			#define MAGN_0_50625_AVSS			(0x03)
			#define MAGN_0_503125_AVSS			(0x02)
			#define MAGN_0_5015625_AVSS			(0x01)
			#define MAGN_0_5_AVSS				(0x00)
			#define MAGN_0_4984375_AVSS			(0x11)
			#define MAGN_0_496875_AVSS			(0x12)
			#define MAGN_0_49375_AVSS			(0x13)
			#define MAGN_0_4875_AVSS			(0x14)
			#define MAGN_0_475_AVSS				(0x15)
			#define MAGN_0_45_AVSS				(0x16)
			#define MAGN_0_4_AVSS				(0x17)
			#define MAGN_0_3_AVSS				(0x18)
			#define MAGN_0_1_AVSS				(0x19)

	/* GPIOCON Register Fields */
		#define CON7_AINCOM						(0x80)
		#define CON6_AIN09						(0x40)
		#define CON5_AIN08						(0x20)
		#define CON4_AIN07						(0x10)
		#define CON3_AIN06						(0x08)
		#define CON2_AIN05						(0x04)
		#define CON1_AIN04						(0x02)
		#define CON0_AIN03						(0x01)

	/* GPIODIR Register Fields */
		#define DIR7_AINCOM						(0x80)
		#define DIR6_AIN09						(0x40)
		#define DIR5_AIN08						(0x20)
		#define DIR4_AIN07						(0x10)
		#define DIR3_AIN06						(0x08)
		#define DIR2_AIN05						(0x04)
		#define DIR1_AIN04						(0x02)
		#define DIR0_AIN03						(0x01)

	/* GPIODAT Register Fields */
		#define DAT7_AINCOM						(0x80)
		#define DAT6_AIN09						(0x40)
		#define DAT5_AIN08						(0x20)
		#define DAT4_AIN07						(0x10)
		#define DAT3_AIN06						(0x08)
		#define DAT2_AIN05						(0x04)
		#define DAT1_AIN04						(0x02)
		#define DAT0_AIN03						(0x01)


		/* Additional ADS1263 Registers */
		#ifdef ADS1263

			/* ADC2CFG Register Fields */
//					#define DR2_1	 			(0x80)
//					#define DR2_0	 			(0x40)
			#define DR2_MASK					(0xC0)
				//DR_2 Field
					#define DR2_10SPS			(0x00)
					#define DR2_100SPS			(0x40)
					#define DR2_400SPS			(0x80)
					#define DR2_800SPS			(0xC0)
//					#define REF2_2	 			(0x20)
//					#define REF2_1				(0x10)
//					#define REF2_0	 			(0x08)
			#define REF2_MASK					(0x38)
				//REF2 Field
					#define REF2_INTP_INTN		(0x00)
					#define REF2_AIN0_AIN1		(0x08)
					#define REF2_AIN2_AIN3		(0x10)
					#define REF2_AIN4_AIN5		(0x18)
					#define REF2_AVDD_AVSS		(0x38)
//					#define GAIN2_2	 			(0x04)
//					#define GAIN2_1				(0x02)
//					#define GAIN2_0				(0x01)
			#define GAIN2_MASK					(0x07)
				//GAIN2 Field
					#define GAIN2_1				(0x00)
					#define GAIN2_2				(0x01)
					#define GAIN2_4				(0x02)
					#define GAIN2_8				(0x03)
					#define GAIN2_16			(0x04)
					#define GAIN2_32			(0x05)
					#define GAIN2_64			(0x06)
					#define GAIN2_128			(0x07)


			/* ADC2MUX Register Fields */
//					#define MUXP2_3	 			(0x80)
//					#define MUXP2_2	 			(0x40)
//					#define MUXP2_1	 			(0x20)
//					#define MUXP2_0				(0x10)
				#define MUXP2_MASK				(0xF0)
				//MUXP2 Field
					#define MUXP2_AIN0			(0x00)
					#define MUXP2_AIN1			(0x10)
					#define MUXP2_AIN2			(0x20)
					#define MUXP2_AIN3			(0x30)
					#define MUXP2_AIN4			(0x40)
					#define MUXP2_AIN5			(0x50)
					#define MUXP2_AIN6			(0x60)
					#define MUXP2_AIN7			(0x70)
					#define MUXP2_AIN8			(0x80)
					#define MUXP2_AIN9			(0x90)
					#define MUXP2_AINCOM		(0xA0)
					#define MUXP2_TEMP			(0xB0)
					#define MUXP2_AVDD			(0xC0)
					#define MUXP2_DVDD			(0xD0)
					#define MUXP2_TEST			(0xE0)
					#define MUXP2_NO_CONN		(0xF0)
//					#define MUXN2_3	 			(0x08)
//					#define MUXN2_2	 			(0x04)
//					#define MUXN2_1				(0x02)
//					#define MUXN2_0				(0x01)
				#define MUXN2_MASK				(0x0F)
					//MUXN2 Field
					#define MUXN2_AIN0			(0x00)
					#define MUXN2_AIN1			(0x01)
					#define MUXN2_AIN2			(0x02)
					#define MUXN2_AIN3			(0x03)
					#define MUXN2_AIN4			(0x04)
					#define MUXN2_AIN5			(0x05)
					#define MUXN2_AIN6			(0x06)
					#define MUXN2_AIN7			(0x07)
					#define MUXN2_AIN8			(0x08)
					#define MUXN2_AIN9			(0x09)
					#define MUXN2_AINCOM		(0x0A)
					#define MUXN2_TEMP			(0x0B)
					#define MUXN2_AVSS			(0x0C)
					#define MUXN2_DVDD			(0x0D)
					#define MUXN2_TEST			(0x0E)
					#define MUXN2_NO_CONN		(0x0F)

				//SKIP ADC2 OFFSET & GAIN CAL REGISTERS

		#endif /* ADS1263 */

//END ADC DEFINITIONS


/* Function Prototypes */

// Low level

void ADS126xSetCS(uint8_t state);						// CS pin control
void set_adc_START(uint8_t state);					// START pin control
unsigned char ADS126xXferByte (uint8_t cData);	// receive byte, simultaneously send data - this function realizes all
													// necessary functionality, the other Send/Receive methods are only
													// designed to improve readability of the code
unsigned ADS126xXferBytes(uint8_t* txData, uint8_t* rxData, uint16_t lenght);

// Higher level

ADS126xData ADS126xReadData(uint8_t NumBytes, uint8_t DataByteStartNum);

//int32_t ADS126xREADandWRITE(int NumDatBytes, int StartAddress, int NumRegs, unsigned char * pdata);
//unsigned char ADS126xReadADC2Data(bufferType_t *readbuffer);

// read a number of consecutive registers to a given array pointer
void ADS126xReadRegister(int StartAddress, int NumRegs, unsigned char *pdata);

// write a number of consecutive registers from a given array pointer
void ADS126xWriteMultiRegister(int StartAddress, int NumRegs, unsigned char *pdata);

// Reset by command (alternative to pin)
void ADS126xSendResetCommand(void);

// Start by command (alternative to pin)
void ADS126xSendStartCommand(void);

void ADS126xSendStopCommand(void);
void ADS126xSendADC2StartCommand(void);
void ADS126xSendADC2StopCommand(void);
void ADS126xInitADC1(void);
void ADS126xWriteRegister(uint8_t, uint8_t);



#endif /* ADS126X_H_ */
