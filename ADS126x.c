/*
 * ADS126x.c
 *
 *  Created on: 		Mar 26, 2014
 *  Last Updated on: 		Sep 13, 2018
 *  Author: 			Michele Gazzarri
 */

#include "ADS126x.h"	// All other required source files are declared here



extern SPI_HandleTypeDef hspi2;
volatile uint8_t ADS126x_Xfer_Completed = 0;
volatile uint8_t ADS126x_Ready_flag = 0;
extern uint8_t ADS126x_Init_Done;
volatile uint8_t ADS126x_DataBuf[ADS126x_DATA_BUF_SIZE];
uint8_t emptyTxBuffer[ADS126x_DATA_BUF_SIZE] = {0,0,0,0,0};


/*****begin low level functions*************************************************/


/* Sets the state of the ADC's /CS pin:
*		0 = LOW 	(active)
*		1 = HIGH	(not selected)
*
*	Asserts:
*		'state' must be '0' or '1'
*/
void ADS126xSetCS(uint8_t state)
{
	if (0 == state)
	{
		HAL_GPIO_WritePin(ADS1263_CS_GPIO_Port, ADS1263_CS_Pin, GPIO_PIN_RESET);
	}
	else if (1 == state)
	{
		HAL_GPIO_WritePin(ADS1263_CS_GPIO_Port, ADS1263_CS_Pin, GPIO_PIN_SET);
	}
	else
		assert(0);		//Aborts program
}


/* Sets the state of the ADC's START pin:
 *		0 = LOW 	(inactive)
 *		1 = HIGH	(converting)
 *
 *	Asserts:
 *		'state' must be '0' or '1'
*/
void set_adc_START(uint8_t state)
{
	if (0 == state)
		ADS126xSendStopCommand();

	else if (1 == state)
		ADS126xSendStartCommand();

	else
		assert(0);		//Aborts program
}

// receive byte, simultaneously send data
unsigned char ADS126xXferByte(uint8_t txData)
{
	//ho visto alcune volte che andava in hard fault dopo aver chiamato questa, se si ripresenta
	//sostituirla con quella non DMA
	uint8_t rxData;
	while(hspi2.State == HAL_SPI_STATE_BUSY);
	HAL_StatusTypeDef s = HAL_SPI_TransmitReceive_DMA( &hspi2, &txData, &rxData, 1);
	return (unsigned char) rxData;
}
unsigned ADS126xXferBytes(uint8_t* txData, uint8_t* rxData, uint16_t lenght) {
	while(hspi2.State == HAL_SPI_STATE_BUSY);
	HAL_StatusTypeDef s = HAL_SPI_TransmitReceive_DMA( &hspi2, txData, rxData, lenght);
	return (unsigned) s;
}

/*****end low level functions***************************************************/

/*****begin higher level functions***********************************************/

/* ADS126xReadData
*
*	Arguments:
*      NumBytes = the number of ADS126x data bytes to read (depends on STATUS and CRC settings)
*      DataByteStartNum = the location of the ADS126x MSB of data in the read operation
*      					= 1 if STATUS is enabled
*      					= 0 if STATUS is disabled
*
*	Asserts:
*      'NumBytes'  must be equal to 4, 5, or 6
*      'DataByteStartNum' must be equal to 0 or
*
*/
ADS126xData ADS126xReadData(uint8_t NumBytes, uint8_t DataByteStartNum)
{
	ADS126xData Channel;

	assert((4 == NumBytes) || (5 == NumBytes) || (6 == NumBytes));			//Aborts program if FALSE
	assert((0 == DataByteStartNum) || (1 == DataByteStartNum));				//Aborts program if FALSE


	uint8_t ADC_Bytes[NumBytes];											//Holds

	ADS126xSetCS(0);

	ADS126xXferByte(RDATA1);

	for(int i = 0; i < NumBytes; ++i)
		ADC_Bytes[i] = ADS126xXferByte(0x00);

	if (NumBytes == 5){
		if(DataByteStartNum==1){
			//We take only status byte
			Channel.Status = ADC_Bytes[0];
		}
		else if(DataByteStartNum==0) {
			//we take only CRC Byte
			Channel.Checksum = ADC_Bytes[5];
		}
	}
	else if(NumBytes == 6){
		//We take both Status and CRC bytes
		Channel.Status = ADC_Bytes[0];
		Channel.Checksum = ADC_Bytes[5];
	}

	Channel.Value = (ADC_Bytes[DataByteStartNum + 0] << 24) |		//Data MSB
					(ADC_Bytes[DataByteStartNum + 1] << 16) |
					(ADC_Bytes[DataByteStartNum + 2] <<  8) |
					(ADC_Bytes[DataByteStartNum + 3] <<  0);		//Data LSB
	ADS126xSetCS(1);

	return Channel;
}

//// Reads data and writes to registers concurrently
//void ADS126xREADandWRITE(int NumDatBytes, int StartAddress, int NumRegs, unsigned char * pdata)
//{
//	uint8_t ADC_Bytes[6];
//	int32_t ADC_Data_Only;
//
//	WaitForDRDY();
//
//	ADS126xSetCS(0);
//
//	ADS126xXferByte(RDATA1);
//
//	ADS126xXferByte(0x40+StartAddress);
//	ADS126xXferByte(NumRegs-1);
//	for(int i=0;i<NumRegs;i++)
//	{
//		ADS126xXferByte(pdata[i]);
//	}
//	ADS126xSetCS(1);
//}



//unsigned char ADS126xReadADC2Data(bufferType_t *readbuffer){
//
//	ADS126xSetCS(0);
//	ADS126xXferByte(RDATA2);
//	readbuffer->data.status=ADS126xXferByte(0);
//	readbuffer->data.payload.bytes[3]=ADS126xXferByte(0);
//	readbuffer->data.payload.bytes[2]=ADS126xXferByte(0);
//	readbuffer->data.payload.bytes[1]=ADS126xXferByte(0);
//	readbuffer->data.payload.bytes[0]=ADS126xXferByte(0);
//	readbuffer->data.checksum=ADS126xXferByte(0);
//	readbuffer->data.adcindicator=0x80|g_packetCounter;
//	if(g_packetCounter<0x3F)
//		g_packetCounter++;
//	else
//		g_packetCounter=0;
//	ADS126xSetCS(1);
//	return readbuffer->data.checksum; //return CRC
//}


// write a number of consecutive registers from a given array pointer
void ADS126xWriteMultiRegister(int StartAddress, int NumRegs, unsigned char * pdata)
{
	uint8_t data[NumRegs+2], prx[NumRegs+2];
	ADS126xSetCS(0);
	data[0] = 0x40+StartAddress;
	data[1] = NumRegs-1;
	for(int i=0;i<NumRegs;i++)
		data[i+2]=pdata[i];
	ADS126x_Xfer_Completed = 0;
	ADS126xXferBytes(&data[0], &prx[0], NumRegs+2);
	//Ensure the DMA has finished transer before take CS High
	while(ADS126x_Xfer_Completed != 1);
	ADS126xSetCS(1);
}
//write single register funcion
void ADS126xWriteRegister(uint8_t reg, uint8_t data){
	ADS126xWriteMultiRegister(reg, 1, &data);
}
// read a number of consecutive registers to a given array pointer
void ADS126xReadRegister(int StartAddress, int NumRegs, unsigned char *pdata)
{
	ADS126xSetCS(0);
	ADS126xXferByte(0x20 + StartAddress);
	ADS126xXferByte(NumRegs - 1);
	for(int i=0;i<NumRegs;i++)
	{
		pdata[i] = ADS126xXferByte(0x00);
	}
	ADS126xSetCS(1);
}

// Reset by command (alternative to pin)
void ADS126xSendResetCommand(void){
	ADS126xSetCS(0);
	ADS126xXferByte(RESET1);
	ADS126xSetCS(1);
}

// Start by command (alternative to pin)
void ADS126xSendStartCommand(void){
	ADS126xSetCS(0);
	ADS126xXferByte(START1);
	ADS126xSetCS(1);
}

// Stop by command (alternative to pin)
void ADS126xSendStopCommand(void){
	ADS126xSetCS(0);
	ADS126xXferByte(STOP1);
	ADS126xSetCS(1);
}

// Start by command
void ADS126xSendADC2StartCommand(void){
	ADS126xSetCS(0);
	ADS126xXferByte(START2);
	ADS126xSetCS(1);
}

// Stop by command
void ADS126xSendADC2StopCommand(void){
	ADS126xSetCS(0);
	ADS126xXferByte(STOP2);
	ADS126xSetCS(1);
}

// not implemented yet
void ADS126xShutdown(void){
	//pull PowerDown Pin low
}

// not implemented yet
void ADS126xWake(void){
	//pull PowerDown Pin high
}
void ADS126xInitADC1(void){
	ADS126xSendResetCommand();
	HAL_Delay(50);

	ADS126xSendStopCommand();
	ADS126xSendADC2StopCommand();
	HAL_Delay(10);

	uint8_t AdcRegData[ADS126x_NUM_REG];							//Stores the register read values
	uint8_t WriteRegData[ADS126x_NUM_REG];							//Stores the register write values

	ADS126xReadRegister(ID, ADS126x_NUM_REG, &AdcRegData[0]);			//Read ALL registers

	/* Configure Register Settings */
	WriteRegData[ID] = AdcRegData[ID];								//ID
	WriteRegData[POWER1] = POWER_DEFAULT_VALUE;						//RESET = 0, INTREF = 1
	WriteRegData[INTERFACE] = STATUS;								//status on, crc off
	WriteRegData[MODE0] = MODE0_DEFAULT_VALUE;						//
	WriteRegData[MODE1] = MODE1_DEFAULT_VALUE;						//
	WriteRegData[MODE2] = (0x5B);									//PGA enable, Gain 32, 4800SPS
	WriteRegData[INPMUX] = (0x01);									// +:AIN0, -:AIN1
	WriteRegData[OFCAL0] = OFCAL0_DEFAULT_VALUE;					//OFCAL0 (reset to default)
	WriteRegData[OFCAL1] = OFCAL1_DEFAULT_VALUE;					//OFCAL1 (reset to default)
	WriteRegData[OFCAL2] = OFCAL2_DEFAULT_VALUE;					//OFCAL2 (reset to default)
	WriteRegData[FSCAL0] = FSCAL0_DEFAULT_VALUE;					//FSCAL0 (reset to default)
	WriteRegData[FSCAL1] = FSCAL1_DEFAULT_VALUE;					//FSCAL1 (reset to default)
	WriteRegData[FSCAL2] = FSCAL2_DEFAULT_VALUE;					//FSCAL2 (reset to default)
	WriteRegData[IDACMUX] = IDACMUX_DEFAULT_VALUE;					//IDACMUX (IDAC1MUX = AINCOM)
	WriteRegData[IDACMAG] = IDACMAG_DEFAULT_VALUE;					//IDACMAG (IDAC1MAG = 500 uA)
	WriteRegData[REFMUX] = RMUXP_AVDD | RMUXN_AVSS;					//REFMUX (REFP = AIN0, REFN = AIN3)
	WriteRegData[TDACP] = TDACP_DEFAULT_VALUE;						//TDACP (reset to default)
	WriteRegData[TDACN] = TDACN_DEFAULT_VALUE;						//TDACN (reset to default)
	WriteRegData[GPIOCON] = GPIOCON_DEFAULT_VALUE;					//GPIOCON (Enable GPIOs on AIN8 & AIN9)
	WriteRegData[GPIODIR] = GPIODIR_DEFAULT_VALUE;					//GPIODIR (reset to default)
	WriteRegData[GPIODAT] = GPIODAT_DEFAULT_VALUE;								//GPIODAT (Biases bridge with + polarity)
#ifdef ADS1263
	WriteRegData[ADC2CFG] = (0xA7);									//400SPS, Analog ref, gain 128
	WriteRegData[ADC2MUX] = (0x23);									//ain2, ain3
	WriteRegData[ADC2OFC0] = ADC2OFC0_DEFAULT_VALUE;				//ADC2OFC0 (reset to default)
	WriteRegData[ADC2OFC1] = ADC2OFC1_DEFAULT_VALUE;				//ADC2OFC1 (reset to default)
	WriteRegData[ADC2FSC0] = ADC2FSC0_DEFAULT_VALUE;				//ADC2FSC0 (reset to default)
	WriteRegData[ADC2FSC1] = ADC2FSC1_DEFAULT_VALUE;				//ADC2FSC1 (reset to default)
#endif

	ADS126xWriteMultiRegister(ID, ADS126x_NUM_REG, &WriteRegData[0]);	//Write ALL registers
	HAL_Delay(5);

	ADS126xSendStartCommand();
	HAL_Delay(10);

}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi -> Instance == SPI2){
		ADS126x_Xfer_Completed = 1;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ADS126x_DRDY_Pin && ADS126x_Init_Done == 1) {
		ADS126x_Ready_flag = 1;
		//ADS126xSetCS(0);
		HAL_SPI_TransmitReceive_DMA(&hspi2, &emptyTxBuffer[0], &ADS126x_DataBuf[0], ADS126x_DATA_BUF_SIZE);
		//ADS126xSetCS(1);
	}
}

/*****end higher level functions*************************************************/
