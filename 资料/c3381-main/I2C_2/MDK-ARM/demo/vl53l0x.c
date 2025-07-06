#include "vl53l0x.h"
#include "gpio.h"

extern vu16 Distance;
VL53L0X_DeviceInfo_t vl53l0x_dev_info;

VL53L0X_Error VL53L0x_Init(VL53L0X_Dev_t *pDev) //ģ���ʼ��
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = pDev;
	
	uint8_t VhvSetting;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	
	
	pMyDevice->I2cDevAddr = 0x52;  //I2C��ַ
	pMyDevice->comms_type = 1;    //I2Cͨ��ģʽ
	pMyDevice->comms_speed_khz = 400;  //I2Cͨ������
	
	XSHUT_LOW;
	delay_ms(30);
	XSHUT_HIGH;
	delay_ms(30);	
  printf("  Addr:0x%x\r\n",pMyDevice->I2cDevAddr);
 
	vl53l0x_Addr_set(pMyDevice,0x54);
	printf("  Addr:0x%x\r\n",pMyDevice->I2cDevAddr);

	Status = VL53L0X_DataInit(pMyDevice);//״̬����ʼ��
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		printf("datainit  failed!  \r\n");
		return Status;
	}
//	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//��ȡ�豸ID��Ϣ
//  if(Status!=VL53L0X_ERROR_NONE) goto error;
//	printf("name:%s ,Type:%s ,ID:%s \r\n",vl53l0x_dev_info.Name,vl53l0x_dev_info.Type,vl53l0x_dev_info.ProductId);

	Status = VL53L0X_StaticInit(pMyDevice);//�豸��ʼ��
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		printf("static  init  failed!\r\n");
		return Status;
	}
  

	Status = VL53L0X_PerformRefCalibration(pMyDevice,&VhvSetting,&PhaseCal);//�¶Ȳο�У׼
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_PerformRefSpadManagement(pMyDevice,&refSpadCount,&isApertureSpads);//Spad�ο�У׼
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_SetDeviceMode(pMyDevice,VL53L0X_DEVICEMODE_SINGLE_RANGING);//���ò���ģʽ
	if(Status!=VL53L0X_ERROR_NONE) goto error;	
	
	error://������Ϣ
	 if(Status!=VL53L0X_ERROR_NONE)
	 {
		print_pal_error(Status);
		return Status;
	 }
	
	return Status;
}

VL53L0X_Error VL53L1_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata)
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 isDataReady = 0;
	uint32_t LoopNb = 0;
	
	
	Status = VL53L0X_StartMeasurement(dev);//��ʼ����
	printf("��ʼ���ԣ�\r\n");
	if(Status != VL53L0X_ERROR_NONE)
		return Status;
	
	do {
		printf("��ʼ׼����\r\n");
	  Status = VL53L0X_GetMeasurementDataReady(dev, &isDataReady);
	  printf("VL53L0X_GetMeasurementDataReady\r\n");

		if(Status != 0)
			break;
		
		if(isDataReady == 1)
			break;
		
		LoopNb++;
		if(LoopNb >= VL53L0X_DEFAULT_MAX_LOOP)
		{
		  Status = VL53L0X_ERROR_TIME_OUT;
		  break;
		}
		
		VL53L0X_PollingDelay(dev);
	}while(1);
	if(isDataReady == 1)
	{
		printf("d: %4imm\r\n",Distance);
	  Status = VL53L0X_GetRangingMeasurementData(dev,pdata);
		printf("VL53L0X_GetRangingMeasurementData\n\r");	
    printf("  Addr:0x%x\r\n",dev->I2cDevAddr);		
		Distance = pdata->RangeMilliMeter;
		printf("d: %4imm\r\n",Distance);//��ӡ��������
		HAL_Delay(2000);
	}
	Status = VL53L0X_ClearInterruptMask(dev,0);
	return Status;
}
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//����Status״̬��ȡ������Ϣ�ַ���
	
  printf("API Status: %i : %s\r\n",Status, buf);//��ӡ״̬�ʹ�����Ϣ
	
}


//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
		return VL53L0X_ERROR_NONE;
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//����I2C��׼ģʽ����
		goto set_error;
	}
	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//����I2C��ַ����
			goto set_error;
		}
		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = FinalAddress;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//��I2C��ַ��д����
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//��ӡ������Ϣ
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}
