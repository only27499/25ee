#include "vl53l0x.h"
#include "gpio.h"

extern vu16 Distance;
VL53L0X_DeviceInfo_t vl53l0x_dev_info;

VL53L0X_Error VL53L0x_Init(VL53L0X_Dev_t *pDev) //模块初始化
{
  VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = pDev;
	
	uint8_t VhvSetting;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	
	
	pMyDevice->I2cDevAddr = 0x52;  //I2C地址
	pMyDevice->comms_type = 1;    //I2C通信模式
	pMyDevice->comms_speed_khz = 400;  //I2C通信速率
	
	XSHUT_LOW;
	delay_ms(30);
	XSHUT_HIGH;
	delay_ms(30);	
  printf("  Addr:0x%x\r\n",pMyDevice->I2cDevAddr);
 
	vl53l0x_Addr_set(pMyDevice,0x54);
	printf("  Addr:0x%x\r\n",pMyDevice->I2cDevAddr);

	Status = VL53L0X_DataInit(pMyDevice);//状态机初始化
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		printf("datainit  failed!  \r\n");
		return Status;
	}
//	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//获取设备ID信息
//  if(Status!=VL53L0X_ERROR_NONE) goto error;
//	printf("name:%s ,Type:%s ,ID:%s \r\n",vl53l0x_dev_info.Name,vl53l0x_dev_info.Type,vl53l0x_dev_info.ProductId);

	Status = VL53L0X_StaticInit(pMyDevice);//设备初始化
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		printf("static  init  failed!\r\n");
		return Status;
	}
  

	Status = VL53L0X_PerformRefCalibration(pMyDevice,&VhvSetting,&PhaseCal);//温度参考校准
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_PerformRefSpadManagement(pMyDevice,&refSpadCount,&isApertureSpads);//Spad参考校准
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
	Status = VL53L0X_SetDeviceMode(pMyDevice,VL53L0X_DEVICEMODE_SINGLE_RANGING);//设置测量模式
	if(Status!=VL53L0X_ERROR_NONE) goto error;	
	
	error://错误信息
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
	
	
	Status = VL53L0X_StartMeasurement(dev);//开始测量
	printf("开始测试：\r\n");
	if(Status != VL53L0X_ERROR_NONE)
		return Status;
	
	do {
		printf("开始准备：\r\n");
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
		printf("d: %4imm\r\n",Distance);//打印测量距离
		HAL_Delay(2000);
	}
	Status = VL53L0X_ClearInterruptMask(dev,0);
	return Status;
}
void print_pal_error(VL53L0X_Error Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
	VL53L0X_GetPalErrorString(Status,buf);//根据Status状态获取错误信息字符串
	
  printf("API Status: %i : %s\r\n",Status, buf);//打印状态和错误信息
	
}


//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//新设备I2C地址与旧地址一致,直接退出
		return VL53L0X_ERROR_NONE;
	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//设置I2C标准模式出错
		goto set_error;
	}
	//尝试使用默认的0x52地址读取一个寄存器
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//读取寄存器出错
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//设置设备新的I2C地址
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//设置I2C地址出错
			goto set_error;
		}
		//修改参数结构体的I2C地址
		dev->I2cDevAddr = FinalAddress;
		//检查新的I2C地址读写是否正常
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//新I2C地址读写出错
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//打印错误信息
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}
