#include "efm32.h"
#include "em_chip.h"
#include "em_timer.h"

#include "config_device.h"
#include "em_gpio.h"
#include "string.h"

#define NVIC_VectTab_RAM             ((uint32_t)0x20000000)
#define NVIC_VectTab_FLASH           ((uint32_t)0x00002C00)


Bool Start_Collect_Flag=0x01;
Bool Send_Data_Host_Flag=0x00;


void  Config_Peripheral(void);
void Init_Parameter(void);
void Current_OutPut(void);
void Send_Host_Regularly(void);


extern u16 distance;

// 1) 此为软件1.2版本 在v1.1基础上改进
// 2）V1.1 为前阶段调试可用版本
//3) 此版本为特殊协议

int main(void)
{       
  
     CHIP_Init();                                   // 初始化芯片 
    
    Select_Clock(HClOCK_32M);                      //初始化时钟
    
 //  SCB->VTOR  =  NVIC_VectTab_FLASH;             //向量表重定向
	   
    Config_Peripheral();                          //初始化外设
	  
    if(MSC_Read_Word(DATABASEADDRESS)!=Fireword_Update_Flag)
    {
        Init_Parameter();                           //初始化参数
    }
    else
    {
     u32 Buffer[8]={0}; 
     MSC_Read_Buffer((uint32_t *)Buffer,8);       //读取参数              
     memcpy((u8 *)&system_param,(u8 *)Buffer+4,sizeof(system_param));      
    }
    Initialize_Uart(system_param.Baudrate);           //初始化串口,波特率为9600
    Uart_SendString("Welcome to use YB_YT_UML",24);

    while (1) {
      Service_Dog();                         //喂狗
       if(Start_Collect_Flag)
            {
              Collect_Voltage();             //电压采集
              Collect_Distance();            //油位采集
              Collect_Temper();              //温度采集
            }
      Deal_Uart_Data();                      //串口数据处理
      Current_OutPut();                      //4-20MA输出
     Send_Host_Regularly();     //             //定时发送数据到主机
    }
  
}



void  Config_Peripheral ()
{
    __disable_irq();                         //关闭系统总中断
	 
    Init_Msc();                             //初始化闪存控制器
	 
    Init_All_Gpio();                        //初始化GPIO

    Init_SysTick();                         //初始化滴答
   
    Init_Timer0();                          //初始化定时器0  

    Init_Timer1();                          //初始化定时器1  

    Init_Timer2();                          //初始化定时器2
     
    Init_SingleADC();                       //初始化ADC

	DMAConfig();                            //配置DMA
	
    RTC_Setup(cmuSelect_CORELEDIV2);        //配置HFCORECLK作为LACLK 

    Init_wdog();                            //初始化看门狗
   
   __enable_irq();                          //打开系统总中断
}

//uint32_t aa[10]={0};
void Init_Parameter()
{
         uint32_t buffer[10]={0}; 
   	 memcpy((u8 *)&system_param,(u8 *)&default_param,sizeof(SYS_PARAM));
	 buffer[0]=Fireword_Update_Flag;
	 buffer[1]=((system_param.Compartment_Num&0xff)<<24)|((system_param.Baudrate&0xff)<<16)|((system_param.TestClass&0xff)<<8)|(system_param.Address&0xff);
	 memcpy(buffer+2,(u8 *)system_param.Device_ID,4);
	 buffer[3]=(((system_param.Density>>8)&0xff)<<24)|((system_param.Density&0xff)<<16)|(((system_param.Voltage>>8)&0xff)<<8)|(system_param.Wave_num&0xff);
	 buffer[4]=(((system_param.Min_level>>8)&0xff)<<24)|((system_param.Min_level&0xff)<<16)|(((system_param.Max_level>>8)&0xff)<<8)|(system_param.Max_level&0xff&0xff);
	 buffer[5]=(((system_param.Off_Speed>>8)&0xff)<<24)|((system_param.Off_Speed&0xff)<<16)|(((system_param.On_Speed>>8)&0xff)<<8)|(system_param.On_Speed&0xff&0xff);
	 buffer[6]=(((system_param.Total_Off_Level>>8)&0xff)<<24)|((system_param.Total_Off_Level&0xff)<<16)|(((system_param.Total_On_Level>>8)&0xff)<<8)|(system_param.Total_On_Level&0xff);

	 buffer[7]=(0<<24)|(0<<16)|((system_param.Enable_Automatic_Flag&0xff)<<8)|(system_param.SendTime&0xff);

	 if(MSC_Write_Buffer(buffer,8)==0)
          {
            Uart_SendString("Write MSC Fail!",15);
          }
       //MSC_Read_Buffer(aa,8);
		 
}


/*********************************************************************************
 * Timer0 产生4MA到20MA输出的PWM  注意：中间549,862都是对应11MA电流的(非线性)
 |\
 |  \
 |    \
 |      \
 |        \
 |          \
 |          \
 |            \
 |              \
 |                \
 |                  \
 |                    \
 |                      \
 |                        \
 |________ ___________________________
 *********************************************************************************/

// 4-20mA输出
//4-20MA===>160mm-2000mm
//{4,5,6,7,8,9,10,11,11,12,13,14,15,16,17,18,19,20};
//{1600,1516,1426,1334,1233,1128,1010,862,549,511,472,430,387,340,280,194,102,20};
//{1600,1445,1350,1250,1145,1025,882,549,511,472,430,387,338,275,186,94,0};
void Current_OutPut(void) // 4-20mA输出
{
    u16 Duty_cul=0;
    u16 level=distance;
    u16 Current[]={1600,1516,1426,1334,1233,1128,1010,862,549,511,472,430,387,340,280,194,102,20};
	u32 MA=0;
	
	if(level>2000) level=2000;
	else if(level<160)  level=160;
    float y=16*(level-160)/(2000-0.35)+4;  //160mm对应4MA,2000对应20MA
    MA = (u32)y;
	if(MA>=4&&MA<=20)
  	{
	  if(MA+1<=11)
	    Duty_cul = (int)(Current[MA-4]-(Current[MA-4]-Current[MA-4+1])*(y-MA));
	  else if(MA!=20)
	    Duty_cul = (int)(Current[MA-3]-(Current[MA-3]-Current[MA-3+1])*(y-MA));
	  else
	  	Duty_cul=Current[17];
	 }
	TIMER_CompareBufSet(TIMER0, 0,Duty_cul);   
}

void Send_Host_Regularly()
{
 if(system_param.Enable_Automatic_Flag)
 	{
	  if(Send_Data_Host_Flag)
	  	{
	  	 Send_Data_Host_Flag=0x00;
	      Get_DevAllData();
	  	}
 	}
}


