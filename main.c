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

// 1) ��Ϊ���1.2�汾 ��v1.1�����ϸĽ�
// 2��V1.1 Ϊǰ�׶ε��Կ��ð汾
//3) �˰汾Ϊ����Э��

int main(void)
{       
  
     CHIP_Init();                                   // ��ʼ��оƬ 
    
    Select_Clock(HClOCK_32M);                      //��ʼ��ʱ��
    
 //  SCB->VTOR  =  NVIC_VectTab_FLASH;             //�������ض���
	   
    Config_Peripheral();                          //��ʼ������
	  
    if(MSC_Read_Word(DATABASEADDRESS)!=Fireword_Update_Flag)
    {
        Init_Parameter();                           //��ʼ������
    }
    else
    {
     u32 Buffer[8]={0}; 
     MSC_Read_Buffer((uint32_t *)Buffer,8);       //��ȡ����              
     memcpy((u8 *)&system_param,(u8 *)Buffer+4,sizeof(system_param));      
    }
    Initialize_Uart(system_param.Baudrate);           //��ʼ������,������Ϊ9600
    Uart_SendString("Welcome to use YB_YT_UML",24);

    while (1) {
      Service_Dog();                         //ι��
       if(Start_Collect_Flag)
            {
              Collect_Voltage();             //��ѹ�ɼ�
              Collect_Distance();            //��λ�ɼ�
              Collect_Temper();              //�¶Ȳɼ�
            }
      Deal_Uart_Data();                      //�������ݴ���
      Current_OutPut();                      //4-20MA���
     Send_Host_Regularly();     //             //��ʱ�������ݵ�����
    }
  
}



void  Config_Peripheral ()
{
    __disable_irq();                         //�ر�ϵͳ���ж�
	 
    Init_Msc();                             //��ʼ�����������
	 
    Init_All_Gpio();                        //��ʼ��GPIO

    Init_SysTick();                         //��ʼ���δ�
   
    Init_Timer0();                          //��ʼ����ʱ��0  

    Init_Timer1();                          //��ʼ����ʱ��1  

    Init_Timer2();                          //��ʼ����ʱ��2
     
    Init_SingleADC();                       //��ʼ��ADC

	DMAConfig();                            //����DMA
	
    RTC_Setup(cmuSelect_CORELEDIV2);        //����HFCORECLK��ΪLACLK 

    Init_wdog();                            //��ʼ�����Ź�
   
   __enable_irq();                          //��ϵͳ���ж�
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
 * Timer0 ����4MA��20MA�����PWM  ע�⣺�м�549,862���Ƕ�Ӧ11MA������(������)
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

// 4-20mA���
//4-20MA===>160mm-2000mm
//{4,5,6,7,8,9,10,11,11,12,13,14,15,16,17,18,19,20};
//{1600,1516,1426,1334,1233,1128,1010,862,549,511,472,430,387,340,280,194,102,20};
//{1600,1445,1350,1250,1145,1025,882,549,511,472,430,387,338,275,186,94,0};
void Current_OutPut(void) // 4-20mA���
{
    u16 Duty_cul=0;
    u16 level=distance;
    u16 Current[]={1600,1516,1426,1334,1233,1128,1010,862,549,511,472,430,387,340,280,194,102,20};
	u32 MA=0;
	
	if(level>2000) level=2000;
	else if(level<160)  level=160;
    float y=16*(level-160)/(2000-0.35)+4;  //160mm��Ӧ4MA,2000��Ӧ20MA
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


