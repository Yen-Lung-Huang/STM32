1.����ت��G
�@�s�ذ��STM32F4�T��wV1.3.0���u�{�ҪO

2.����{�H�G
�@�U���i�}�o�O����,����O�{�{

3.�`�N�ƶ��G
�@1)�s�ؤu�{���ɭ�,�нT�O��ܥؼС�C / c++�ﶵ�d���������w�q�w�q��J�ئr�Ŧꬰ:STM32F40_41xxx USE_STDPERIPH_DRIVER

4 .��U�����Ѩ�B�J15���ɭԪ�c���X�p�U:





#include "stm32f4xx.h"


//ALIENTEK������STM32F407�}�o�O����0
//STM32F4�u�{�ҪO-�w��ƪ���
//�޳N���:www.openedv.com
//�^�_���Q:http://eboard.taobao.com
//�s�{���P�l�q�l��ަ������q
//�@��:���I��l@ALIENTEK
  
void Delay(__IO uint32_t nCount);

void Delay(__IO uint32_t nCount)
{
  while(nCount--){}
}

int main(void)
{

  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  while(1){
		GPIO_SetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
		Delay(0x7FFFFF);
		GPIO_ResetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_10);
		Delay(0x7FFFFF);
	
	}
}
