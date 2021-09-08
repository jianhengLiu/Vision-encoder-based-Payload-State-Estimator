/**
  ******************************************************************************
  * @file    usb2adc.h
  * $Author: wdluo $
  * $Revision: 447 $
  * $Date:: 2013-06-29 18:24:57 +0800 #$
  * @brief   usb2adc��غ������������Ͷ���.
  ******************************************************************************
  * @attention
  *
  *<center><a href="http:\\www.toomoss.com">http://www.toomoss.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */

#ifndef __USB2ADC_H_
#define __USB2ADC_H_
#define OS_UNIX
#include <stdint.h>
#ifndef OS_UNIX

#else
#include <unistd.h>
#ifndef WINAPI
#define WINAPI
#endif
#endif


//���庯�����ش������
#define ADC_SUCCESS             (0)   //����ִ�гɹ�
#define ADC_ERR_NOT_SUPPORT     (-1)  //��������֧�ָú���
#define ADC_ERR_USB_WRITE_FAIL  (-2)  //USBд����ʧ��
#define ADC_ERR_USB_READ_FAIL   (-3)  //USB������ʧ��
#define ADC_ERR_CMD_FAIL        (-4)  //����ִ��ʧ��
#define ADC_ERR_CH_NO_INIT      (-5)  //��ͨ��δ��ʼ��

//���������ɼ�����ģʽ�µĻص�����
typedef  int (WINAPI *PADC_GET_DATA_HANDLE)(int DevHandle,unsigned short *pData,int DataNum);//�������ݻص�����

#ifdef __cplusplus
extern "C"
{
#endif
/**
  * @brief  ��ʼ������ADC
  * @param  DevHandle �豸������
  * @param  Channel ��Ҫ���õ�ADCͨ����ÿ��bitΪ��Ӧһ��ͨ����Ϊ1ʱ����Ҫ���ø�ͨ�������λ����ͨ��0
  * @param  SampleRateHz ADC�����ʣ���ͨ�����2.5MHz����λΪHz
  * @retval ����ִ��״̬��С��0����ִ�г���
  */
int WINAPI ADC_Init(int DevHandle,char Channel,int SampleRateHz);
/**
  * @brief  ����������ADCת��ֵ
  * @param  DevHandle �豸������
  * @param  pData ���ݴ洢�������׵�ַ
  * @param  DataNum ��ȡת������������ÿ������֮���ʱ����Ϊ��ʼ������ʱ��Ĳ���Ƶ�ʾ���
  * @retval ����ִ��״̬��С��0����ִ�г���
  */
int WINAPI ADC_Read(int DevHandle,short *pData,int DataNum);
/**
  * @brief  ����ADC����ת��ģʽ
  * @param  DevHandle �豸������
  * @param  Channel ��Ҫ���õ�ADCͨ����ÿ��bitΪ��Ӧһ��ͨ����Ϊ1ʱ����Ҫ���ø�ͨ�������λ����ͨ��0
  * @param  SampleRateHz ADC�����ʣ���ͨ�����2.5MHz����λΪHz
  * @param  FrameSize ��������ʱÿ�δ������ݸ���SampleRateHz*1000/FrameSize��ô��ڻ��ߵ���20
  * @retval ����ִ��״̬��С��0����ִ�г���
  */
int WINAPI ADC_StartContinueRead(int DevHandle,char Channel,int SampleRateHz,int FrameSize,PADC_GET_DATA_HANDLE pGetDataHandle);
/**
  * @brief  ֹͣADC����ת��ģʽ
  * @param  DevHandle �豸������
  * @retval ����ִ��״̬��С��0����ִ�г���
  */
int WINAPI ADC_StopContinueRead(int DevHandle);
/**
  * @brief  ��ȡADC����ת��ģʽ�´洢�����ݻ������е�����ֵ
  * @param  DevHandle �豸������
  * @param  pDataBuffer ���ݴ洢�������׵�ַ
  * @param  BufferSize ���ݴ洢���������£�ע�ⵥλΪ�����ͣ������ֽڣ���������СΪ10240�������ȡ��������
  * @retval �ɹ���ȡ��������������λΪ�����ͣ�
  */
int WINAPI ADC_GetData(int DevHandle,unsigned short *pDataBuffer,int BufferSize);

#ifdef __cplusplus
}
#endif

#endif

