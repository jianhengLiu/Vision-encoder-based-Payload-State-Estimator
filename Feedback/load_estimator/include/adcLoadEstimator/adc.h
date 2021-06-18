//
// Created by kevin on 2020/8/7.
//

#ifndef ADCLOADESTIMATOR_ADC_H
#define ADCLOADESTIMATOR_ADC_H

#include "adcLoadEstimator/common_include.h"

#define CONTINUE_GET_ADC_DATA   1
#define ADC_NUMS                10
//计算一个整数bit位为1的个数
uint32_t BitCount(uint32_t data)
{
    uint32_t count=0;
    while(data){
        count++;
        data&=(data-1);
    }
    return count;
}

class ADC{
    DEVICE_INFO DevInfo;
    int DevHandle[10];
    bool state;
    int ret;
    char ADC_Channel = 0x03;//使能ADC_CH0和ADC_CH1
    short Buffer[40960];
    int ADC_Data[10];
    void read(){
        int ret;
        //手动读取ADC数据
        ret = ADC_Read(DevHandle[0], Buffer, ADC_NUMS);
        if (ret != ADC_SUCCESS) {
            printf("Read adc error!\n");
            while(1);
        } else {
            int num_chan = BitCount(ADC_Channel);
            ADC_Data[0] = 0;
            ADC_Data[1] = 0;
            for(int j = 0 ; j < ADC_NUMS ; j ++){
                ADC_Data[0]+= Buffer[ j*2];
                ADC_Data[1]+= Buffer[ 2*j+1];
            }
            ADC_Data[0]/=ADC_NUMS;
            ADC_Data[1]/=ADC_NUMS;

        }
    }
public:
    double position[3];
    ADC(){
        //扫描查找设备
        ret = USB_ScanDevice(DevHandle);
        if(ret <= 0){
            printf("No device connected!\n");
            while(1);
        }
        //打开设备
        state = USB_OpenDevice(DevHandle[0]);
        if(!state){
            printf("Open device error!\n");
            while(1);
        }
        //获取固件信息
        char FuncStr[256]={0};
        state = DEV_GetDeviceInfo(DevHandle[0],&DevInfo,FuncStr);
        if(!state){
            printf("Get device infomation error!\n");
            while(1);
        }
        //初始化ADC
        ret = ADC_Init(DevHandle[0],ADC_Channel,1000000);
        if(ret != ADC_SUCCESS){
            printf("Init adc error!\n");
            while(1);
        }
    }


    double* getLoadPosition(){
        /*
         * @breif:
         *              min 1762
         *  ymin 1779          nax2401
         *              xman 2459
         *
         *              2050 mid
         * */
        read();

        double theta1  = ((double )ADC_Data[0] - 2050 ) * 3.14/4 / 288;
        double theta2  = ((double )ADC_Data[1] - 2050 ) * 3.14/4  / 288;
        std::cout<<" "<<ADC_Data[0]<<" "<< ADC_Data[1]<<" theta:"<<theta1<<" "<< theta2<<std::endl;
        double l = 0.41;
        position[2] = sqrt(l*l/(1+tan(theta1)*tan(theta1)+tan(theta2)*tan(theta2)));
        position[0] =position[2]  * tan(theta2);
        position[1] = position[2] * tan(theta1);

        return (double*)position;
    }

    ~ADC(){
        USB_CloseDevice(DevHandle[0]);
    }

};



#endif //ADCLOADESTIMATOR_ADC_H
