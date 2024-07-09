/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <stdio.h>
#include <unistd.h>

#include <hi_stdlib.h>
#include <hisignalling_protocol.h>
#include <hi_uart.h>
#include <app_demo_uart.h>
#include <iot_uart.h>
#include <hi_gpio.h>
#include <hi_io.h>
#include "iot_gpio_ex.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_watchdog.h"
#include "hi_time.h"

#include "oled_ssd1306.h"

UartDefConfig uartDefConfig = {0};
#define GPIO2 2
#define GPIO8 8
#define IOT_GPIO_IDX_6 6
#define IOT_PWM_PORT_PWM0   0
#define IOT_PWM_BEEP        9
#define GPIO6 6
#define GPIO_11 11
#define GPIO_12 12
#define COUNT 10
#define GPIO_FUNC 0
float distance; 
int t=90;

float velocity;

static void Uart1GpioCOnfig(void)
{
#ifdef ROBOT_BOARD
    IoSetFunc(HI_IO_NAME_GPIO_5, IOT_IO_FUNC_GPIO_5_UART1_RXD);
    IoSetFunc(HI_IO_NAME_GPIO_6, IOT_IO_FUNC_GPIO_6_UART1_TXD);
    /* IOT_BOARD */
#elif defined (EXPANSION_BOARD)
    IoSetFunc(HI_IO_NAME_GPIO_0, IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(HI_IO_NAME_GPIO_1, IOT_IO_FUNC_GPIO_1_UART1_RXD);
#endif
}

static void cargpioint(void)
{
    IoTGpioInit(GPIO2);
    IoSetFunc(GPIO2, 0);
    IoTGpioSetDir(GPIO2, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(GPIO2, IOT_GPIO_VALUE1);

    IoTGpioInit(GPIO8);
    IoSetFunc(GPIO8, 0);
    IoTGpioSetDir(GPIO8, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(GPIO8, IOT_GPIO_VALUE0);



    IoTGpioInit(IOT_PWM_BEEP);
    IoSetFunc(IOT_PWM_BEEP, 5); /* 设置IO5的功能 */
    IoTGpioSetDir(IOT_PWM_BEEP, IOT_GPIO_DIR_OUT);
    IoTPwmInit(IOT_PWM_PORT_PWM0);
    IoTWatchDogDisable();
}

void set_angle(unsigned int duty)
{
    unsigned int time = 20000;
    IoTGpioInit(GPIO6);

    IoTGpioSetDir(GPIO6, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(GPIO6, IOT_GPIO_VALUE1);
    hi_udelay(duty);
    IoTGpioSetOutputVal(GPIO6, IOT_GPIO_VALUE0);
    hi_udelay(time - duty);
}

void engine_turn_left(void)
{
    unsigned int angle = 1000;
    for (int i = 0; i < COUNT; i++) {
        set_angle(angle);
    }
}

void engine_turn_right(void)
{
    unsigned int angle = 2000;
    for (int i = 0; i < COUNT; i++) {
        set_angle(angle);
    }
}

void regress_middle(void)
{
    unsigned int angle = 1500;
    for (int i = 0; i < COUNT; i++) {
        set_angle(angle);
    }
}

float GetDistance  (void)
{
    static unsigned long start_time = 0, time = 0;
    float distance = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    float pi = 0.034;
    int l = 2;
    unsigned int delayTime = 20;
    IoTWatchDogDisable();

    hi_io_set_func(GPIO_12, GPIO_FUNC);
    IoTGpioSetDir(GPIO_12, IOT_GPIO_DIR_IN);

    IoTGpioSetDir(GPIO_11, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(GPIO_11, IOT_GPIO_VALUE1);
    hi_udelay(delayTime);
    IoTGpioSetOutputVal(GPIO_11, IOT_GPIO_VALUE0);

    while (1) {
        IoTGpioGetInputVal(GPIO_12, &value);
        if (value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
    }
    distance = time * pi / l;
    return distance;
}

float Getvelocity  (void)
{
    static unsigned long start_time = 0, time = 0;
    float velocity = 0.0;
    IotGpioValue value = IOT_GPIO_VALUE0;
    unsigned int flag = 0;
    float pi = 0.034;
    int l = 2;
    unsigned int delayTime = 20;
    IoTWatchDogDisable();

    hi_io_set_func(GPIO_12, GPIO_FUNC);
    IoTGpioSetDir(GPIO_12, IOT_GPIO_DIR_IN);

    IoTGpioSetDir(GPIO_11, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(GPIO_11, IOT_GPIO_VALUE1);
    hi_udelay(delayTime);
    IoTGpioSetOutputVal(GPIO_11, IOT_GPIO_VALUE0);

    while (1) {
        IoTGpioGetInputVal(GPIO_12, &value);
        if (value == IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            flag = 1;
        }
        if (value == IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() - start_time;
            start_time = 0;
            break;
        }
    }
    velocity = (distance*2-pi*time)/time;
    return velocity;
}

int SetUartRecvFlag(UartRecvDef def)
{
    if (def == UART_RECV_TRUE) {
        uartDefConfig.g_uartReceiveFlag = HI_TRUE;
    } else {
        uartDefConfig.g_uartReceiveFlag = HI_FALSE;
    }
    
    return uartDefConfig.g_uartReceiveFlag;
}

int GetUartConfig(UartDefType type)
{
    int receive = 0;

    switch (type) {
        case UART_RECEIVE_FLAG:
            receive = uartDefConfig.g_uartReceiveFlag;
            break;
        case UART_RECVIVE_LEN:
            receive = uartDefConfig.g_uartLen;
            break;
        default:
            break;
    }
    return receive;
}

void ResetUartReceiveMsg(void)
{
    (void)memset_s(uartDefConfig.g_receiveUartBuff, sizeof(uartDefConfig.g_receiveUartBuff),
        0x0, sizeof(uartDefConfig.g_receiveUartBuff));
}

unsigned char *GetUartReceiveMsg(void)
{
    return uartDefConfig.g_receiveUartBuff;
}

static hi_void *UartDemoTask(char *param)
{
    OledInit();
    OledFillScreen(0);
    OledShowString(20, 3, "Hello  2", 1);
    hi_u8 uartBuff[UART_BUFF_SIZE] = {0};
    hi_unref_param(param);
    printf("Initialize uart demo successfully, please enter some datas via DEMO_UART_NUM port...\n");
    Uart1GpioCOnfig();
    cargpioint();
    regress_middle();

    distance = GetDistance();
    hi_udelay(5000000);
    IoTPwmStart(IOT_PWM_PORT_PWM0, 90, 4000);
    velocity = Getvelocity();//cm/us
    IoTPwmStop(IOT_PWM_PORT_PWM0);
    hi_udelay(1000000);
    printf("distance is %f\r\n", distance);
    printf("velocity is %f\r\n", velocity);
    
    for (;;) {
        IoTPwmStart(IOT_PWM_PORT_PWM0, t, 4000);
        uartDefConfig.g_uartLen = IoTUartRead(DEMO_UART_NUM, uartBuff, UART_BUFF_SIZE);
        if ((uartDefConfig.g_uartLen > 0) && (uartBuff[0] == 0xaa) && (uartBuff[1] == 0x55)) {
            OledShowString(20, 5, "Hello  3561", 1);
            float s1=GetDistance();
            hi_udelay(500000);
            float s2=GetDistance();
            float dv=velocity+(s1-s2)/500000;
            if(dv>velocity)
            {
                t=t-5;
            }
            if (GetUartConfig(UART_RECEIVE_FLAG) == HI_FALSE) {
                (void)memcpy_s(uartDefConfig.g_receiveUartBuff, uartDefConfig.g_uartLen,
                    uartBuff, uartDefConfig.g_uartLen);
                (void)SetUartRecvFlag(UART_RECV_TRUE);
            }
            if(((uartBuff[2] < 0x40)&&(uartBuff[4] > 0xc0))||((uartBuff[3] < 0x40)&&(uartBuff[5] > 0xc0)))
           {
                t=85;
                OledShowString(20, 1, "turn left,mode2", 1);
                engine_turn_left();
                hi_udelay(1000000);
                engine_turn_right();
                hi_udelay(1000000);
                regress_middle();
            }
        }
        TaskMsleep(20); /* 20:sleep 20ms */
    }
    return HI_NULL;
}

/*
 * This demo simply shows how to read datas from UART2 port and then echo back.
 */
hi_void UartTransmit(hi_void)
{
    hi_u32 ret = 0;

    IotUartAttribute uartAttr = {
        .baudRate = 115200, /* baudRate: 115200 */
        .dataBits = 8, /* dataBits: 8bits */
        .stopBits = 1, /* stop bit */
        .parity = 0,
    };
    /* Initialize uart driver */
    ret = IoTUartInit(DEMO_UART_NUM, &uartAttr);
    if (ret != HI_ERR_SUCCESS) {
        printf("Failed to init uart! Err code = %d\n", ret);
        return;
    }
    /* Create a task to handle uart communication */
    osThreadAttr_t attr = {0};
    attr.stack_size = UART_DEMO_TASK_STAK_SIZE;
    attr.priority = UART_DEMO_TASK_PRIORITY;
    attr.name = (hi_char*)"uart demo";
    if (osThreadNew((osThreadFunc_t)UartDemoTask, NULL, &attr) == NULL) {
        printf("Falied to create uart demo task!\n");
    }
}
SYS_RUN(UartTransmit);