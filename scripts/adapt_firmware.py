#!/usr/bin/env python
"""适配 STM32 固件以支持 UART PID 数据输出和参数接收。"""

import argparse
import os
import re
import sys


UART_H_TEMPLATE = '''\
#ifndef __USART_H__
#define __USART_H__

#include "main.h"

void MX_USART1_UART_Init(void);
void UART_SendString(const char *str);
int UART_ParsePIDUpdate(float *kp, float *ki, float *kd, float *deadband);

extern UART_HandleTypeDef huart1;

#endif /* __USART_H__ */
'''

UART_C_TEMPLATE = '''\
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

UART_HandleTypeDef huart1;

#define RX_BUF_SIZE 64
static uint8_t rx_byte;
static char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_idx = 0;
static volatile uint8_t rx_ready = 0;

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        char c = (char)rx_byte;
        if (c == '\\n' || c == '\\r') { if (rx_idx > 0) { rx_buf[rx_idx] = '\\0'; rx_ready = 1; } }
        else if (rx_idx < RX_BUF_SIZE - 1) { rx_buf[rx_idx++] = c; }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

int UART_ParsePIDUpdate(float *kp, float *ki, float *kd, float *deadband)
{
    if (!rx_ready) return 0;
    rx_ready = 0;
    if (rx_buf[0] != 'P' || rx_buf[1] != ':') return 0;
    char *p = &rx_buf[2], *end;
    *kp = strtof(p, &end); if (*end != ',') return 0; p = end + 1;
    *ki = strtof(p, &end); if (*end != ',') return 0; p = end + 1;
    *kd = strtof(p, &end); if (*end != ',') return 0; p = end + 1;
    *deadband = strtof(p, &end);
    return 1;
}

void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}
'''

PID_SEND_SNIPPET = '''
    /* UART 输出 PID 数据（格式: sp,meas,out） */
    {
        static uint8_t uart_div = 0;
        if (++uart_div >= 5)
        {
            uart_div = 0;
            char buf[48];
            int n = snprintf(buf, sizeof(buf), "%.1f,%.2f,%.1f\\n",
                             rollSetpoint, roll, rollOutput);
            HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 50);
        }
    }

    /* UART 接收：实时更新 PID 参数 */
    {
        float new_kp, new_ki, new_kd, new_db;
        if (UART_ParsePIDUpdate(&new_kp, &new_ki, &new_kd, &new_db))
        {
            pidPitch.kp = new_kp; pidPitch.ki = new_ki;
            pidPitch.kd = new_kd; pidPitch.deadband = new_db;
            pidRoll.kp = new_kp; pidRoll.ki = new_ki;
            pidRoll.kd = new_kd; pidRoll.deadband = new_db;
            char ack[48];
            int n = snprintf(ack, sizeof(ack), "OK:%.2f,%.2f,%.3f,%.1f\\n",
                             new_kp, new_ki, new_kd, new_db);
            HAL_UART_Transmit(&huart1, (uint8_t *)ack, n, 50);
        }
    }
'''


UART_PIN_MAP = {
    'USART1': {'tx': 'PA9',  'rx': 'PA10', 'clk': 'USART1', 'gpio': 'GPIOA', 'irq': 'USART1_IRQn'},
    'USART2': {'tx': 'PA2',  'rx': 'PA3',  'clk': 'USART2', 'gpio': 'GPIOA', 'irq': 'USART2_IRQn'},
    'USART3': {'tx': 'PB10', 'rx': 'PB11', 'clk': 'USART3', 'gpio': 'GPIOB', 'irq': 'USART3_IRQn'},
    'UART4':  {'tx': 'PC10', 'rx': 'PC11', 'clk': 'UART4',  'gpio': 'GPIOC', 'irq': 'UART4_IRQn'},
    'UART5':  {'tx': 'PC12', 'rx': 'PD2',  'clk': 'UART5',  'gpio': 'GPIOC', 'irq': 'UART5_IRQn'},
}


def adapt_firmware(project_root, uart='USART1'):
    """适配固件。"""
    src_dir = os.path.join(project_root, 'Src')
    inc_dir = os.path.join(project_root, 'Inc')

    # 检测项目中使用的 UART
    detected_uart = None
    for root, dirs, files in os.walk(project_root):
        dirs[:] = [d for d in dirs if d not in ('Drivers', '.git', 'MDK-ARM')]
        for f in files:
            if f.endswith(('.c', '.h')):
                try:
                    content = open(os.path.join(root, f), 'r', encoding='utf-8', errors='ignore').read()
                    for u in ['USART1', 'USART2', 'USART3', 'UART4', 'UART5']:
                        if f'HAL_UART_Init' in content and u in content:
                            detected_uart = u
                            break
                except:
                    pass
        if detected_uart:
            break

    if detected_uart and detected_uart != uart:
        print(f'检测到项目使用 {detected_uart}，将使用该串口')
        uart = detected_uart

    pins = UART_PIN_MAP.get(uart, UART_PIN_MAP['USART1'])
    print(f'使用 {uart}: TX={pins["tx"]}, RX={pins["rx"]}')

    # 创建 usart.h
    uart_h = os.path.join(inc_dir, 'usart.h')
    if not os.path.exists(uart_h):
        with open(uart_h, 'w') as f:
            f.write(UART_H_TEMPLATE)
        print(f'创建: {uart_h}')
    else:
        print(f'已存在: {uart_h}')

    # 创建 usart.c
    uart_c = os.path.join(src_dir, 'usart.c')
    if not os.path.exists(uart_c):
        with open(uart_c, 'w') as f:
            f.write(UART_C_TEMPLATE)
        print(f'创建: {uart_c}')
    else:
        print(f'已存在: {uart_c}')

    # 检查 HAL 配置
    hal_conf = os.path.join(inc_dir, 'stm32f1xx_hal_conf.h')
    if os.path.exists(hal_conf):
        content = open(hal_conf, 'r').read()
        if '/*#define HAL_UART_MODULE_ENABLED' in content:
            content = content.replace('/*#define HAL_UART_MODULE_ENABLED', '#define HAL_UART_MODULE_ENABLED')
            with open(hal_conf, 'w') as f:
                f.write(content)
            print(f'已启用: HAL_UART_MODULE_ENABLED')
        else:
            print(f'HAL_UART_MODULE_ENABLED 已启用')

    print(f'\n=== 固件适配完成 ===')
    print(f'还需要手动完成:')
    print(f'1. 在 Keil 项目中添加 usart.c 和 stm32f1xx_hal_uart.c')
    print(f'2. 在 System_Init() 中调用 MX_USART1_UART_Init()')
    print(f'3. 在 Process_IMU() 末尾添加 UART 输出和接收代码')
    print(f'4. 接线: PA9→CH340 RX, PA10→CH340 TX, GND→GND')
    print(f'\nUART 输出/接收代码片段:')
    print(PID_SEND_SNIPPET)


def main():
    parser = argparse.ArgumentParser(description='适配 STM32 固件 UART')
    parser.add_argument('--project', required=True, help='项目根目录')
    parser.add_argument('--uart', default='USART1', help='USART 实例')
    args = parser.parse_args()

    adapt_firmware(args.project, args.uart)
    return 0


if __name__ == '__main__':
    sys.exit(main())
