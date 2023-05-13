/*
 * @Date: 2022-10-26 21:35:22
 * @LastEditTime: 2022-10-30 16:48:03
 * @FilePath: \RM-Vision2022\serial\serial_port.hpp
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h> 
#include <fcntl.h>   //文件控制定义
#include <termios.h> //终端控制定义
#include <errno.h>
#include <iostream>
#include <pthread.h>
#include <sys/ioctl.h>
#include "string.h"
#include "iostream"

#include "general.h"

#define TIMEOUT_SEC(buflen, baud) (buflen * 20/ baud + 2) //接收超时
// unsigned char DATA[9];

#define UART_NAME "/dev/ttyUSB0"   // ttyUSB1
#define GYUART_NAME "/dev/ttyUSB0" // ttyUSB1
#define PASSWD "ncu"
 
void uart_send(int16_t yaw_angle, int16_t pitch_angle, u_char aim_mode, int16_t distance);
void uart_receive(unsigned char &uart_char);
void usrt_init(void);
void gyuart_receive(unsigned char &gychar);
void gyusrt_init(void);
class Serial_Port
{
public:
    Serial_Port(const char *__file);
    Serial_Port();

public:
    /** 线程退出标志变量 */
    static bool s_Listen_bExit;
    int serial_fd;

    //* 判断串口是否打开
    bool is_serial_opened;

    unsigned char data[300];
    int datalen = 8;

    int mode;
    int bullet_speed;

private:
    //互斥锁
    pthread_mutex_t Com_lock;

    pthread_t m_hListenThread;

public:
    //打开串口并初始化设置
    int init_serial(const char *__file);
    /**
     *串口发送数据
     *@fd:串口描述符
     *@data:待发送数据
     *@datalen:数据长度
     */

    // void uart_send(int16_t yaw_angle, int16_t pitch_angle, u_char aim_mode, int16_t distance);
    int uart_send(const VisionData& v_data, int datalen);
    /**
     *串口接收数据
     *要求启动后，在pc端发送ascii文件
     */
    bool uart_recv();
    int close_serial();

    /** 开启监听线程
     *
     *  本监听线程完成对串口数据的监听,并将接收到的数据打印到屏幕输出
     *  @return: bool  操作是否成功
     *  @note:   当线程已经处于开启状态时,返回flase
     *  @see:
     */
    bool OpenListenThread();

    /** 关闭监听线程
     *
     *
     *  @return: bool  操作是否成功
     *  @note:   调用本函数后,监听串口的线程将会被关闭
     *  @see:
     */
    bool CloseListenTread();

    /** 读取串口接收缓冲区中一个字节的数据
     *
     *
     *  @param:  char & cRecved 存放读取数据的字符变量
     *  @return: bool  读取是否成功
     *  @note:
     *  @see:
     */
    bool ReadChar(unsigned char &cRecved);

    static void *ListenThread(void *Palm);

    int GetBytesInCOM();
};
