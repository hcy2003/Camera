
//串口的设置主要是设置struct termios结构体的各成员
// struct termios {
//         tcflag_t c_iflag;               /* input mode flags */
//         tcflag_t c_oflag;               /* output mode flags */
//         tcflag_t c_cflag;               /* control mode flags */
//         tcflag_t c_lflag;               /* local mode flags */
//         cc_t c_line;                    /* line discipline */
//         cc_t c_cc[NCCS];                /* control characters */
// };
#include "serial_port.hpp"

bool Serial_Port::s_Listen_bExit = false;
bool isPortOpened;
Serial_Port Com, gyCom;
int16_t pitch_now;
int16_t yaw_now;
const unsigned char SLEEP_TIME_INTERVAL = 5;

Serial_Port::Serial_Port(const char *__file)
{
    init_serial(__file);
}

Serial_Port::Serial_Port()
{
    is_serial_opened = false;
}

//打开串口并初始化设置
int Serial_Port::init_serial(const char *__file)
{
    serial_fd = open(__file, O_RDWR | O_NOCTTY | O_NDELAY); //返回句柄
    if (serial_fd < 0)
    {
        printf("open Com: %s fail !\n", __file);
        std::cout << serial_fd;
        return -1;
    }

    //串口主要设置结构体termios <termios.h>
    struct termios options;

    /**1. tcgetattr函数用于获取与终端相关的参数。
     *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
     */
    tcgetattr(serial_fd, &options);
    /**2. 修改所获得的参数*/
    options.c_cflag |= (CLOCAL | CREAD); //设置控制模式状态，本地连接，接收使能
    options.c_cflag &= ~CSIZE;           //字符长度，设置数据位之前一定要屏掉这个位
    options.c_cflag &= ~CRTSCTS;         //无硬件流控
    options.c_cflag |= CS8;              // 8位数据长度
    options.c_cflag &= ~CSTOPB;          // 1位停止位
    options.c_iflag |= IGNPAR;           //无奇偶检验位
    options.c_oflag = 0;                 //输出模式
    options.c_lflag = 0;                 //不激活终端模式
    cfsetospeed(&options, B115200);      //设置波特率
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(serial_fd, TCIFLUSH); //溢出数据可以接收，但不读
    tcsetattr(serial_fd, TCSANOW, &options);

    is_serial_opened=true;
    cout<<"open serial success!!!!!!!\n";
    return 0;
}

/**
 *串口发送数据
 *@fd:串口描述符
 *@data:待发送数据
 *@datalen:数据长度
 */
int Serial_Port::uart_send(const VisionData& vis_data, int datalen)
{
    // cout << "********************************************" << endl;
    // cout << "yaw_angle = " << vis_data.yaw_angle << "  pitch_angle = " << vis_data.pitch_angle << "  isFire = " <<  vis_data.isFire << "  dis = " << vis_data.dis << endl;
    // cout << "********************************************" << endl;
    unsigned char data[17];
    data[0] = 0xFF;                                                             //帧头
    data[1] = (int16_t)(vis_data.pitch_angle) >> 8;                                        
    data[2] = (int16_t)(vis_data.pitch_angle);    
    data[3] = (int16_t)(vis_data.roll_angle) >> 8;                         
    data[4] = (int16_t)(vis_data.roll_angle);                                                                  
    data[5] = (int16_t)(vis_data.yaw_angle) >> 8;                          
    data[6] = (int16_t)(vis_data.yaw_angle); 
    data[7] = (int16_t)vis_data.isDetected;                         
    data[8] = (int16_t)(vis_data.dis)>> 8;                                 //目标距离高八位
    data[9] = (int16_t)(vis_data.dis);
    data[10] = (int16_t)(vis_data.x)>> 8;                                 
    data[11] = (int16_t)(vis_data.x);  
    data[12] = (int16_t)(vis_data.y)>> 8;                                      
    data[13] = (int16_t)(vis_data.y);  
    data[14] = (int16_t)(vis_data.z)>> 8;                                      
    data[15] = (int16_t)(vis_data.z);                                              
    data[16] = 0xFE;                                                              //帧尾

    int len = 0;
    
    //pthread_mutex_lock(&Com_lock);         //锁串口
    len = write(serial_fd, data, datalen); //实际写入的长度
    //pthread_mutex_unlock(&Com_lock);       // unlock Com
    if (len == datalen)
    {
        return len;
    }
    else
    {
        //pthread_mutex_lock(&Com_lock);   //锁串口
        tcflush(serial_fd, TCOFLUSH);    // TCOFLUSH刷新写入的数据但不传送
        //pthread_mutex_unlock(&Com_lock); // unlock Com
        return -1;
    }

    return 0;
}

bool Serial_Port::ReadChar(unsigned char &data)
{
    int readlen, fs_sel;
    fd_set fs_read;
    struct timeval tv_timeout;
    FD_ZERO(&fs_read);
   // pthread_mutex_lock(&Com_lock); // unlock Com
    FD_SET(serial_fd, &fs_read);
    tv_timeout.tv_sec = TIMEOUT_SEC(1, 115200);
    tv_timeout.tv_usec = 0;
    fs_sel = select(serial_fd + 1, &fs_read, NULL, NULL, &tv_timeout);
    if (fs_sel)
    {

        readlen = read(serial_fd, (void *)&data, 1);
      //  pthread_mutex_unlock(&Com_lock); // unlock Com
        return true;
    }
    else
    {
      //  pthread_mutex_unlock(&Com_lock); // unlock Com
        return false;
    }
}

bool Serial_Port::OpenListenThread()
{

    /** 检测线程是否已经开启了 */
    if (m_hListenThread != 0)
    {
        /** 线程已经开启 */
        return false;
    }
    s_Listen_bExit = false;
    /** 线程ID */
    /** 开启串口数据监听线程 */
    pthread_create(&m_hListenThread, NULL, ListenThread, (void *)this);
    //此处的this 代表的就是SerialPort这个类，这样的话，我们就可以在ListenThread中使用所有的函数和变量了
    //    if (!m_hListenThread)
    //    {
    //        return false;
    //    }
    //    /** 设置线程的优先级,高于普通线程 */
    return true;
}
int Serial_Port::GetBytesInCOM()
{
    int BytesInQue;
    pthread_mutex_lock(&Com_lock); // unlock Com
    ioctl(serial_fd, FIONREAD, &BytesInQue);
    pthread_mutex_unlock(&Com_lock); // unlock Com
    return BytesInQue;
}
void *Serial_Port::ListenThread(void *Palm)
{
    /** 得到本类的指针 */
    Serial_Port *pSerialPort = reinterpret_cast<Serial_Port *>(Palm);
    // 线程循环,轮询方式读取串口数据
    while (!pSerialPort->s_Listen_bExit)
    {
        int BytesInQue = pSerialPort->GetBytesInCOM();
        /** 如果串口输入缓冲区中无数据,则休息一会再查询 */
        if (BytesInQue == 0)
        {
            usleep(SLEEP_TIME_INTERVAL * 1000);
            continue;
        }

        /** 读取输入缓冲区中的数据并输出显示 */
        u_char cRecved = 0x00;
        u_char rec_meg[9];
        int idx = 0;
        for (int i = 0; i < BytesInQue; i++)
        {
            cRecved = 0x00;
            if (pSerialPort->ReadChar(cRecved) == true)
            {
                if (idx < 9)
                    rec_meg[idx] = cRecved;
                idx++;
                // if((byte)cRecved==0xAA)
                //                 printf("%02x ", (unsigned char)cRecved);
                // std::cout << cRecved;
                // continue;
            }
        }

        //        std::cout<<std::endl;
        // printf("\nmes: %02x %02x %02x \n", rec_meg[0], rec_meg[1], rec_meg[2]);
        if (rec_meg[0] == 0xFF && rec_meg[8] == 0xFE)
        {
            //            pitch_now = rec_meg[3]<<8|rec_meg[4];
            //            yaw_now = rec_meg[1]<<8|rec_meg[2];

            //            printf("%d \n", pich_now);
            //            char sss[] = "OK";
            //            pSerialPort->uart_send((unsigned char*)sss, sizeof(sss));
            //
        }
        tcflush(pSerialPort->serial_fd, TCIFLUSH);
    }

    return 0;
}

/**
 *串口接收数据
 */

bool Serial_Port::uart_recv()
{
    int readlen, fs_sel;
    fd_set fs_read;
    struct timeval tv_timeout;

    FD_ZERO(&fs_read);
    FD_SET(serial_fd, &fs_read);
    tv_timeout.tv_sec = TIMEOUT_SEC(datalen, 115200);
    tv_timeout.tv_usec = 0;

    fs_sel=select(serial_fd+1,&fs_read,NULL,NULL,&tv_timeout);
    if (fs_sel)
    {
        readlen=read(serial_fd,data,datalen);

        if(data[0]==0xFF &&data[2]==0xFE)
        {
            mode=(int8_t)data[1];

        }
    
    
    // readlen = read(serial_fd, data, datalen);
    // cout<<"/*********/"<<data[0]<<"  *  "<<data[1]<<"  *  "<<data[2]<<data[3];
    // fs_sel = select(serial_fd + 1, &fs_read, NULL, NULL, &tv_timeout);
    // if(data[0]== 0xFF && data[2] == 0xFE){
    //     cout<<"/********/"<<endl;
    //     choose = (int8_t) data[1];
        tcflush(serial_fd, TCIFLUSH);
        return true;
   }
    else
    {
        tcflush(serial_fd, TCIFLUSH);
        return false;
    }

    //return true;

    //     ret = select ( serial_fd+1, &fs_read, NULL, NULL, &tv_timeout );
    //     std::cout<<ret;
    //     printf ( "ret = %d\n", ret );
    //如果返回0，代表在描述符状态改变前已超过timeout时间,错误返回-1

    //     if ( FD_ISSET ( fd, &fs_read ) )
    //     {
    //    len = read (serial_fd, data, datalen );
    //    printf ( "len = %d\n", len );
    //    std::cout<<len;
    //    return len;
    //     }
    //     else
    //     {
    //     perror ( "select" );
    //     return -1;
    //     }

    // return 0;
}

int Serial_Port::close_serial()
{
    close(serial_fd);
    return 0;
}

void usrt_init(void)
{
    //串口通讯相关程序
    printf("\n*******Begin setting Com*********\n ");
    char const *device0 = UART_NAME;
    //串口无权限时自动开启所有权限
    if (access(UART_NAME, W_OK) != 0) //成功执行时，返回0。失败返回-1
    {
        char commed1[100], dest[30];

        strcpy(commed1, "echo ");
        strcpy(dest, " | sudo -S chmod 777 ");
        strcat(commed1, PASSWD);
        strcat(commed1, dest);
        strcat(commed1, UART_NAME);
        std::cout << commed1 << std::endl;
        system(commed1);
    }

    //开机时系统会占用串口向外发送开机信息，所以此处不断轮询，直到系统释放所有权为止
    while (1)
    {
        if (Com.init_serial(device0) != -1)
        {
            isPortOpened = true;
            printf("Open Com for UART: %s success!\n", device0);
            break;
        }
    }

    if (!isPortOpened)
        std::cout << "initPort fail !" << std::endl;
    //     else//串口开启成功
    //     {
    //                 if (!Com.OpenListenThread())//打开串口监听线程。
    //                     std::cout << "Open Com Thread fail !" << std::endl;
    //                 else
    //                     std::cout << "Open Com Thread success !" << std::endl;
    //     }
    printf("*******END of setting Com*********\n");
}

void gyusrt_init(void)
{
    //串口通讯相关程序
    printf("\n*******Begin setting Com*********\n ");
    char const *device1 = GYUART_NAME;

    //串口无权限时自动开启所有权限
    if (access(GYUART_NAME, R_OK) != 0) //成功执行时，返回0。失败返回-1
    {
        char commed1[100], dest[30];

        strcpy(commed1, "echo ");
        strcpy(dest, " | sudo -S chmod 777 ");
        strcat(commed1, PASSWD);
        strcat(commed1, dest);
        strcat(commed1, GYUART_NAME);
        std::cout << commed1 << std::endl;
        system(commed1);
    }
    std::cout << access(GYUART_NAME, W_OK) << std::endl;
    // system("pause");
    //开机时系统会占用串口向外发送开机信息，所以此处不断轮询，直到系统释放所有权为止
    while (1)
    {
        if (gyCom.init_serial(device1) != -1)
        {
            isPortOpened = true;
            printf("Open Com for GY: %s success!\n", device1);
            break;
        }
    }

    // if (isPortOpened)
    //  {
    //                 if (!Com.OpenListenThread())//打开串口监听线程。
    //                     std::cout << "Open Com Thread fail !" << std::endl;
    //                 else
    //                     std::cout << "Open Com Thread success !" << std::endl;
    //     }
    //     else//串口开启失败
    //     std::cout << "initPort fail !" << std::endl;

    printf("*******END of setting Com*********\n");
}

void gyuart_receive(unsigned char &gychar)
{
    gyCom.ReadChar(gychar);
}



// int main(){
//     cout << "测试中" << endl;
//     VisionData visiondata;
//     visiondata.pitch_angle = 20;
//     visiondata.yaw_angle = 30;
//     visiondata.dis = 40;
//     visiondata.isFire = 1;
//     Serial_Port serial_port;
//     serialWatcher(serial_port);
//     int test = serial_port.uart_send(visiondata,9);
// }
