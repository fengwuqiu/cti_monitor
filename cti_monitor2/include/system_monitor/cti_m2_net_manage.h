//
// Created by wq on 22-5-9.
//

#ifndef CTI_MONITOR2_CTI_M2_NET_MANAGE_H
#define CTI_MONITOR2_CTI_M2_NET_MANAGE_H

#include "cti_monitor2_info.h"

#define NET_INFO_PATH "/proc/net/dev"
#define NET_DIFF_TIME 1  //Time difference threshold （s）

using namespace CM2;
namespace CM2 {
    class NetManage {
    public:
        static NetManage *GetInstance();//获取该类的单例指针

    private:
        NetManage();

        ~NetManage();

        static NetManage *m_instance;//该类的单例指针

    private:
        Info *m_info = Info::GetInstance();//存放信息类的单例指针

        bool m_net_init_flag =true;//初始化flag

        NetUseInfo m_net_use_info;//网络信息

        vector<NetUseInfo> m_net_use_info_vector_p;//前一帧的网络使用信息结构体vector
        vector<NetUseInfo> m_net_use_info_vector_c;//当前帧的网络使用信息结构体vector

        time_t net_previous_timeStamp;//记录上一次系统时间戳   时间单位:秒
        time_t net_current_timeStamp;//记录当前系统时间戳 时间单位:秒
        double net_dif_time;//记录时间差 时间单位:秒

        char net_file[64] = {NET_INFO_PATH};//要读取的目标文件名
        int net_off;//记录文件游标当前偏移位置
        int line_num;//记录行数
        FILE *net_stream; //访问虚拟文件系统的文件流
        char net_buffer[256];//行缓冲区
        char *net_line_return;//记录读取每行的时候的返回结果（NULL或者返回 net_buffer）
        char net_tmp_itemName[32];//临时存放文件中的每行的项目名称

        int net_itemReceive;//存放每一个网卡的接受到的字节总数（单位：Byte）
        int net_receive_packets;//存放接受的数据包总的包数
        int net_receive_errs;//存放接受的数据包错误的包数
        int net_receive_drop;//存放接受的数据包丢掉的包数
        int net_itemTransmit;//存放每一个网卡的已发送的字节总数（单位：Byte）
        int net_transmit_packets;//存放发送的数据包总的包数
        int net_transmit_errs;//存放发送的数据包错误的包数
        int net_transmit_drop;//存放发送的数据包丢掉的包数
        double net_receive_mb;//存放接受数据的速率 单位mb/s
        double net_transmit_mb;//存放发送数据的速率 单位mb/s

        int net_current_receive_total;//存放【当前】收到的网络包字节总数（单位：Byte）
        int net_previous_receive_total;//存放【上一次】收到的网络包字节总数（单位：Byte）

        int net_receive_speed;//平均接受网络字节速度

        int net_current_transmit_total; //存放【当前】已发送的网络包字节总数（单位：Byte）
        int net_previous_transmit_total;//存放【上一次】已发送的网络包字节总数（单位：Byte）
        int net_transmit_speed;//平均发送网络字节速度

        float net_averageLoad_speed; //网络负载情况（单位：Bytes/s）
    private:
        void NetInit();//初始化函数

        void NetUpdate(bool init_flag =false);//更新网络信息的函数

        void GetNetUsage();//获取网络使用信息

        double ChangeByteToMb(int byte);//单位转换函数

    public:
        void RunNetManage();//供外部调用的运行网络监测功能的函数
    };
}
#endif //CTI_MONITOR2_CTI_M2_NET_MANAGE_H
