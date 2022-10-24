//
// Created by wq on 22-5-12.
//

#ifndef CTI_MONITOR2_CTI_M2_IP_MANAGE_H
#define CTI_MONITOR2_CTI_M2_IP_MANAGE_H

#include "cti_monitor2_info.h"

#define PACKET_SEND_MAX_NUM 64


using namespace CM2;
namespace CM2 {
    class IpManage {
    public:
        static IpManage *GetInstance();//获取该类的单例指针

    private:
        IpManage();

        ~IpManage();

        static IpManage *m_instance;//该类的单例指针

    private:
        Info *m_info = Info::GetInstance();//存放信息类的单例指针

        bool m_ip_init_flag = true;//初始化flag

        vector<IpInfo> m_ip_info_vector;//存放ip信息结构体vector

    private:
        void IpInit();//初始化函数

        void UpdateIpInfo();//更新ip信息函数

        bool PingIp(string ip_address);//ping ip函数

    public:
        void RunIpManage();//供外部调用的运行ip连接状态监测功能的函数
    };
}
#endif //CTI_MONITOR2_CTI_M2_IP_MANAGE_H
