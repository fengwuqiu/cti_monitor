//
// Created by wq on 22-4-28.
//

#ifndef CTI_MONITOR2_CTI_M2_CPU_MANAGE_H
#define CTI_MONITOR2_CTI_M2_CPU_MANAGE_H

#include "cti_monitor2_info.h"

using namespace CM2;
namespace CM2 {
    class CpuManage {
    public:
        static CpuManage *GetInstance();//获取该类的单例指针

    private:
        CpuManage();

        ~CpuManage();

        static CpuManage *m_instance;//该类的单例指针

    private:
        Info *m_info = Info::GetInstance();//存放信息类的单例指针
        LinuxProcessList m_lpl;//存放系统相关信息
        LoadAverage m_la;//存放平均负载信息
        map<int, ProcessCpuInfo> m_pro_cpu_info_map;//存放进程号对应的进程cpu使用信息的map
        double m_jiffy = 0.0;//进行时间单位转换用的参数
        int m_pro_cpu_five[5] = {-1, -1, -1, -1, -1};//存放前五cpu占用率的进程的进程号的数组
        struct statfs m_sf_info;//存放磁盘占用信息
    private:
        void GetCPUCount();//获取cpu核心数

        void ScanCPUTime();//扫描cpu使用数据

        double SetCPUValues(int cpu);//获取cpu使用信息

        bool ReadStatFile(int pid, const char *dirname);//读取stat文件的函数

        unsigned long long AdjustTime(unsigned long long t);//时间单位转换函数

        void InitProCpuFiveArray();//初始化存放前五cpu占用率的进程的进程号的数组

        void ScanMemoryInfo();//扫描内存使用信息

        void GetLoadAverage();//获取平均负载信息

        bool recurseProcTree(const char *dirname, double period, struct timeval tv);//获取进程的cpu使用数据

        void GetCpuTemperature();//获取cpu温度

        void GetStatFsInfo();//获取磁盘使用信息

        ssize_t xread(int fd, void *buf, size_t count);//读取文件用的函数

    public:
        void RunCpuManage();//供外部调用的运行cpu监测功能的函数
    };
}
#endif //CTI_MONITOR2_CTI_M2_CPU_MANAGE_H
