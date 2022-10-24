//
// Created by wq on 22-5-10.
//

#ifndef CTI_MONITOR2_CTI_M2_GPU_MANAGE_H
#define CTI_MONITOR2_CTI_M2_GPU_MANAGE_H
#include "cti_monitor2_info.h"


using namespace CM2;
namespace CM2 {
    class GpuManage {
    public:
        static GpuManage *GetInstance();//获取该类的单例指针

    private:
        GpuManage();

        ~GpuManage();

        static GpuManage *m_instance;//该类的单例指针

    private:
        Info *m_info = Info::GetInstance();//存放信息类的单例指针

        bool m_gpu_init_flag =true;//初始化flag
        nvmlReturn_t m_result;//存放获取的gpu信息
        unsigned int m_gpu_count, m_gpu_index;//gpu数量，gpu索引
        vector<GpuUseInfo> m_gpu_use_info_vector;//存放gpu使用信息结构体vector
    private:
        void GpuInit();//初始化函数

        void UpdateGpuInfo();//更新gpu信息
    public:
        void RunGpuManage();//供外部调用的运行gpu监测功能的函数
    };
}
#endif //CTI_MONITOR2_CTI_M2_GPU_MANAGE_H
