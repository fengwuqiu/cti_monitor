//
// Created by wq on 22-4-28.
//

#ifndef CTI_MONITOR2_CTI_MONITOR2_INFO_H
#define CTI_MONITOR2_CTI_MONITOR2_INFO_H

#include "cti_monitor2_define.h"
#include "cti_monitor2_node.h"

namespace CM2 {
    class Info {
    public:
        static Info *GetInstance();//获取该类的单例指针

    private:
        static Info *m_instance;//该类的单例指针

        Info();

        ~Info() { delete m_instance; };
    private:
        shared_ptr<rclcpp::Node> m_cm2_node;//保存节点指针
        ConfigParameter m_config_parameter;//保存配置参数的结构体
        string m_version_num;//保存版本号

        vector<ProcessCpuInfo> m_pro_cpu_info_vector;//保存进程cpu占用信息结构体vector
        LoadAverage m_la_info;//保存平均负载信息结构体
        MemInfo m_mem_info;//保存内存使用信息结构体
        vector<CpuUseInfo> m_cpu_use_info_vector;//保存cpu使用信息结构体vector
        float m_cpu_temp;//保存cpu温度数据
        int m_sf_use_info;//保存磁盘占用率数据
        vector<NetUseInfo> m_net_use_info_vector;//保存网络使用信息结构体vector
        vector<GpuUseInfo> m_gpu_use_info_vector;//保存gpu使用信息结构体vector
        vector<IpInfo> m_ip_info_vector;//保存ip信息结构体vector
        vector<TopicStateInfo> m_topic_state_info_vector;//保存话题状态信息结构体vector
        vector<NodeStateInfo> m_node_state_info_vector;//保存节点状态信息结构体vector

        mutex m_node_mutex;//节点指针用的线程锁
        mutex m_version_num_mutex;//版本号用的线程锁
        mutex m_ip_info_mutex;//ip信息结构体vector用的线程锁
        mutex m_topic_state_info_mutex;//话题状态信息结构体vector用的线程锁
        mutex m_node_state_info_mutex;//节点状态信息结构体vector用的线程锁

    private:

    public:
        /// 保存节点指针的函数
        /// \param node_handle 节点指针
        inline void SaveNodeHandle(shared_ptr<rclcpp::Node> &node_handle) {
            lock_guard<mutex> m_node_lock(m_node_mutex);
            m_cm2_node = node_handle;
        };

        /// 获取节点指针的函数
        /// \return 节点指针
        inline shared_ptr<rclcpp::Node> GetNodeHandle() {
            lock_guard<mutex> m_node_lock(m_node_mutex);
            return m_cm2_node;
        };

        /// 保存版本号的函数
        /// \param version_num 版本号
        inline void SaveVersionNum(string &version_num) {
            lock_guard<mutex> m_version_num_lock(m_version_num_mutex);
            m_version_num = version_num;
        };

        /// 获取版本号的函数
        /// \return 版本号
        inline string GetVersionNum() {
            lock_guard<mutex> m_version_num_lock(m_version_num_mutex);
            return m_version_num;
        };

        ///进行初始化配置参数的函数
        void CreateConfigParameter();

        /// 获取配置参数的函数
        /// \return 配置参数数据的结构体
        inline ConfigParameter GetConfigParameter() { return m_config_parameter; };

        /// 保存进程cpu占用信息结构体vector的函数
        /// \param pro_cpu_info_vector 进程cpu占用信息结构体vector
        inline void SaveProcessCpuInfo(
                vector<ProcessCpuInfo> &pro_cpu_info_vector) { m_pro_cpu_info_vector = pro_cpu_info_vector; };

        /// 获取进程cpu占用信息结构体vector的函数
        /// \return
        inline vector<ProcessCpuInfo> GetProcessCpuInfo() { return m_pro_cpu_info_vector; };

        /// 保存平均负载信息结构体的函数
        /// \param la_info 平均负载信息结构体
        inline void SaveLoadAverageInfo(LoadAverage la_info) { m_la_info = la_info; };

        /// 获取平均负载信息结构体的函数
        /// \return 平均负载信息结构体
        inline LoadAverage GetLoadAverageInfo() { return m_la_info; };

        /// 保存内存信息结构体的函数
        /// \param mem_info 内存信息结构体
        inline void SaveMemInfo(MemInfo mem_info) { m_mem_info = mem_info; };

        /// 获取内存信息结构体的函数
        /// \return 内存信息结构体
        inline MemInfo GetMemInfo() { return m_mem_info; };

        /// 保存cpu使用信息结构体vector的函数
        /// \param cpu_use_info_vector cpu使用信息结构体vector
        inline void
        SaveCpuUseInfo(vector<CpuUseInfo> &cpu_use_info_vector) { m_cpu_use_info_vector = cpu_use_info_vector; };

        /// 获取cpu使用信息结构体vector的函数
        /// \return cpu使用信息结构体vector
        inline vector<CpuUseInfo> GetCpuUseInfo() { return m_cpu_use_info_vector; };

        /// 保存cpu温度数据的函数
        /// \param cpu_temp cpu温度
        inline void SaveCpuTemp(float cpu_temp) { m_cpu_temp = cpu_temp; };

        /// 获取cpu温度数据的函数
        /// \return cpu温度
        inline float GetCpuTemp() { return m_cpu_temp; };

        /// 保存磁盘占用率数据的函数
        /// \param sf_use_info 磁盘占用率
        inline void SaveSfUseInfo(int sf_use_info) { m_sf_use_info = sf_use_info; };

        /// 获取磁盘占用率数据的函数
        /// \return 磁盘占用率
        inline int GetSfUseInfo() { return m_sf_use_info; };

        /// 保存网络信息结构体vector的函数
        /// \param net_use_info_vector 网络信息结构体vector
        inline void
        SaveNetUseInfo(vector<NetUseInfo> &net_use_info_vector) { m_net_use_info_vector = net_use_info_vector; };

        /// 获取网络信息结构体vector的函数
        /// \return 网络信息结构体vector
        inline vector<NetUseInfo> GetNetUseInfo() { return m_net_use_info_vector; };

        /// 保存gpu使用信息结构体vector的函数
        /// \param gpu_use_info_vector gpu使用信息结构体vector
        inline void
        SaveGpuUseInfo(vector<GpuUseInfo> &gpu_use_info_vector) { m_gpu_use_info_vector = gpu_use_info_vector; };

        /// 获取gpu使用信息结构体vector的函数
        /// \return gpu使用信息结构体vector
        inline vector<GpuUseInfo> GetGpuUseInfo() { return m_gpu_use_info_vector; };

        /// 保存ip信息结构体vector的函数
        /// \param ip_info_vector ip信息结构体vector
        inline void
        SaveIpInfo(vector<IpInfo> &ip_info_vector) {
            lock_guard<mutex> m_ip_info_lock(m_ip_info_mutex);
            m_ip_info_vector = ip_info_vector;
        };

        /// 获取ip信息结构体vector的函数
        /// \return ip信息结构体vector
        inline vector<IpInfo> GetIpInfo() {
            lock_guard<mutex> m_ip_info_lock(m_ip_info_mutex);
            return m_ip_info_vector;
        };

        /// 保存话题状态信息结构体vector的函数
        /// \param topic_state_info_vector 话题状态信息结构体vector
        inline void
        SaveTopicStateInfo(vector<TopicStateInfo> &topic_state_info_vector) {
            lock_guard<mutex> m_topic_state_info_lock(m_topic_state_info_mutex);
            m_topic_state_info_vector = topic_state_info_vector;
        };

        /// 获取话题状态信息结构体vector的函数
        /// \return 话题状态信息结构体vector
        inline vector<TopicStateInfo> GetTopicStateInfo() {
            lock_guard<mutex> m_topic_state_info_lock(m_topic_state_info_mutex);
            return m_topic_state_info_vector;
        };

        /// 保存节点状态信息结构体vector的函数
        /// \param node_state_info_vector 节点状态信息结构体vector
        inline void
        SaveNodeStateInfo(vector<NodeStateInfo> &node_state_info_vector) {
            lock_guard<mutex> m_node_state_info_lock(m_node_state_info_mutex);
            m_node_state_info_vector = node_state_info_vector;
        };

        /// 获取节点状态信息结构体vector的函数
        /// \return 节点状态信息结构体vector
        inline vector<NodeStateInfo> GetNodeStateInfo() {
            lock_guard<mutex> m_node_state_info_lock(m_node_state_info_mutex);
            return m_node_state_info_vector;
        };
    };
}


#endif //CTI_MONITOR2_CTI_MONITOR2_INFO_H
