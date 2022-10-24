//
// Created by wq on 22-5-17.
//

#ifndef CTI_MONITOR2_CTI_M2_NODE_MANAGE_H
#define CTI_MONITOR2_CTI_M2_NODE_MANAGE_H

#include "cti_monitor2_info.h"

using namespace CM2;
namespace CM2 {
    class NodeManage {
    public:
        static NodeManage *GetInstance();//获取该类的单例指针

    private:
        NodeManage();

        ~NodeManage();

        static NodeManage *m_instance;//该类的单例指针

    private:
        Info *m_info = Info::GetInstance();//存放信息类的单例指针

        shared_ptr<rclcpp::Node> m_node_ptr;//存放节点指针

        bool m_topic_manage_init_flag = true;//初始化flag

        vector<string> m_node_alive_list;//存放当前系统中活动的节点的列表
        vector<NodeStateInfo> m_node_state_info_vector;//存放节点状态信息结构体vector

    private:
        void InitNodeManage();//初始化函数

        void UpdateNodeState();//更新节点状态

    public:
        void RunNodeManage();//供外部调用的运行节点状态监测功能的函数

    };
}
#endif //CTI_MONITOR2_CTI_M2_NODE_MANAGE_H
