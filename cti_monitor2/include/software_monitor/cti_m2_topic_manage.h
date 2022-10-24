//
// Created by wq on 22-5-16.
//

#ifndef CTI_MONITOR2_CTI_M2_TOPIC_MANAGE_H
#define CTI_MONITOR2_CTI_M2_TOPIC_MANAGE_H

#include "cti_monitor2_info.h"

using namespace CM2;
namespace CM2 {
    class TopicManage {
    public:
        static TopicManage *GetInstance();//获取该类的单例指针

    private:
        TopicManage();

        ~TopicManage();

        static TopicManage *m_instance;//该类的单例指针

    private:
        Info *m_info = Info::GetInstance();//存放信息类的单例指针
        shared_ptr<rclcpp::Node> m_node_ptr;//存放节点指针

        bool m_topic_manage_init_flag = true;//初始化flag

        vector<TopicStateInfo> m_topic_state_info_vector;//存放话题状态信息vector

        map<int, bool> m_is_topic_alive_map;//存放话题索引对应的存活状态的map
        map<int, double> m_topic_time_map;//存放话题索引对应的时间戳数据的map

        mutex m_map_mutex;//话题索引对应的存活状态的map使用的线程锁

        double end_timeStamp;//存放每次进行更新时的时间戳

    private:
        void InitTopicManage();//初始化函数

        void UpdateTopicState();//更新话题状态信息

        bool IsTopicTimeout(int type, double timeout);//判断话题是否超时

    public:
        void RunTopicManage();//供外部调用的运行话题状态监测功能的函数

        void UpdateIsTopicAliveFlag(int map_index);//更新话题存活状态的函数

    };
}
#endif //CTI_MONITOR2_CTI_M2_TOPIC_MANAGE_H
