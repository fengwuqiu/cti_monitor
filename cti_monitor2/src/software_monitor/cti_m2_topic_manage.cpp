//
// Created by wq on 22-5-16.
//
#include "software_monitor/cti_m2_topic_manage.h"

TopicManage *TopicManage::m_instance = nullptr;

TopicManage::TopicManage() {
    m_node_ptr = m_info->GetNodeHandle();
    m_is_topic_alive_map[E_TOP_LIDAR_TOPIC] = false;
    m_is_topic_alive_map[E_FRONT_LEFT_LIDAR_TOPIC] = false;
    m_is_topic_alive_map[E_REAR_RIGHT_LIDAR_TOPIC] = false;
    m_is_topic_alive_map[E_LH_LIDAR_TOPIC] = false;
    m_is_topic_alive_map[E_IMU_TOPIC] = false;
}

TopicManage::~TopicManage() {
    delete m_instance;
}

TopicManage *TopicManage::GetInstance() {
    if (m_instance == nullptr) m_instance = new TopicManage;
    return m_instance;
}

void TopicManage::RunTopicManage() {
    if (m_topic_manage_init_flag)
        InitTopicManage();
    else
        UpdateTopicState();
}

void TopicManage::InitTopicManage() {
    string version_num = m_info->GetVersionNum();
    if (version_num[0] != 'v') return;
    version_num=version_num.substr(0,2);
    ConfigParameter config_parameter = m_info->GetConfigParameter();
    string topic_state_config_dir =
            config_parameter.config_file_dir + version_num + "/" + config_parameter.topic_state_config_file_name;
    YAML::Node config;
    try {
        config = YAML::LoadFile(topic_state_config_dir);
    } catch (YAML::BadFile &e) {
        std::cout << "read " << config_parameter.topic_state_config_file_name << " error!" << std::endl;
        return;
    }

    for (auto i: config["topic_state"]) {
        TopicStateInfo topic_state_info;
        topic_state_info.topic_name = i["topic"].as<string>();
        topic_state_info.name = i["name"].as<string>();
        topic_state_info.timeout = i["timeout"].as<double>();
        topic_state_info.enable = i["enable"].as<bool>();
//        cout<<"topic_name :"<<topic_state_info.topic_name<<endl;
//        cout<<"name :"<<topic_state_info.name<<endl;
//        cout<<"timeout :"<<topic_state_info.timeout<<endl;
//        cout<<"enable :"<<(int)topic_state_info.enable<<endl;
        m_topic_state_info_vector.push_back(topic_state_info);
    }
    m_topic_state_info_vector = CtiMonitor2Node::GetInstance()->CreateTopicSub(m_topic_state_info_vector);

    double time = m_node_ptr->get_clock()->now().seconds();
    m_topic_time_map[E_TOP_LIDAR_TOPIC] = time;
    m_topic_time_map[E_FRONT_LEFT_LIDAR_TOPIC] = time;
    m_topic_time_map[E_REAR_RIGHT_LIDAR_TOPIC] = time;
    m_topic_time_map[E_LH_LIDAR_TOPIC] = time;
    m_topic_time_map[E_IMU_TOPIC] = time;

    m_topic_manage_init_flag = false;
}

void TopicManage::UpdateIsTopicAliveFlag(int map_index) {
    lock_guard<mutex> m_map_lock(m_map_mutex);
    m_is_topic_alive_map[map_index] = true;
}

void TopicManage::UpdateTopicState() {
    end_timeStamp = m_node_ptr->get_clock()->now().seconds();
    for (auto &i: m_topic_state_info_vector) {
        if (i.enable) {
            lock_guard<mutex> m_map_lock(m_map_mutex);
            if (m_is_topic_alive_map[i.type]) {
                if (IsTopicTimeout(i.type, i.timeout))
                    i.is_timeout = true;
                else
                    i.is_timeout = false;
                m_is_topic_alive_map[i.type] = false;
                m_topic_time_map[i.type] = end_timeStamp;
            } else {
                if (IsTopicTimeout(i.type, i.timeout))
                    i.is_timeout = true;
                else
                    i.is_timeout = false;
            }
        } else i.is_timeout = true;
    }
    m_info->SaveTopicStateInfo(m_topic_state_info_vector);
}

bool TopicManage::IsTopicTimeout(int type, double timeout) {
//    cout << "type :" << type << " diff :" << (end_timeStamp - m_topic_time_map[type]) << endl;
    if ((end_timeStamp - m_topic_time_map[type]) > timeout) {
//        cout << "timeout" << endl;
        return true;
    } else
        return false;
}
