//
// Created by wq on 22-4-28.
//
#include "cti_monitor2_info.h"

using namespace CM2;
Info *Info::m_instance = nullptr;

Info *Info::GetInstance() {
    if (m_instance == nullptr) m_instance = new Info;
    return m_instance;
}

Info::Info() {

}

void Info::CreateConfigParameter() {
    m_cm2_node->declare_parameter("config_file_dir", string("/opt/ros/humble/share/cti_monitor2/config/"));
    m_config_parameter.config_file_dir = m_cm2_node->get_parameter("config_file_dir").as_string();

    m_cm2_node->declare_parameter("ip_address_config_file", string("ip_address_config.yaml"));
    m_config_parameter.ip_address_config_file_name = m_cm2_node->get_parameter("ip_address_config_file").as_string();

    m_cm2_node->declare_parameter("topic_state_config_file", string("topic_state.yaml"));
    m_config_parameter.topic_state_config_file_name = m_cm2_node->get_parameter("topic_state_config_file").as_string();

    m_cm2_node->declare_parameter("node_list_config_file", string("node_list.yaml"));
    m_config_parameter.node_list_config_file_name = m_cm2_node->get_parameter("node_list_config_file").as_string();
}






