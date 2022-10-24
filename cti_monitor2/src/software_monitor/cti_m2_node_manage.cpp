//
// Created by wq on 22-5-17.
//
#include "software_monitor/cti_m2_node_manage.h"

#define NODE_ALIVE_TIMEOUT 5.0

NodeManage *NodeManage::m_instance = nullptr;

NodeManage::NodeManage() {
    m_node_ptr = m_info->GetNodeHandle();
}

NodeManage::~NodeManage() {
    delete m_instance;
}

NodeManage *NodeManage::GetInstance() {
    if (m_instance == nullptr) m_instance = new NodeManage;
    return m_instance;
}

void NodeManage::RunNodeManage() {
    if (m_topic_manage_init_flag)
        InitNodeManage();
    else
        UpdateNodeState();
}

void NodeManage::InitNodeManage() {
    string version_num = m_info->GetVersionNum();
    if (version_num[0] != 'v') return;
    version_num=version_num.substr(0,2);
    std::cout << "读取config文件的版本： " << version_num << std::endl;
    ConfigParameter config_parameter = m_info->GetConfigParameter();
    string node_list_config_dir =
            config_parameter.config_file_dir + version_num + "/" + config_parameter.node_list_config_file_name;
           
    YAML::Node config;
    try {
        config = YAML::LoadFile(node_list_config_dir);
    } catch (YAML::BadFile &e) {
        std::cout << "read " << config_parameter.node_list_config_file_name << " error!" << std::endl;
        return;
    }
    for (auto i: config["nodes_list"]) {
        NodeStateInfo node_state_info;
        node_state_info.node_name = i.as<string>();
//        cout << "node_name :" << node_state_info.node_name << endl;
        m_node_state_info_vector.push_back(node_state_info);
//        cout << i.Type() << endl;
    }
    m_topic_manage_init_flag = false;
}

void NodeManage::UpdateNodeState() {
    m_node_alive_list = m_info->GetNodeHandle()->get_node_names();
    for (auto &i: m_node_state_info_vector) {
//        cout << "i_node_name :" << i.node_name << endl;
        i.is_alive = false;
        for (auto node_name: m_node_alive_list) {
            if (i.node_name == node_name) {
                i.is_alive = true;
                break;
            }
        }
    }
    m_info->SaveNodeStateInfo(m_node_state_info_vector);
}
