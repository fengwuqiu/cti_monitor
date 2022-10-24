//
// Created by wq on 22-5-12.
//
#include "system_monitor/cti_m2_ip_manage.h"

IpManage *IpManage::m_instance = nullptr;

IpManage::IpManage() {

}

IpManage::~IpManage() {
    delete m_instance;
}

IpManage *IpManage::GetInstance() {
    if (m_instance == nullptr) m_instance = new IpManage;
    return m_instance;
}

void IpManage::RunIpManage() {
    if (m_ip_init_flag)IpInit();
    else UpdateIpInfo();
}

void IpManage::IpInit() {
    string version_num = m_info->GetVersionNum();
    if (version_num[0] != 'v') return;
    version_num=version_num.substr(0,2);
    ConfigParameter config_parameter = m_info->GetConfigParameter();
    string ip_address_config_dir =
            config_parameter.config_file_dir + version_num + "/" + config_parameter.ip_address_config_file_name;
    YAML::Node config;
    try {
        config = YAML::LoadFile(ip_address_config_dir);
    } catch (YAML::BadFile &e) {
        std::cout << "read " << config_parameter.ip_address_config_file_name << " error!" << std::endl;
        return;
    }

    for (auto i: config["ip_address_info"]) {
        IpInfo ip_info;
        ip_info.ip_address = i["ip_address"].as<string>();
        ip_info.lidar_name = i["lidar_name"].as<string>();
        m_ip_info_vector.push_back(ip_info);
    }
    m_ip_init_flag = false;
}

void IpManage::UpdateIpInfo() {
    for (auto &ip_info: m_ip_info_vector) {
        ip_info.is_ip_alive = PingIp(ip_info.ip_address);
    }
    m_info->SaveIpInfo(m_ip_info_vector);
}

bool IpManage::PingIp(string ip_address) {
    string cmd;
    cmd += "ping -c 1 ";
    cmd += ip_address;
//    cout<<"cmd :"<<cmd<<endl;
    bool result = system(cmd.c_str()) == 0 ? true : false;
//    cout << "result :" << result << endl;
    return result;
}
