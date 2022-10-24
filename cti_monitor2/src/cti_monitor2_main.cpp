//
// Created by wq on 22-4-28.
//
#include "cti_monitor2_node.h"
#include "system_monitor/cti_m2_cpu_manage.h"
#include "system_monitor/cti_m2_net_manage.h"
#include "system_monitor/cti_m2_gpu_manage.h"
#include "system_monitor/cti_m2_ip_manage.h"
#include "software_monitor/cti_m2_topic_manage.h"
#include "software_monitor/cti_m2_node_manage.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto cm2_node = std::make_shared<rclcpp::Node>("cti_monitor_2_node");
    //存放节点指针
    Info::GetInstance()->SaveNodeHandle(cm2_node);
    //获取配置参数
    Info::GetInstance()->CreateConfigParameter();

    rclcpp::Rate loop_rate(1);
    ///运行ip监测功能的线程
    auto IpManageThread = [&]() {
        while (rclcpp::ok()) {
            IpManage::GetInstance()->RunIpManage();
            loop_rate.sleep();
        }
    };
    thread ip_manage_thread(IpManageThread);
    ///运行软件监测功能的线程
    auto SoftwareManageThread = [&]() {
        rclcpp::Rate rate(10.0);
        while (rclcpp::ok()) {
            RCLCPP_INFO_STREAM(cm2_node->get_logger(), LOG_HEAD << "software manage start_time");
            TopicManage::GetInstance()->RunTopicManage();
            NodeManage::GetInstance()->RunNodeManage();
            RCLCPP_INFO_STREAM(cm2_node->get_logger(), LOG_HEAD << "software manage end_time");
            rate.sleep();
        }
    };
    thread software_manage_thread(SoftwareManageThread);
    ///运行硬件监测功能的线程
    auto HardwareManageThread = [&]() {
        while (rclcpp::ok()) {
            RCLCPP_INFO_STREAM(cm2_node->get_logger(), LOG_HEAD << "hardware manage start_time");
            CpuManage::GetInstance()->RunCpuManage();
            NetManage::GetInstance()->RunNetManage();
            GpuManage::GetInstance()->RunGpuManage();
            CtiMonitor2Node::GetInstance()->PubMonitorInfo();
            RCLCPP_INFO_STREAM(cm2_node->get_logger(), LOG_HEAD << "hardware manage end_time");
            loop_rate.sleep();
        }
    };
    thread hardware_manage_thread(HardwareManageThread);

    auto GetExceptionFile=[&]{
        rclcpp::Rate lidar_rate(1);
        while(rclcpp::ok()){
            RCLCPP_INFO_STREAM(cm2_node->get_logger(),LOG_HEAD<<"GetExceptionFile");
            CtiMonitor2Node::GetInstance()->GetTopLidarStatus();
            lidar_rate.sleep();
        }
    };
    thread get_exception_file(GetExceptionFile);
    
    rclcpp::spin(cm2_node);

    ip_manage_thread.join();
    software_manage_thread.join();
    hardware_manage_thread.join();
    get_exception_file.join();
    rclcpp::shutdown();
    return 0;
}