//
// Created by wq on 22-4-28.
//

#ifndef CTI_MONITOR2_CTI_MONITOR2_NODE_H
#define CTI_MONITOR2_CTI_MONITOR2_NODE_H

//#include "cti_monitor2_info.h"
#include "cti_monitor2_define.h"

#include <dirent.h>
#include <sys/stat.h>
#include <ifaddrs.h>
#include <arpa/inet.h>

using namespace CM2;

class CtiMonitor2Node
{
public:
    static CtiMonitor2Node *GetInstance(); //获取该类的单例指针

private:
    CtiMonitor2Node();

    ~CtiMonitor2Node();

    static CtiMonitor2Node *m_instance; //该类的单例指针
private:
    //    Info *m_info = Info::GetInstance();
    shared_ptr<rclcpp::Node> m_node = nullptr; //保存节点指针的成员

    bool m_version_read_flag = true; //是否读取了版本号的flag
    // sub
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_version_num_sub;              //订阅版本号
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_top_lidar_sub;        //订阅上雷达话题
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_front_left_lidar_sub; //订阅左前雷达话题
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_rear_right_lidar_sub; //订阅右后雷达话题
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_lh_lidar_sub;           //订阅lh雷达话题
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;                      //订阅imu话题
    // pub
    rclcpp::Publisher<cti_msgs::msg::SystemStatusInfo>::SharedPtr m_system_status_pub;      //发布系统状态
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_topic_status_pub; //发布话题状态
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_node_status_pub;  //发布节点状态

    char *dev_lidar; //网口的名称
    std::string record_folder_ = "/home/neousys/pcap/";  //保存雷达数据文件地址
private:
    void VersionNumSubCallBack(const std_msgs::msg::String::SharedPtr msg); //订阅版本号的回调函数

    void TopLidarSubCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg); //订阅上雷达话题的回调函数

    void FrontLeftLidarSubCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg); //订阅左前雷达话题的回调函数

    void RearRightLidarSubCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg); //订阅右后雷达话题的回调函数

    void LhLidarSubCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg); //订阅lh雷达话题的回调函数

    void ImuSubCallBack(const sensor_msgs::msg::Imu::SharedPtr msg); //订阅imu话题的回调函数

    void PubSystemStatus(); //发布系统状态

    void PubTopicStatus(); //发布话题状态

    void PubNodeStatus(); //发布节点状态

    void recordPcapFileTcpdump(const std::string lidar_ip); //使用tcpdump库进行抓包

    void ctrlRecordFolderSize(double size_max); //控制保存pcap文件文件夹的大小,如果大于限制大小,删除最旧的文件,直到小于限制

    long long int getDirectorySize(char *dir); //获取文件夹的大小包含子文件夹

    bool getDev(const std::string &pc_ip); //获取ip为pc_ip_的网卡名称

    void deleteOldestFile(std::string dir); //删除 dir下时间最早的一个文件

public:
    void PubMonitorInfo(); //发布所有监测信息

    void GetTopLidarStatus(); //获取上雷达的异常数据

    vector<TopicStateInfo> CreateTopicSub(vector<TopicStateInfo> &topic_state_info_vector); //创建话题订阅者的函数
};

#endif // CTI_MONITOR2_CTI_MONITOR2_NODE_H
