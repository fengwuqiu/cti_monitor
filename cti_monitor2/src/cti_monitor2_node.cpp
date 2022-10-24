//
// Created by wq on 22-4-28.
//
#include "cti_monitor2_info.h"
#include "cti_monitor2_node.h"
#include "software_monitor/cti_m2_topic_manage.h"

#define ALL_LOG_FLAG 1
#define CPU_LOG_FLAG 1
#define PRO_LOG_FLAG 1
#define LOAD_AVERAGE_LOG_FLAG 1
#define MEM_LOG_FLAG 1
#define DISK_LOG_FLAG 1
#define NET_LOG_FLAG 1
#define GPU_LOG_FLAG 1
#define IP_LOG_FLAG 1
#define TOPIC_LOG_FLAG 1
#define NODE_LOG_FLAG 1

CtiMonitor2Node *CtiMonitor2Node::m_instance = nullptr;

CtiMonitor2Node::CtiMonitor2Node()
{
    m_node = Info::GetInstance()->GetNodeHandle();

    m_version_num_sub = m_node->create_subscription<std_msgs::msg::String>("/robot_config/parameter_server",
                                                                           rclcpp::QoS{1}.transient_local(),
                                                                           bind(&CtiMonitor2Node::VersionNumSubCallBack,
                                                                                this, std::placeholders::_1));

    m_system_status_pub = m_node->create_publisher<cti_msgs::msg::SystemStatusInfo>("cti_monitor2/system_status", 1);
    m_topic_status_pub = m_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("cti_monitor2/topic_status",
                                                                                         1);
    m_node_status_pub = m_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("cti_monitor2/node_status", 1);
}

CtiMonitor2Node::~CtiMonitor2Node()
{
    delete m_instance;
}

CtiMonitor2Node *CtiMonitor2Node::GetInstance()
{
    if (m_instance == nullptr)
        m_instance = new CtiMonitor2Node;
    return m_instance;
}

void CtiMonitor2Node::VersionNumSubCallBack(const std_msgs::msg::String::SharedPtr msg)
{
    if (m_version_read_flag)
    {
        Json::Value parameter;
        Json::Reader reader;
        string version_num;
        if (reader.parse(msg->data, parameter) && parameter.isObject())
        {
            if (parameter.isMember("CTI_RUN_VER"))
            {
                version_num = parameter["CTI_RUN_VER"].asString();
                cout << "get version num :" << version_num << endl;
            }
        }
        // version_num = "v7.0";
        Info::GetInstance()->SaveVersionNum(version_num);
        m_version_read_flag = false;
    }
}

void CtiMonitor2Node::TopLidarSubCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    TopicManage::GetInstance()->UpdateIsTopicAliveFlag(E_TOP_LIDAR_TOPIC);
}

void CtiMonitor2Node::FrontLeftLidarSubCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    TopicManage::GetInstance()->UpdateIsTopicAliveFlag(E_FRONT_LEFT_LIDAR_TOPIC);
}

void CtiMonitor2Node::RearRightLidarSubCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    TopicManage::GetInstance()->UpdateIsTopicAliveFlag(E_REAR_RIGHT_LIDAR_TOPIC);
}

void CtiMonitor2Node::LhLidarSubCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    TopicManage::GetInstance()->UpdateIsTopicAliveFlag(E_LH_LIDAR_TOPIC);
}

void CtiMonitor2Node::ImuSubCallBack(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    TopicManage::GetInstance()->UpdateIsTopicAliveFlag(E_IMU_TOPIC);
}

vector<TopicStateInfo> CtiMonitor2Node::CreateTopicSub(vector<TopicStateInfo> &topic_state_info_vector)
{
    for (auto &i : topic_state_info_vector)
    {
        if (i.name == "lh_lidar" && i.enable)
        {
            m_lh_lidar_sub = m_node->create_subscription<sensor_msgs::msg::LaserScan>(i.topic_name,
                                                                                      rclcpp::QoS{1}.best_effort(),
                                                                                      bind(&CtiMonitor2Node::LhLidarSubCallBack,
                                                                                           this,
                                                                                           std::placeholders::_1));
            //            m_lh_lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(i.topic_name, 1,&CtiMonitor2Node::LhLidarSubCallBack);
            i.type = E_LH_LIDAR_TOPIC;
        }
        else if (i.name == "front_left" && i.enable)
        {
            m_front_left_lidar_sub = m_node->create_subscription<sensor_msgs::msg::PointCloud2>(i.topic_name,
                                                                                                rclcpp::QoS{
                                                                                                    1}
                                                                                                    .best_effort(),
                                                                                                bind(&CtiMonitor2Node::FrontLeftLidarSubCallBack,
                                                                                                     this,
                                                                                                     std::placeholders::_1));
            i.type = E_FRONT_LEFT_LIDAR_TOPIC;
        }
        else if (i.name == "rear_right" && i.enable)
        {
            m_rear_right_lidar_sub = m_node->create_subscription<sensor_msgs::msg::PointCloud2>(i.topic_name,
                                                                                                rclcpp::QoS{
                                                                                                    1}
                                                                                                    .best_effort(),
                                                                                                bind(&CtiMonitor2Node::RearRightLidarSubCallBack,
                                                                                                     this,
                                                                                                     std::placeholders::_1));
            i.type = E_REAR_RIGHT_LIDAR_TOPIC;
        }
        else if (i.name == "top_lidar" && i.enable)
        {
            m_top_lidar_sub = m_node->create_subscription<sensor_msgs::msg::PointCloud2>(i.topic_name,
                                                                                         rclcpp::QoS{1}.best_effort(),
                                                                                         bind(&CtiMonitor2Node::TopLidarSubCallBack,
                                                                                              this,
                                                                                              std::placeholders::_1));
            i.type = E_TOP_LIDAR_TOPIC;
        }
        else if (i.name == "imu" && i.enable)
        {
            m_imu_sub = m_node->create_subscription<sensor_msgs::msg::Imu>(i.topic_name, 1,
                                                                           bind(&CtiMonitor2Node::ImuSubCallBack, this,
                                                                                std::placeholders::_1));
            i.type = E_IMU_TOPIC;
        }
    }
    return topic_state_info_vector;
}

void CtiMonitor2Node::PubMonitorInfo()
{
    PubSystemStatus();
    PubTopicStatus();
    PubNodeStatus();
}

void CtiMonitor2Node::GetTopLidarStatus()
{
    Info *info = Info::GetInstance();
    std::string top_lidar_ip = "192.168.1.200"; //上雷达ip地址

    vector<TopicStateInfo> topic_state_info_vector = info->GetTopicStateInfo();
    for (auto i : topic_state_info_vector)
    {
        if (i.name == "top_lidar" && i.is_timeout)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "......top_lidar error......");
            recordPcapFileTcpdump(top_lidar_ip); //抓取上雷达异常数据
            std::this_thread::sleep_for(std::chrono::seconds(20));
        }
    }
}

void CtiMonitor2Node::PubSystemStatus()
{
    Info *info = Info::GetInstance();

    auto system_status_info_msg = cti_msgs::msg::SystemStatusInfo();

    std_msgs::msg::Header header;
    header.frame_id = "cti_monitor2";
    header.stamp = m_node->get_clock()->now();
    system_status_info_msg.header = header;

    vector<CpuUseInfo> cpu_use_info_vector = info->GetCpuUseInfo();
    auto cpu_status_msg = cti_msgs::msg::CpuStatus();
    for (auto i : cpu_use_info_vector)
    {
        cpu_status_msg.cpu_index = i.cpu_index;
        cpu_status_msg.cpu_percent = i.cpu_percent;
        system_status_info_msg.cpu_status_array.push_back(cpu_status_msg);
#if CPU_LOG_FLAG && ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "cpu_index :" << i.cpu_index);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "cpu_percent :" << i.cpu_percent);
#endif
    }

    float cpu_temp = info->GetCpuTemp();
    system_status_info_msg.cpu_temp = cpu_temp;
#if CPU_LOG_FLAG && ALL_LOG_FLAG
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "cpu_temp :" << cpu_temp);
#endif

    vector<ProcessCpuInfo> pro_cpu_info_vector = info->GetProcessCpuInfo();
    auto process_status_msg = cti_msgs::msg::ProcessStatus();
    for (auto i : pro_cpu_info_vector)
    {
        process_status_msg.pro_name = i.pro_name;
        process_status_msg.pro_cpu_percent = i.cpu_percent;
        system_status_info_msg.process_status_array.push_back(process_status_msg);
#if PRO_LOG_FLAG && ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "pro_name :" << i.pro_name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "pro_cpu_percent :" << i.cpu_percent);
#endif
    }

    MemInfo mem_info = info->GetMemInfo();
    system_status_info_msg.mem_status.used_mem = mem_info.usedMem;
    system_status_info_msg.mem_status.total_mem = mem_info.totalMem;
    system_status_info_msg.mem_status.mem_usage = mem_info.MemUsage;
    system_status_info_msg.mem_status.used_swap = mem_info.usedSwap;
    system_status_info_msg.mem_status.total_swap = mem_info.totalSwap;
    system_status_info_msg.mem_status.swap_usage = mem_info.SwapUsage;
#if MEM_LOG_FLAG && ALL_LOG_FLAG
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "usedMem :" << mem_info.usedMem);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "totalMem :" << mem_info.totalMem);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "usedSwap :" << mem_info.usedSwap);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "totalSwap :" << mem_info.totalSwap);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "MemUsage :" << mem_info.MemUsage);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "SwapUsage :" << mem_info.SwapUsage);
#endif

    LoadAverage la_info = info->GetLoadAverageInfo();
    system_status_info_msg.load_average_status.one = la_info.one;
    system_status_info_msg.load_average_status.five = la_info.five;
    system_status_info_msg.load_average_status.fifteen = la_info.fifteen;
#if LOAD_AVERAGE_LOG_FLAG && ALL_LOG_FLAG
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "load_average_one :" << la_info.one);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "load_average_five :" << la_info.five);
    RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "load_average_fifteen :" << la_info.fifteen);
#endif

    int sf_use_info = info->GetSfUseInfo();
    system_status_info_msg.disk_usage = sf_use_info;
#if DISK_LOG_FLAG && ALL_LOG_FLAG
    RCLCPP_INFO_STREAM(m_node->get_logger(), "stat_fs_info :" << sf_use_info);
#endif

    vector<NetUseInfo> net_use_info_vector = info->GetNetUseInfo();
    auto net_status_msg = cti_msgs::msg::NetStatus();
    for (auto i : net_use_info_vector)
    {
        net_status_msg.net_name = i.net_name;
        net_status_msg.receive_speed_kbs = i.net_receive_speed;
        net_status_msg.receive_packets = i.net_receive_packets;
        net_status_msg.receive_packets_errs = i.net_receive_packets_errs;
        net_status_msg.receive_packets_drop = i.net_receive_packets_drop;
        net_status_msg.transmit_speed_kbs = i.net_transmit_speed;
        net_status_msg.transmit_packets = i.net_transmit_packets;
        net_status_msg.transmit_packets_errs = i.net_transmit_packets_errs;
        net_status_msg.transmit_packets_drop = i.net_transmit_packets_drop;
        system_status_info_msg.net_status_array.push_back(net_status_msg);
#if NET_LOG_FLAG && ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "net_name :" << i.net_name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "net_receive_speed :" << i.net_receive_speed);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "net_receive_packets :" << i.net_receive_packets);
        RCLCPP_INFO_STREAM(m_node->get_logger(),
                           LOG_HEAD << "net_receive_packets_errs :" << i.net_receive_packets_errs);
        RCLCPP_INFO_STREAM(m_node->get_logger(),
                           LOG_HEAD << "net_receive_packets_drop :" << i.net_receive_packets_drop);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "net_transmit_speed :" << i.net_transmit_speed);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "net_transmit_packets :" << i.net_transmit_packets);
        RCLCPP_INFO_STREAM(m_node->get_logger(),
                           LOG_HEAD << "net_transmit_packets_errs :" << i.net_transmit_packets_errs);
        RCLCPP_INFO_STREAM(m_node->get_logger(),
                           LOG_HEAD << "net_transmit_packets_drop :" << i.net_transmit_packets_drop);
#endif
    }

    vector<GpuUseInfo> gpu_use_info_vector = info->GetGpuUseInfo();
    auto gpu_status_msg = cti_msgs::msg::GpuStatus();
    for (auto i : gpu_use_info_vector)
    {
        gpu_status_msg.gpu_index = i.gpu_index;
        gpu_status_msg.gpu_name = i.gpu_name;
        gpu_status_msg.gpu_use_percent = i.gpu_use_percent;
        gpu_status_msg.gpu_mem_percent = i.gpu_mem_percent;
        gpu_status_msg.gpu_temp = i.gpu_temp;
        system_status_info_msg.gpu_status_array.push_back(gpu_status_msg);
#if GPU_LOG_FLAG && ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "gpu_index :" << i.gpu_index);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "gpu_name :" << i.gpu_name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "gpu_use_percent :" << i.gpu_use_percent);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "gpu_mem_percent :" << i.gpu_mem_percent);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "gpu_temp :" << i.gpu_temp);
#endif
    }

    vector<IpInfo> ip_info_vector = info->GetIpInfo();
    auto ip_info_msg = cti_msgs::msg::IpStatus();
    for (auto i : ip_info_vector)
    {
        ip_info_msg.ip_address = i.ip_address;
        ip_info_msg.lidar_name = i.lidar_name;
        ip_info_msg.is_ip_alive = i.is_ip_alive;
        system_status_info_msg.ip_status_array.push_back(ip_info_msg);
#if IP_LOG_FLAG && ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "ip_address :" << i.ip_address);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "lidar_name :" << i.lidar_name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "is_ip_alive :" << i.is_ip_alive);
#endif
    }
    m_system_status_pub->publish(system_status_info_msg);
}

void CtiMonitor2Node::PubTopicStatus()
{
    Info *info = Info::GetInstance();

    auto topic_status_msgs = diagnostic_msgs::msg::DiagnosticArray();
    std_msgs::msg::Header header;
    header.frame_id = "cti_monitor2";
    header.stamp = m_node->get_clock()->now();
    topic_status_msgs.header = header;

    vector<TopicStateInfo> topic_state_info_vector = info->GetTopicStateInfo();
    auto topic_state_msg = diagnostic_msgs::msg::DiagnosticStatus();
    for (auto i : topic_state_info_vector)
    {
        topic_state_msg.name = i.topic_name;
        if (i.is_timeout)
        {
            topic_state_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            topic_state_msg.message = i.topic_name + "/is time out";
        }
        else
        {
            topic_state_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            topic_state_msg.message = i.topic_name + "/is run now";
        }
        topic_status_msgs.status.push_back(topic_state_msg);
#if TOPIC_LOG_FLAG && ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "topic_name :" << i.topic_name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "sensor_type :" << i.name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "topic_timeout :" << i.timeout);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "is_topic_enable :" << i.enable);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "is_topic_timeout :" << i.is_timeout);
#endif
    }
    m_topic_status_pub->publish(topic_status_msgs);
}

void CtiMonitor2Node::PubNodeStatus()
{
    Info *info = Info::GetInstance();

    auto node_status_msgs = diagnostic_msgs::msg::DiagnosticArray();
    std_msgs::msg::Header header;
    header.frame_id = "cti_monitor2";
    header.stamp = m_node->get_clock()->now();
    node_status_msgs.header = header;

    vector<NodeStateInfo> node_state_info_vector = info->GetNodeStateInfo();
    auto node_state_msg = diagnostic_msgs::msg::DiagnosticStatus();
    for (auto i : node_state_info_vector)
    {
        node_state_msg.name = i.node_name;
        if (i.is_alive)
        {
            node_state_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            node_state_msg.message = node_state_msg.name + "/is alive";
        }
        else
        {
            node_state_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            node_state_msg.message = node_state_msg.name + "/is dead";
        }
        node_status_msgs.status.push_back(node_state_msg);
#if NODE_LOG_FLAG && ALL_LOG_FLAG
        //#if NODE_LOG_FLAG || ALL_LOG_FLAG
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "node_name :" << i.node_name);
        RCLCPP_INFO_STREAM(m_node->get_logger(), LOG_HEAD << "is_node_alive :" << i.is_alive);
#endif
    }
    m_node_status_pub->publish(node_status_msgs);
}

void CtiMonitor2Node::recordPcapFileTcpdump(const std::string lidar_ip)
{
    double record_floder_size_max_ = 500.0; //设置存储数据包文件大小最大为 500M
    std::string robot_pwd_ = "0";           // docker容器密码
    std::string robot_num_ = "888";         // 车号 ，暂统一用888
    std::string record_dura_ = "10";        //录制10s的数据包
    time_t tm = time(0);
    char time_now[64];

    ctrlRecordFolderSize(record_floder_size_max_);
    strftime(time_now, sizeof(time_now), "%Y-%m-%d-%H:%M:%S", localtime(&tm));
    std::string time_now_str = time_now; //开始的时间
    system("mkdir -p ~/pcap");
    std::string pc_ip = "192.168.1.102";
    if (getDev(pc_ip))
    {
        std::string dev_lidar_str_ = dev_lidar;
        std::string file_name = robot_num_ + "_" + lidar_ip + "_" + time_now_str + "_error_" + ".pcap";
        std::string record_cmd = "echo \"" + robot_pwd_ + "\" | sudo -S timeout " + record_dura_ +
                                 " tcpdump -i " + dev_lidar_str_ + " host " + lidar_ip + " -w " + record_folder_ + file_name + " &";
        system(record_cmd.c_str());
        printf("record_cmd: %s", record_cmd.c_str());
        std::string chmod_cmd = "echo \"" + robot_pwd_ + "\" | sudo -S chmod 777 -R " + record_folder_;
        system(chmod_cmd.c_str());
        printf("rchmod_cmd: %s", chmod_cmd.c_str());
        printf("Record pcap file success,file location: %s%s\n", record_folder_.c_str(), file_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Cannot get lidar dev...");
    }
}

void CtiMonitor2Node::ctrlRecordFolderSize(double size_max)
{
    bool in_folder_size_limit = false;
    char *record_folder_char = new char[256];
    strcpy(record_folder_char, record_folder_.c_str());
    while (!in_folder_size_limit)
    {
        long long int folder_size = getDirectorySize(record_folder_char);
        int folder_size_M = folder_size / 1000 / 1000;
        // printf("folder size: %dM\n",folder_size_M);
        if (folder_size_M < size_max)
        {
            in_folder_size_limit = true;
        }
        else
        {
            deleteOldestFile(record_folder_);
        }
    }
}

//获取文件夹的大小包含子文件夹
long long int CtiMonitor2Node::getDirectorySize(char *dir)
{
    DIR *dp;
    struct dirent *entry;
    struct stat statbuf;
    long long int total_size = 0;

    if ((dp = opendir(dir)) == NULL)
    {
        printf("Can not open dir: %s\n", dir);
        return -1;
    }
    lstat(dir, &statbuf);
    total_size += statbuf.st_size;
    while ((entry = readdir(dp)) != NULL)
    {
        char subdir[256];
        sprintf(subdir, "%s/%s", dir, entry->d_name);
        lstat(subdir, &statbuf);
        if (S_ISDIR(statbuf.st_mode))
        {
            if (strcmp(".", entry->d_name) == 0 || strcmp("..", entry->d_name) == 0)
            {
                continue;
            }
            long long int subDirSize = getDirectorySize(subdir);
            total_size += subDirSize;
        }
        else
        {
            total_size += statbuf.st_size;
        }
    }

    closedir(dp);
    return total_size;
}

//删除 dir下时间最早的一个文件
void CtiMonitor2Node::deleteOldestFile(std::string dir)
{
    std::string delete_cmd = "rm " + dir + "`ls " + dir + " -tr | head -1`";
    system(delete_cmd.c_str());
}

//获取ip为pc_ip_的网卡名称
bool CtiMonitor2Node::getDev(const std::string &pc_ip)
{
    struct ifaddrs *ifaddr = NULL;
    int ret = getifaddrs(&ifaddr);
    if (ret)
    {

        printf("get faddrs failed,errno:%d\n", errno);
        return false;
    }
    struct ifaddrs *ifp = ifaddr;
    char ip[16];
    char netmask[16];
    for (; ifp != NULL; ifp = ifp->ifa_next)
    {
        if (ifp->ifa_addr && ifp->ifa_addr->sa_family == AF_INET)
        {
            strncpy(ip, inet_ntoa(((struct sockaddr_in *)ifp->ifa_addr)->sin_addr), 16);
            printf("dev:%s, ip:%s\n", ifp->ifa_name, ip);
            if (!strncmp(ip, pc_ip.c_str(), 16))
            {
                dev_lidar = ifp->ifa_name;
                printf("get dev_lidar_: %s\n", dev_lidar);
                return true;
            }
        }
    }
    return false;
}
