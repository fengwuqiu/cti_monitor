//
// Created by wq on 22-4-28.
//

#ifndef CTI_MONITOR2_CTI_MONITOR2_DEFINE_H
#define CTI_MONITOR2_CTI_MONITOR2_DEFINE_H

#include <rclcpp/rclcpp.hpp>
//
#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <ctime>
#include <cstdlib>
//
#include <armadillo>
#include <cstddef>
#include <iomanip>
#include <fcntl.h>
#include <sys/sysinfo.h>
#include <sys/statfs.h>
#include <sys/time.h>
//
#include <nvidia/gdk/nvml.h>
#include <yaml-cpp/yaml.h>
#include <jsoncpp/json/json.h>
//
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <cti_msgs/msg/system_status_info.hpp>


using namespace std;

namespace CM2 {
    /*****************************VARIABLE*******************************/
#ifndef LOG_HEAD
#define LOG_HEAD "cti_monitor_2::"
#endif
#ifndef PROCDIR
#define PROCDIR "/proc"
#endif
#ifndef PROCSTATFILE
#define PROCSTATFILE PROCDIR "/stat"
#endif
#ifndef PROCMEMINFOFILE
#define PROCMEMINFOFILE PROCDIR "/meminfo"
#endif
#ifndef SYSCLASSTHERMALTEMPDIR
#define SYSCLASSTHERMALTEMPDIR "/sys/class/thermal/thermal_zone4/temp"
#endif
#ifndef MAX_NAME
#define MAX_NAME 128
#endif
#ifndef MAX_READ
#define MAX_READ 2048
#endif
#ifndef PROC_LINE_LENGTH
#define PROC_LINE_LENGTH 512
#endif
#ifndef String_startsWith
#define String_startsWith(s, match) (strncmp((s),(match),strlen(match)) == 0)
#endif
#ifndef MIN
#define MIN(a, b) ((a)<(b)?(a):(b))
#endif
#ifndef MAX
#define MAX(a, b) ((a)>(b)?(a):(b))
#endif
#ifndef CLAMP
#define CLAMP(x, low, high) (((x)>(high))?(high):(((x)<(low))?(low):(x)))
#endif
    /*****************************ENUM***********************************/
    typedef enum TopicList_{
        E_TOP_LIDAR_TOPIC = 0,
        E_FRONT_LEFT_LIDAR_TOPIC = 1,
        E_REAR_RIGHT_LIDAR_TOPIC = 2,
        E_LH_LIDAR_TOPIC = 3,
        E_IMU_TOPIC = 4,
    }TopicList;
    /*****************************STRUCT*********************************/
    typedef struct ProcessList_ {
        unsigned long long int totalMem;
        unsigned long long int usedMem;
        unsigned long long int freeMem;
        unsigned long long int sharedMem;
        unsigned long long int buffersMem;
        unsigned long long int cachedMem;
        unsigned long long int totalSwap;
        unsigned long long int usedSwap;
        unsigned long long int freeSwap;

        int cpuCount;

    } ProcessList;
    typedef struct CPUData_ {
        unsigned long long int totalTime;
        unsigned long long int userTime;
        unsigned long long int systemTime;
        unsigned long long int systemAllTime;
        unsigned long long int idleAllTime;
        unsigned long long int idleTime;
        unsigned long long int niceTime;
        unsigned long long int ioWaitTime;
        unsigned long long int irqTime;
        unsigned long long int softIrqTime;
        unsigned long long int stealTime;
        unsigned long long int guestTime;

        unsigned long long int totalPeriod;
        unsigned long long int userPeriod;
        unsigned long long int systemPeriod;
        unsigned long long int systemAllPeriod;
        unsigned long long int idleAllPeriod;
        unsigned long long int idlePeriod;
        unsigned long long int nicePeriod;
        unsigned long long int ioWaitPeriod;
        unsigned long long int irqPeriod;
        unsigned long long int softIrqPeriod;
        unsigned long long int stealPeriod;
        unsigned long long int guestPeriod;
    } CPUData;

    typedef struct LinuxProcessList_ {
        ProcessList super;
        vector<CPUData> cpus;
    } LinuxProcessList;

    typedef struct LoadAverage_ {
        double one = 0;
        double five = 0;
        double fifteen = 0;
    } LoadAverage;

    typedef struct ProcessCpuInfo_ {
        string pro_name;
        unsigned long long int utime = 0;
        unsigned long long int stime = 0;
        unsigned long long int lasttimes = 0;
        float cpu_percent = 0;
    } ProcessCpuInfo;

    typedef struct MemInfo_ {
        double totalMem;
        double usedMem;
        double MemUsage;
        double totalSwap;
        double usedSwap;
        double SwapUsage;
    } MemInfo;

    typedef struct CpuUseInfo_ {
        int cpu_index;
        double cpu_percent;
    } CpuUseInfo;
    typedef struct NetUseInfo_ {
        string net_name;
        double net_receive_bytes_mb = 0;
        int net_receive_packets = 0;
        int net_receive_packets_errs = 0;
        int net_receive_packets_drop = 0;
        double net_receive_speed = 0;
        double net_transmit_bytes_mb = 0;
        int net_transmit_packets = 0;
        int net_transmit_packets_errs = 0;
        int net_transmit_packets_drop = 0;
        double net_transmit_speed = 0;
    } NetUseInfo;
    typedef struct GpuUseInfo_ {
        unsigned int gpu_index;
        string gpu_name;
        unsigned int gpu_use_percent;
        unsigned int gpu_mem_percent;
        unsigned int gpu_temp;
    } GpuUseInfo;
    typedef struct IpInfo_{
        string ip_address;
        string lidar_name;
        bool is_ip_alive = false;
    }IpInfo;
    typedef struct TopicStateInfo_{
        string topic_name;
        string name;
        double timeout;
        bool enable = false;
        bool is_timeout = false;
        int type;
    }TopicStateInfo;
    typedef struct NodeStateInfo_{
        string node_name;
        bool is_alive = false;
    }NodeStateInfo;
    typedef struct ConfigParameter_{
        string config_file_dir;
        string ip_address_config_file_name;
        string topic_state_config_file_name;
        string node_list_config_file_name;
    }ConfigParameter;
    /********************************************************************/
}

#endif //CTI_MONITOR2_CTI_MONITOR2_DEFINE_H
