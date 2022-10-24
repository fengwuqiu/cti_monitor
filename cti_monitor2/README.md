安全系统监测模块功能代码 ：

文件结构：
include :
  software_monitor:                 //软件管理分类文件夹
    cti_m2_node_manage.h            //节点管理类头文件
    cti_m2_topic_manage.h           //话题管理类头文件
  system_monitor:                   //系统管理分类文件夹
    cti_m2_cpu_manage.h             //cpu管理类头文件
    cti_m2_gpu_manage.h             //gpu管理类头文件
    cti_m2_ip_manage.h              //ip管理类头文件
    cti_m2_net_manage.h             //网络管理类头文件
  cti_monitor2_define.h             //项目主要定义声明的头文件
  cti_monitor2_info.h               //数据信息类头文件
  cti_monitor2_node.h               //节点功能类头文件
src :
  software_monitor:
    cti_m2_node_manage.cpp          //节点管理类：根据yaml中配置的节点列表进行对这些节点运行状态的监测
    cti_m2_topic_manage.cpp         //话题管理类：根据yaml中配置的话题列表进行对这些话题超时状态的监测
  system_monitor:
    cti_m2_cpu_manage.cpp           //cpu管理类：根据htop源码编写的，监测所有cpu的占用情况，内存占用情况，平均负载情况，获取cpu占用率前五的进程，获取cpu温度。
    cti_m2_gpu_manage.cpp           //gpu管理类：调用nvml库实现，可以获取gpu的温度，内存占用情况，显存占用情况。
    cti_m2_ip_manage.cpp            //ip管理类：根据yaml中配置的ip列表对这些ip进行连接状态的监测
    cti_m2_net_manage.cpp           //网络管理类：读取系统文件，进行系统网卡的数据监测。
  cti_monitor2_info.cpp             //数据信息类：统一管理项目中需要用到的各种公共数据，以供所有类取用。
  cti_monitor2_main.cpp             //主函数
  cti_monitor2_node.cpp             //节点功能类：持有所有的发布和订阅者，统一管理话题的发布和订阅
config :
  v7.0:
    ip_address_config.yaml          //需要监测的ip地址列表
    node_list.yaml                  //需要监测的节点列表
    topic_state.yaml                //需要监测的话题列表
launch :
  cti_monitor2_v7.launch.py       //运行用的launch文件
  
  备注事项：
  1. cpu温度获取目前是获取的/sys/class/thermal/thermal_zone0/temp中的数据，工作用的电脑笔记本没有异常，但是在车的工控机的系统里获取的数据不合适，后续可以直接获取/sys/devices/platform/coretemp.0/hwmon/hwmonX/temp1_input中的数据，但由于hwmonX中X的数值每次开机都会变化，所以需要先在前一个hwmon目录下先获取当前目录的文件夹名称再补全整个文件路径去获取温度数据。
  2. 目前网络信息管理类监测的是整个网口的流量数据，不适用于具体监测外网流量数据的需求，所以可以参考iftop源码实现监测该网口下所有ip的流量的功能，然后取这些ip中属于外网的部分来达成需求。

