//
// Created by wq on 22-5-9.
//
#include "system_monitor/cti_m2_net_manage.h"

NetManage *NetManage::m_instance = nullptr;

NetManage::NetManage() {
}

NetManage::~NetManage() {
    delete m_instance;
}

NetManage *NetManage::GetInstance() {
    if (m_instance == nullptr) m_instance = new NetManage;
    return m_instance;
}

void NetManage::RunNetManage() {
    if (m_net_init_flag)NetInit();
    else GetNetUsage();
}

void NetManage::NetInit() {
    net_line_return = "INIT"; //设置初始值，只要不为空字符串即可
    /*step1: 打开文件 （注意：此处是大坑  原因：每次更新计算机的网络的信息时，必须重新读入，或者重置游标的默认位置）*/

    NetUpdate(m_net_init_flag);//更新计算机的网络的信息

    net_previous_timeStamp = net_current_timeStamp = time(nullptr);//开始获取初始化的时间戳
    net_dif_time = 0;//初始化时间差

    m_net_init_flag = false;
}

void NetManage::NetUpdate(bool init_flag) {
    net_stream = fopen(net_file, "r"); //以R读的方式打开文件再赋给指针fd
    //获取网络的初始化数据
    /*step1: 获取并重置游标位置为文件开头处（注意：此处是大坑  原因：每次更新计算机的网络的信息时，必须重新读入，或者重置游标的默认位置）*/
    net_off = fseek(net_stream, 0,
                    SEEK_SET);//获取并重置游标位置为文件流开头处，SEEK_CUR当前读写位置后增加net_off个位移量,重置游标位置比重新打开并获取文件流的资源要低的道理，应该都懂吧 0.0

    line_num = 1;//重置行数
    net_line_return = fgets(net_buffer, sizeof(net_buffer), net_stream);//读取第一行

    line_num++;
    net_line_return = fgets(net_buffer, sizeof(net_buffer), net_stream);//读取第二行

    net_itemReceive = 0;
    net_itemTransmit = 0;
    int flag = 1;
    while (flag == 1) {
        NetUseInfo net_use_info;
        line_num++;
        net_line_return = fgets(net_buffer, sizeof(net_buffer), net_stream);
        net_itemReceive = 0;
        net_itemTransmit = 0;
        memset(net_tmp_itemName,' ',sizeof(net_tmp_itemName));
        if (net_line_return != NULL) {
            sscanf(net_buffer, "%s%d%d%d%d%d%d%d%d%d%d%d%d",
                   net_tmp_itemName,
                   &net_itemReceive,
                   &net_receive_packets,
                   &net_receive_errs,
                   &net_receive_drop,
                   &net_itemTransmit,
                   &net_itemTransmit,
                   &net_itemTransmit,
                   &net_itemTransmit,
                   &net_itemTransmit,
                   &net_transmit_packets,
                   &net_transmit_errs,
                   &net_transmit_drop);
//            net_receive_mb = ChangeByteToMb(net_itemReceive);
//            net_transmit_mb = ChangeByteToMb(net_itemTransmit);
            net_receive_mb = net_itemReceive;
            net_transmit_mb = net_itemTransmit;
            for (auto i: net_tmp_itemName)net_use_info.net_name += i;
            net_use_info.net_receive_bytes_mb = net_receive_mb;
            net_use_info.net_receive_packets = net_receive_packets;
            net_use_info.net_receive_packets_errs = net_receive_errs;
            net_use_info.net_receive_packets_drop = net_receive_drop;
            net_use_info.net_transmit_bytes_mb = net_transmit_mb;
            net_use_info.net_transmit_packets = net_transmit_packets;
            net_use_info.net_transmit_packets_errs = net_transmit_errs;
            net_use_info.net_transmit_packets_drop = net_transmit_drop;
            if (init_flag)m_net_use_info_vector_p.push_back(net_use_info);
            else m_net_use_info_vector_c.push_back(net_use_info);

//            cout << "net_name :" << net_tmp_itemName << endl;
//            cout << "net_receive_bytes_kb :" << net_itemReceive << endl;
////            cout << "net_receive_speed :" << i.net_receive_speed << endl;
//            cout << "net_receive_packets :" << net_receive_packets << endl;
//            cout << "net_receive_packets_errs :" << net_receive_errs << endl;
//            cout << "net_receive_packets_drop :" << net_receive_drop << endl;
//            cout << "net_transmit_bytes_kb :" << net_itemTransmit << endl;
////            cout << "net_transmit_speed :" << i.net_transmit_speed << endl;
//            cout << "net_transmit_packets :" << net_transmit_packets << endl;
//            cout << "net_transmit_packets_errs :" << net_transmit_errs << endl;
//            cout << "net_transmit_packets_drop :" << net_transmit_drop << endl;
        } else {
            flag = -1;
        }
    }
    fclose(net_stream);
}

void NetManage::GetNetUsage() {
    net_current_timeStamp = time(nullptr);//time() 单位：秒
//    printf("[net_usage] net_current_timeStamp: %f.\n",
//           (double) net_current_timeStamp);//time_t[long int]必须转型,否则无法正常(输出显示:0.000)
    net_dif_time = (double) (net_current_timeStamp - net_previous_timeStamp);//计算时间差（单位：秒）
    if ((net_dif_time) >= NET_DIFF_TIME) {//只有满足达到时间戳以后，才更新接收与发送的网络字节数据信息
        m_net_use_info_vector_c.clear();
        NetUpdate();//是否按行打印更新的网络参数信息
        for (auto &net_use_info_c: m_net_use_info_vector_c) {
            for (auto &net_use_info_p: m_net_use_info_vector_p) {
                if (net_use_info_c.net_name == net_use_info_p.net_name) {
                    net_use_info_c.net_receive_speed =
                            (net_use_info_c.net_receive_bytes_mb - net_use_info_p.net_receive_bytes_mb) / 1024 /
                            net_dif_time;
                    net_use_info_c.net_transmit_speed =
                            (net_use_info_c.net_transmit_bytes_mb - net_use_info_p.net_transmit_bytes_mb) /1024 /
                            net_dif_time;
                    net_use_info_p = net_use_info_c;
                }
            }
        }
        net_previous_timeStamp = net_current_timeStamp;
    }
    m_info->SaveNetUseInfo(m_net_use_info_vector_p);
//    for (auto i: m_net_use_info_vector_p) {
//        cout << "net_name :" << i.net_name << endl;
//        cout << "net_receive_bytes_kb :" << i.net_receive_bytes_kb << endl;
//        cout << "net_receive_speed :" << i.net_receive_speed << endl;
//        cout << "net_receive_packets :" << i.net_receive_packets << endl;
//        cout << "net_receive_packets_errs :" << i.net_receive_packets_errs << endl;
//        cout << "net_receive_packets_drop :" << i.net_receive_packets_drop << endl;
//        cout << "net_transmit_bytes_kb :" << i.net_transmit_bytes_kb << endl;
//        cout << "net_transmit_speed :" << i.net_transmit_speed << endl;
//        cout << "net_transmit_packets :" << i.net_transmit_packets << endl;
//        cout << "net_transmit_packets_errs :" << i.net_transmit_packets_errs << endl;
//        cout << "net_transmit_packets_drop :" << i.net_transmit_packets_drop << endl;
//    }
}

double NetManage::ChangeByteToMb(int byte) {
    if (byte >= 0) return (double) (byte / 1024.0 / 1024.0);
    else
        return (double) ((byte + 2147483647) / 1024.0 / 1024.0) + (double) (2147483647 / 1024.0 / 1024.0);
}
