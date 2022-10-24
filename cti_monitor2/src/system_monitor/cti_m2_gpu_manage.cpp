//
// Created by wq on 22-5-10.
//
#include "system_monitor/cti_m2_gpu_manage.h"

GpuManage *GpuManage::m_instance = nullptr;

GpuManage::GpuManage() {

}

GpuManage::~GpuManage() {
    delete m_instance;
}

GpuManage *GpuManage::GetInstance() {
    if (m_instance == nullptr) m_instance = new GpuManage;
    return m_instance;
}

void GpuManage::RunGpuManage() {
    if (m_gpu_init_flag)GpuInit();
    else UpdateGpuInfo();
}

void GpuManage::GpuInit() {
    m_result = nvmlInit();

    m_result = nvmlDeviceGetCount(&m_gpu_count);
    if (NVML_SUCCESS != m_result) {
        std::cout << "Failed to query device count: " <<
                  nvmlErrorString(m_result);
        return;
    }
    //add gpu_use_info to vector
    for (unsigned int i = 0; i < m_gpu_count; i++) {
        GpuUseInfo gpu_use_info;
        gpu_use_info.gpu_index = i;
        m_gpu_use_info_vector.push_back(gpu_use_info);
    }
    m_gpu_init_flag = false;
}

void GpuManage::UpdateGpuInfo() {
    m_gpu_index = 0;
    for(auto &gpu_use_info:m_gpu_use_info_vector){
        nvmlDevice_t device;
        char name[NVML_DEVICE_NAME_BUFFER_SIZE];
        memset(name, ' ', sizeof(name));
        m_result = nvmlDeviceGetHandleByIndex(m_gpu_index, &device);
        if (NVML_SUCCESS != m_result) {
            std::cout << "get device failed " <<
                      endl;
        }
        m_result = nvmlDeviceGetName(device, name, NVML_DEVICE_NAME_BUFFER_SIZE);
        if (NVML_SUCCESS != m_result) {
            std::cout << "GPU name： " << name <<
                      endl;
        }
        string gpu_name;
        for (auto name_char: name)gpu_name += name_char;
        gpu_use_info.gpu_name = gpu_name;
//使用率
        nvmlUtilization_t utilization;
        m_result = nvmlDeviceGetUtilizationRates(device, &utilization);
        if (NVML_SUCCESS == m_result) {
            gpu_use_info.gpu_use_percent = utilization.gpu;
            gpu_use_info.gpu_mem_percent = utilization.memory;
        }
        unsigned int temp;
        m_result = nvmlDeviceGetTemperature(device,NVML_TEMPERATURE_GPU,&temp);
        if(NVML_SUCCESS == m_result){
            gpu_use_info.gpu_temp = temp;
        }
    }
    m_info->SaveGpuUseInfo(m_gpu_use_info_vector);
}



