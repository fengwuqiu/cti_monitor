//
// Created by wq on 22-4-28.
//
#include <dirent.h>
#include "system_monitor/cti_m2_cpu_manage.h"


CpuManage *CpuManage::m_instance = nullptr;

CpuManage::CpuManage() {
    GetCPUCount();
    m_pro_cpu_info_map[-1].cpu_percent = 0;
}

CpuManage::~CpuManage() {
    delete m_instance;
}

CpuManage *CpuManage::GetInstance() {
    if (m_instance == nullptr) m_instance = new CpuManage;
    return m_instance;
}

ssize_t CpuManage::xread(int fd, void *buf, size_t count) {
    // Read some bytes. Retry on EINTR and when we don't get as many bytes as we requested.
    size_t alreadyRead = 0;
    for (;;) {
        ssize_t res = read(fd, buf, count);
        if (res == -1 && errno == EINTR) continue;
        if (res > 0) {
            buf = ((char *) buf) + res;
            count -= res;
            alreadyRead += res;
        }
        if (res == -1) return -1;
        if (count == 0 || res == 0) return alreadyRead;
    }
}


void CpuManage::RunCpuManage() {
    ScanCPUTime();
    ScanMemoryInfo();
    GetLoadAverage();
    GetCpuTemperature();
    GetStatFsInfo();
}


void CpuManage::GetCPUCount() {
    FILE *file = fopen(PROCSTATFILE, "r");
    if (file == NULL) {
        cout << ("Cannot open "
        PROCSTATFILE) << endl;
    }
    char buffer[PROC_LINE_LENGTH + 1];
    int cpus = -1;
    do {
        cpus++;
        char *s = fgets(buffer, PROC_LINE_LENGTH, file);
        (void) s;
    } while (String_startsWith(buffer, "cpu"));
    fclose(file);

    m_lpl.super.cpuCount = MAX(cpus - 1, 1);

    for (int i = 0; i < cpus; i++) {
        CPUData cpu;
        cpu.totalTime = 1;
        cpu.totalPeriod = 1;
        m_lpl.cpus.push_back(cpu);
    }
}

void CpuManage::ScanCPUTime() {
    FILE *file = fopen(PROCSTATFILE, "r");
    if (file == NULL) {
        cout << ("Cannot open "
        PROCSTATFILE) << endl;
    }
    int cpus = m_lpl.super.cpuCount;
    assert(cpus > 0);
    for (int i = 0; i <= cpus; i++) {
        char buffer[PROC_LINE_LENGTH + 1];
        unsigned long long int usertime, nicetime, systemtime, idletime;
        unsigned long long int ioWait, irq, softIrq, steal, guest, guestnice;
        ioWait = irq = softIrq = steal = guest = guestnice = 0;
        // Depending on your kernel version,
        // 5, 7, 8 or 9 of these fields will be set.
        // The rest will remain at zero.
        char *ok = fgets(buffer, PROC_LINE_LENGTH, file);
        if (!ok) buffer[0] = '\0';
        if (i == 0)
            sscanf(buffer, "cpu  %16llu %16llu %16llu %16llu %16llu %16llu %16llu %16llu %16llu %16llu", &usertime,
                   &nicetime, &systemtime, &idletime, &ioWait, &irq, &softIrq, &steal, &guest, &guestnice);
        else {
            int cpuid;
            sscanf(buffer, "cpu%4d %16llu %16llu %16llu %16llu %16llu %16llu %16llu %16llu %16llu %16llu", &cpuid,
                   &usertime, &nicetime, &systemtime, &idletime, &ioWait, &irq, &softIrq, &steal, &guest, &guestnice);
            assert(cpuid == i - 1);
        }
        // Guest time is already accounted in usertime
        usertime = usertime - guest;
        nicetime = nicetime - guestnice;
        // Fields existing on kernels >= 2.6
        // (and RHEL's patched kernel 2.4...)
        unsigned long long int idlealltime = idletime + ioWait;
        unsigned long long int systemalltime = systemtime + irq + softIrq;
        unsigned long long int virtalltime = guest + guestnice;
        unsigned long long int totaltime = usertime + nicetime + systemalltime + idlealltime + steal + virtalltime;
        CPUData *cpuData = &(m_lpl.cpus[i]);
        // Since we do a subtraction (usertime - guest) and cputime64_to_clock_t()
        // used in /proc/stat rounds down numbers, it can lead to a case where the
        // integer overflow.
#define WRAP_SUBTRACT(a, b) (a > b) ? a - b : 0
//        cpuData->userPeriod = WRAP_SUBTRACT(usertime, cpuData->userTime);
//        cpuData->nicePeriod = WRAP_SUBTRACT(nicetime, cpuData->niceTime);
//        cpuData->systemPeriod = WRAP_SUBTRACT(systemtime, cpuData->systemTime);
//        cpuData->systemAllPeriod = WRAP_SUBTRACT(systemalltime, cpuData->systemAllTime);
//        cpuData->idleAllPeriod = WRAP_SUBTRACT(idlealltime, cpuData->idleAllTime);
        cpuData->idlePeriod = WRAP_SUBTRACT(idletime, cpuData->idleTime);
//        cpuData->ioWaitPeriod = WRAP_SUBTRACT(ioWait, cpuData->ioWaitTime);
//        cpuData->irqPeriod = WRAP_SUBTRACT(irq, cpuData->irqTime);
//        cpuData->softIrqPeriod = WRAP_SUBTRACT(softIrq, cpuData->softIrqTime);
//        cpuData->stealPeriod = WRAP_SUBTRACT(steal, cpuData->stealTime);
//        cpuData->guestPeriod = WRAP_SUBTRACT(virtalltime, cpuData->guestTime);
        cpuData->totalPeriod = WRAP_SUBTRACT(totaltime, cpuData->totalTime);
#undef WRAP_SUBTRACT
//        cpuData->userTime = usertime;
//        cpuData->niceTime = nicetime;
//        cpuData->systemTime = systemtime;
//        cpuData->systemAllTime = systemalltime;
//        cpuData->idleAllTime = idlealltime;
        cpuData->idleTime = idletime;
//        cpuData->ioWaitTime = ioWait;
//        cpuData->irqTime = irq;
//        cpuData->softIrqTime = softIrq;
//        cpuData->stealTime = steal;
//        cpuData->guestTime = virtalltime;
        cpuData->totalTime = totaltime;
    }
    double period = (double) m_lpl.cpus[0].totalPeriod / cpus;
    fclose(file);
    int cpu_c = cpus + 1;
    int count = 0;
    vector <CpuUseInfo> cpu_use_info_vector;
    while (cpu_c--) {
        double result = SetCPUValues(count);
//        cout << "cpu " << count << ":" << result << endl;
        CpuUseInfo cpu_use_info;
        cpu_use_info.cpu_index = count;
        cpu_use_info.cpu_percent = result;
        cpu_use_info_vector.push_back(cpu_use_info);
        count++;
    }
    m_info->SaveCpuUseInfo(cpu_use_info_vector);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    recurseProcTree(PROCDIR, period, tv);
}

void CpuManage::ScanMemoryInfo() {
    unsigned long long int swapFree = 0;
    unsigned long long int shmem = 0;
    unsigned long long int sreclaimable = 0;

    FILE *file = fopen(PROCMEMINFOFILE, "r");
    if (file == NULL) {
        cout << ("Cannot open "
        PROCMEMINFOFILE) << endl;
    }
    char buffer[128];
    while (fgets(buffer, 128, file)) {

#define tryRead(label, variable) (String_startsWith(buffer, label) && sscanf(buffer + strlen(label), " %32llu kB", variable))
        switch (buffer[0]) {
            case 'M':
                if (tryRead("MemTotal:", &m_lpl.super.totalMem)) {}
                else if (tryRead("MemFree:", &m_lpl.super.freeMem)) {}
                else if (tryRead("MemShared:", &m_lpl.super.sharedMem)) {}
                break;
            case 'B':
                if (tryRead("Buffers:", &m_lpl.super.buffersMem)) {}
                break;
            case 'C':
                if (tryRead("Cached:", &m_lpl.super.cachedMem)) {}
                break;
            case 'S':
                switch (buffer[1]) {
                    case 'w':
                        if (tryRead("SwapTotal:", &m_lpl.super.totalSwap)) {}
                        else if (tryRead("SwapFree:", &swapFree)) {}
                        break;
                    case 'h':
                        if (tryRead("Shmem:", &shmem)) {}
                        break;
                    case 'R':
                        if (tryRead("SReclaimable:", &sreclaimable)) {}
                        break;
                }
                break;
        }
#undef tryRead
    }

    m_lpl.super.usedMem = m_lpl.super.totalMem - m_lpl.super.freeMem;
    m_lpl.super.cachedMem = m_lpl.super.cachedMem + sreclaimable - shmem;
    m_lpl.super.usedMem -= m_lpl.super.cachedMem + m_lpl.super.buffersMem;
    m_lpl.super.usedSwap = m_lpl.super.totalSwap - swapFree;
    fclose(file);
//    cout << "usedMem:" << (double) m_lpl.super.usedMem / 1024000 << endl;
//    cout << "totalMem:" << (double) m_lpl.super.totalMem / 1024000 << endl;
//    cout << "usedSwap:" << (double) m_lpl.super.usedSwap / 1024000 << endl;
//    cout << "totalSwap:" << (double) m_lpl.super.totalSwap / 1024000 << endl;
    MemInfo mem_info;
    mem_info.usedMem = (double) m_lpl.super.usedMem / 1024000;
    mem_info.totalMem = (double) m_lpl.super.totalMem / 1024000;
    mem_info.usedSwap = (double) m_lpl.super.usedSwap / 1024000;
    mem_info.totalSwap = (double) m_lpl.super.totalSwap / 1024000;
    mem_info.MemUsage = mem_info.usedMem / mem_info.totalMem * 100;
    mem_info.SwapUsage = mem_info.usedSwap / mem_info.totalSwap * 100;
    m_info->SaveMemInfo(mem_info);
}

void CpuManage::GetLoadAverage() {
    int activeProcs, totalProcs, lastProc;
    FILE *fd = fopen(PROCDIR
    "/loadavg", "r");
    if (fd) {
        int total = fscanf(fd, "%32lf %32lf %32lf %32d/%32d %32d", &m_la.one, &m_la.five, &m_la.fifteen,
                           &activeProcs, &totalProcs, &lastProc);
        (void) total;
        assert(total == 6);
        fclose(fd);
    }
//    cout<<"one :"<<m_la.one<<endl;
//    cout<<"five :"<<m_la.five<<endl;
//    cout<<"fifteen :"<<m_la.fifteen<<endl;
    m_info->SaveLoadAverageInfo(m_la);
}

double CpuManage::SetCPUValues(int cpu) {
    CPUData *cpuData = &(m_lpl.cpus[cpu]);
    double total = (double) (cpuData->totalPeriod == 0 ? 1 : cpuData->totalPeriod);
    double percent;
    percent = 100 - (double) (cpuData->idlePeriod) / total * 100;

    percent = CLAMP(percent, 0.0, 100.0);
    if (isnan(percent)) percent = 0.0;
//    cout << "cpu " << cpu << ":" << percent << endl;
    return percent;
}

bool CpuManage::recurseProcTree(const char *dirname, double period, struct timeval tv) {
    DIR *dir;
    struct dirent *entry;
    time_t curTime = tv.tv_sec;
    dir = opendir(dirname);
    if (!dir) return false;
    int cpus = m_lpl.super.cpuCount;
    bool hideThreads = false;
    InitProCpuFiveArray();
    while ((entry = readdir(dir)) != NULL) {
        char *name = entry->d_name;

        // The RedHat kernel hides threads with a dot.
        // I believe this is non-standard.
        if ((!hideThreads) && name[0] == '.') {
            name++;
        }

        // Just skip all non-number directories.
        if (name[0] < '0' || name[0] > '9') {
            continue;
        }

        // filename is a number: process directory
        int pid = atoi(name);
//        cout << "pid :" << pid << endl;

        char filename[MAX_NAME + 1];
        snprintf(filename, MAX_NAME, "%s/%s/stat", dirname, name);
        if (!ReadStatFile(pid, filename))
            continue;
        ProcessCpuInfo pro_cpu_info = m_pro_cpu_info_map[pid];

        float percent_cpu = (float) (pro_cpu_info.utime + pro_cpu_info.stime - pro_cpu_info.lasttimes) / period * 100.0;
//        cout<<"utime"<<pro_cpu_info.utime<<endl;
//        cout<<"stime"<<pro_cpu_info.stime<<endl;
//        cout<<"lasttimes"<<pro_cpu_info.lasttimes<<endl;
        percent_cpu = CLAMP(percent_cpu, 0.0, cpus * 100.0);
        if (isnan(percent_cpu)) percent_cpu = 0.0;

        m_pro_cpu_info_map[pid].cpu_percent = percent_cpu;
        m_pro_cpu_info_map[pid].lasttimes = pro_cpu_info.utime + pro_cpu_info.stime;
//        cout<<m_pro_cpu_info_map[pid].lasttimes<<endl;
        //将该进程的占用率逐一比对当前存放的前五进程的占用率，并替代掉最低的进程
        float min_cpu_percent = percent_cpu;
        int min_pid = pid;
        int tmp_pid;
        for (auto &i:m_pro_cpu_five){
            if (min_cpu_percent > m_pro_cpu_info_map[i].cpu_percent){
                min_cpu_percent = m_pro_cpu_info_map[i].cpu_percent;
                tmp_pid = i;
                i = min_pid;
                min_pid = tmp_pid;
            }
        }
//        m_pro_cpu_info_map.erase(pid);
    }
    closedir(dir);

    vector <ProcessCpuInfo> pro_cpu_info;
    for (auto i: m_pro_cpu_five) {
//        cout << "pid :" << i << endl;
//        cout << "pid name :" << m_pro_cpu_info_map[i].pro_name << endl;
//        cout << "cpu_percent :" << m_pro_cpu_info_map[i].cpu_percent << endl;
        pro_cpu_info.push_back(m_pro_cpu_info_map[i]);
    }
//    cout << "///////////////////////////////////" << endl;
    m_info->SaveProcessCpuInfo(pro_cpu_info);
    return true;
}

bool CpuManage::ReadStatFile(int pid, const char *filename) {
    ProcessCpuInfo pro_cpu_info;
    int fd = open(filename, O_RDONLY);
    if (fd == -1)
        return false;
    static char buf[MAX_READ + 1];

    int size = xread(fd, buf, MAX_READ);
    close(fd);
    if (size <= 0) return false;
    buf[size] = '\0';

    assert(pid == atoi(buf));
    char *location = strchr(buf, ' ');
    if (!location) return false;

    location += 2;
    char *end = strrchr(location, ')');
    if (!end) return false;

    int commsize = end - location;
    char command[MAX_NAME + 1];
    int commLen = 0;
    memset(command, ' ', sizeof(command));
    memcpy(command, location, commsize);
//    command[commsize] = '\0';
    commLen = commsize;

    string pro_name;
    for (auto i: command)pro_name += i;

    location = end + 2;
//    process->state = location[0];
    location += 2;
    strtol(location, &location, 10);
    location += 1;
    strtoul(location, &location, 10);
    location += 1;
    strtoul(location, &location, 10);
    location += 1;
    strtoul(location, &location, 10);
    location += 1;
    strtol(location, &location, 10);
    location += 1;
    strtoul(location, &location, 10);
    location += 1;
    strtoull(location, &location, 10);
    location += 1;
    strtoull(location, &location, 10);
    location += 1;
    strtoull(location, &location, 10);
    location += 1;
    strtoull(location, &location, 10);
    location += 1;
    pro_cpu_info.utime = AdjustTime(strtoull(location, &location, 10));
//    cout << "utime" << strtoull(location, &location, 10) << endl;
//    cout << "utime" << pro_cpu_info.utime << endl;
    location += 1;
    pro_cpu_info.stime = AdjustTime(strtoull(location, &location, 10));
    location += 1;
    AdjustTime(strtoull(location, &location, 10));
    location += 1;
    AdjustTime(strtoull(location, &location, 10));
    location += 1;
    strtol(location, &location, 10);
    location += 1;
    strtol(location, &location, 10);
    location += 1;
    strtol(location, &location, 10);
    location += 1;
    for (int i = 0; i < 17; i++) location = strchr(location, ' ') + 1;
    strtol(location, &location, 10);
    location += 1;
    assert(location != NULL);
    strtol(location, &location, 10);

    m_pro_cpu_info_map[pid].utime = pro_cpu_info.utime;
    m_pro_cpu_info_map[pid].stime = pro_cpu_info.stime;
    m_pro_cpu_info_map[pid].pro_name = pro_name;


    return true;
}

unsigned long long CpuManage::AdjustTime(unsigned long long int t) {
    if (m_jiffy == 0.0) m_jiffy = sysconf(_SC_CLK_TCK);
    double jiffytime = 1.0 / m_jiffy;
    return (unsigned long long) t * jiffytime * 100;
}

void CpuManage::InitProCpuFiveArray() {
    for (auto &i: m_pro_cpu_five)i = -1;
}

void CpuManage::GetCpuTemperature() {
    if ((access(SYSCLASSTHERMALTEMPDIR, F_OK)) == -1) {
        return;
    }
    FILE *fd = fopen(SYSCLASSTHERMALTEMPDIR, "r");
    if (fd == NULL) {
        cout << ("Cannot open "
        SYSCLASSTHERMALTEMPDIR) << endl;
    }
    char temp_char[10] = {0};
    fgets(temp_char, 10, fd);
    fclose(fd);

    string temp_str;
    for (auto i: temp_char)temp_str += i;
    stringstream ss;
    ss << temp_str;
    float temp = 0.0;
    ss >> temp;
    temp /= 1000.0;
//    cout << "temp :" << temp << endl;
    m_info->SaveCpuTemp(temp);
}

void CpuManage::GetStatFsInfo() {
    if (statfs("/", &m_sf_info) < 0)
        cout << "read / fail" << endl;
    else {
        int sf_use_info = (m_sf_info.f_blocks - m_sf_info.f_bfree) * 100 /
                          (m_sf_info.f_blocks - m_sf_info.f_bfree + m_sf_info.f_bavail) + 1;
        m_info->SaveSfUseInfo(sf_use_info);
    }
}
