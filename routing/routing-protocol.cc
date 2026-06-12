#include "routing-protocol.h"
#include "routing-protocol-simulator.h"
#include "routing-protocol-comm.h"
#include "ns3/rng-seed-manager.h"

int main(int argc, char **argv) {
    RoutingProtocols rp;
    if (!rp.Configure(argc, argv))
        NS_FATAL_ERROR("Configuration failed. Aborted.");

    rp.Run();

    return 0;
}

// 构造函数实现：初始化所有成员变量
RoutingProtocols::RoutingProtocols() :
    algorithm("intelligent-protocol-stack"),//intelligent-protocol-stack
    mobilityModel("GaussMarkovMobilityModel"),
    seed(196),
    size(200),
    step(50),
    simScopeXMin(0.0),
    simScopeXMax(2000),
    simScopeYMin(0.0),
    simScopeYMax(2000),
    simScopeZMin(0.0),
    simScopeZMax(0),
    nodeSpeedMin(10),
    nodeSpeedMax(10),
    nodeDirectionMin(0),
    nodeDirectionMax(360),
    commRange(250),
    gridWidth(50),
    nodesPairs(1),
    totalTime(60),
    port(9),
    packetSizeMax(512),
    maxPacketCount(300),
    interPacketIntervalTime(1),
    pcapEnabled(false),
    newfile(true),
    calConvergeTime(true),
    pathCount(1),
    enableMultiRole(true),
    enableTcp(true),//对接win端
    m_serverIp("192.168.159.1"),
    m_serverPort(8000),
    m_sockfd(-1),
    m_socketConnected(false),
    m_taskId(GenerateUUID()),
    m_jitterRng(CreateObject<UniformRandomVariable>()),
    m_waitingForResponse(false),
    m_uploadInterval(10.0),
    m_uploadEnabled(true),
    m_listenerThread(nullptr),
    weightDistance(1),
    weightLinkTime(0),
    weightRelVelocity(0),
    weightNeighborCount(0){}

/*配置：通过控制台赋值*/
bool RoutingProtocols::Configure(int argc, char **argv){
    CommandLine cmd;

    cmd.AddValue("algorithm", "routing algorithm", algorithm);
    cmd.AddValue("mobilityModel", "mobility model", mobilityModel);
    cmd.AddValue("simScopeXMax", "simScopeXMax", simScopeXMax);
    cmd.AddValue("simScopeYMax", "simScopeYMax", simScopeYMax);
    cmd.AddValue("nodeSpeedMin", "nodeSpeedMin", nodeSpeedMin);
    cmd.AddValue("nodeSpeedMax", "nodeSpeedMax", nodeSpeedMax);
    cmd.AddValue("commRange", "commRange", commRange);
    cmd.AddValue("size", "Number of nodes.", size);
    cmd.AddValue("seed", "seed value", seed);
    cmd.AddValue("pathCount", "Number of paths for MPMRGPSR (1-7)", pathCount);
    cmd.AddValue("enableMultiRole", "Enable multi-role feature for MPMRGPSR", enableMultiRole);
    cmd.AddValue("serverIp", "Ubuntu server IP address for TCP connection", m_serverIp);
    cmd.AddValue("serverPort", "Ubuntu server port for TCP connection", m_serverPort);
    cmd.AddValue("enableTcp", "Whether to open TCP connection to server (true) or run simulation directly (false)", enableTcp);
    cmd.AddValue("weightDistance", "IPS weight w1 for distance (0-1)", weightDistance);
    cmd.AddValue("weightLinkTime", "IPS weight w2 for link holding time (0-1)", weightLinkTime);
    cmd.AddValue("weightRelVelocity", "IPS weight w3 for relative velocity (0-1)", weightRelVelocity);
    cmd.AddValue("weightNeighborCount", "IPS weight w4 for neighbor count (0-1)", weightNeighborCount);

    cmd.Parse(argc, argv);
    return true;
}

/*启动仿真*/
void RoutingProtocols::Run(){
    if (enableTcp) {
        std::cout << "[TCP] Connecting to server " << m_serverIp << ":" << m_serverPort << "..." << std::endl;

        m_sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (m_sockfd < 0) {
            std::cerr << "[TCP] Socket creation failed!" << std::endl;
            return;
        }

        sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(m_serverPort);

        if (inet_pton(AF_INET, m_serverIp.c_str(), &serv_addr.sin_addr) <= 0) {
            std::cerr << "[TCP] Invalid address!" << std::endl;
            close(m_sockfd);
            m_sockfd = -1;
            return;
        }

        if (connect(m_sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "[TCP] Connection failed! Server may not be running." << std::endl;
            close(m_sockfd);
            m_sockfd = -1;
            return;
        }

        m_socketConnected = true;
        std::cout << "[TCP] Connected to server successfully!" << std::endl;
    } else {
        std::cout << "[TCP] enableTcp=false, skipping TCP connection and running simulation directly." << std::endl;
    }

    RngSeedManager::SetSeed(seed);
    CreateNodes_Sim(this);
    InstallMobilityModel_Sim(this);
    CreateDevices_Sim(this);
    InstallInternetStack_Sim(this);
    InstallEnergyModel_Sim(this);
    InstallApplications_Sim(this);
    InitRoutingProtocol_Sim(this);

    // 按 m_uploadInterval 周期性地把源节点信息（场景/参数/结果快照）序列化为
    // JSON 通过 TCP POST 上传到 Windows 端，并同步阻塞等待 Windows 端返回
    // 的 result 后再继续仿真。每次上传结束 UploadLoop_Comm 内部会自行按
    // m_uploadInterval 重新调度自身，从而形成周期循环。
    if (enableTcp && m_uploadEnabled)
    {
        std::cout << "[Upload] 启用周期性上报，间隔 = " << m_uploadInterval
                  << " s，首次上报将在仿真时间 " << m_uploadInterval
                  << " s 触发 (taskId=" << m_taskId << ")" << std::endl;
        Simulator::Schedule(Seconds(m_uploadInterval), &UploadLoop_Comm, this);
    }

    RunAndCal_Sim(this);
}
