#include <string>
#include <cmath>
#include <sys/stat.h>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/udp-echo-server.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/energy-module.h"
#include "ns3/simulator.h"

#include "ns3/gpsr-module.h"
#include "ns3/mmgpsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/olsr-module.h"
#include "ns3/intelligent-protocol-stack-module.h"
#include "ns3/node-list.h"                       // InitRoutingProtocol 中按 id 取节点

using namespace ns3;

struct PacketTransmitInfo{
    std::vector<int> AccrossNode;
    double txTime;
    double rxTime;
};

class RoutingProtocols {
public:
    RoutingProtocols();  // 构造函数声明

    bool Configure(int argc, char **argv);  //配置：通过控制台赋值
    void Run(); //启动仿真

private:
    std::string algorithm;  // 使用的路由算法名称
    std::string mobilityModel;  // 使用的移动模型
    uint32_t seed;          // 随机数生成器种子，用于实验可重复性
    uint32_t size;          // 网络规模参数（如节点总数）
    double step;            // 仿真步长（单位：秒）
    double simScopeXMin;    // 仿真区域X轴最小值（单位：米）
    double simScopeXMax;    // 仿真区域X轴最大值（单位：米）
    double simScopeYMin;    // 仿真区域Y轴最小值（单位：米）
    double simScopeYMax;    // 仿真区域Y轴最大值（单位：米）
    double simScopeZMin;    // 仿真区域Z轴最小值（单位：米）
    double simScopeZMax;    // 仿真区域Z轴最大值（单位：米）
    int nodeSpeedMin;       // 节点移动速度最小值（单位：米/秒）
    int nodeSpeedMax;       // 节点移动速度最大值（单位：米/秒）
    int nodeDirectionMin;   // 节点移动方向角度最小值（单位：度，0-360）
    int nodeDirectionMax;   // 节点移动方向角度最大值（单位：度，0-360）
    double commRange;       // 节点通信半径（单位：米）
    int gridWidth;          // 网格划分宽度（单位：网格数/行
    int nodesPairs;         // 同时通信的节点对数
    int totalTime;          // 总仿真时间（单位：秒）
    int port;               // UDP通信端口号
    int packetSizeMax;      // 最大数据包大小（单位：字节）
    uint32_t maxPacketCount;          // 包最大传输数量
    double interPacketIntervalTime;     //发送包时间间隔
    bool pcapEnabled;       // 是否生成PCAP抓包文件（true/false）
    bool newfile;
    bool calConvergeTime; //是否计算收敛时间
    uint32_t pathCount; // 路径条数参数（用于MPMGPSR多路径）
    bool enableMultiRole; // 是否启用多角色功能

    std::string m_CSVfileName; // 仿真随时间变化文本
    uint32_t packetsReceived = 0; // 数据包接收副本，用于随时间变化的情况。

    // 报告拓扑相关的成员变量
    Ptr<UniformRandomVariable> m_jitterRng;  // 用于随机延迟的RNG

    NodeContainer nodes;//节点容器
    NetDeviceContainer devices;//设备管理容器
    Ipv4InterfaceContainer interfaces;//接口
    std::vector<std::pair<int, int>> source_dest_pairs;//发送节点和接受节点对

    FlowMonitorHelper flowHelper;// 添加FlowMonitorHelper作为成员变量

    void CreateNodes();             //生成节点并命名
    void InstallMobilityModel();
    void CreateDevices();           //网络设备：wifi设备、物理层、信道、mac层
    void InstallInternetStack();    //网络协议栈：安装路由算法至节点
    void InstallEnergyModel();      //安装能量模型
    void InstallApplications();     //应用：节点通信
    void InitRoutingProtocol();     //初始化路由协议
    void RunAndCal();               //开始仿真并统计指标

    void InstallMobilityModel_RandomWalk2dMobilityModel();
    void InstallMobilityModel_RandomWaypointMobilityModel();
    void InstallMobilityModel_GaussMarkovMobilityModel();

    void writeToFile(double avgDelay,double deliveryRatio,double avgThroughput,double avgHopCount,double convergenceTime,double flowDelay,uint32_t receivedPackets);
    static void handler(int arg0){
        std::cout << "The simulation is now at: " << arg0 << " seconds" << std::endl;
    }
};

int main(int argc, char **argv) {
    RoutingProtocols rp;
    if (!rp.Configure(argc, argv))
        NS_FATAL_ERROR("Configuration failed. Aborted.");

    rp.Run();

    return 0;
}

// 构造函数实现：初始化所有成员变量
RoutingProtocols::RoutingProtocols() :
    algorithm("ips"), // intelligent-protocol-stack||gpsr||mmgpsr||aodv||dsdv||olsr||
    mobilityModel("GaussMarkovMobilityModel"),//RandomWalk2dMobilityModel||RandomWaypointMobilityModel||GaussMarkovMobilityModel
    seed(272),
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
    nodesPairs(1),//result统计只支持一对通信节点对,收敛时间同理
    totalTime(10),
    port(9),
    packetSizeMax(512),
    maxPacketCount(300),
    interPacketIntervalTime(1),
    pcapEnabled(false),
    newfile(true),
    calConvergeTime(true),
    pathCount(1),
    enableMultiRole(true),
    m_jitterRng(CreateObject<UniformRandomVariable>()){}

/*配置：通过控制台赋值*/
bool RoutingProtocols::Configure(int argc, char **argv){
    //LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_ALL);

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

    cmd.Parse(argc, argv);

    // 把 --seed 同步到 ns-3 全局 RNG：影响 RandomBoxPositionAllocator / 移动模型 / 高斯扰动 等所有 UniformRandomVariable
    RngSeedManager::SetSeed(seed);

    return true;
}

/*启动仿真*/
void RoutingProtocols::Run(){
    CreateNodes();
    InstallMobilityModel();
    CreateDevices();
    InstallInternetStack();
    InstallEnergyModel();
    InstallApplications();
    InitRoutingProtocol();

    RunAndCal();
}

/*生成节点并命名*/
void RoutingProtocols::CreateNodes(){
    std::cout<<"Size : "<<size<<std::endl;
    nodes.Create(size);

    //节点命名，例如"/Names/node-1"
    for (uint32_t i = 0; i < size; ++i){
        std::ostringstream os;
        os << "node-" << i;
        Names::Add(os.str(), nodes.Get(i));
    }
}

void RoutingProtocols::InstallMobilityModel(){
    if(mobilityModel == "RandomWalk2dMobilityModel"){
        std::cout<<"Mobility model : RandomWalk2dMobilityModel"<<std::endl;
        InstallMobilityModel_RandomWalk2dMobilityModel();
    }else if(mobilityModel == "RandomWaypointMobilityModel"){
        std::cout<<"Mobility model : RandomWaypointMobilityModel"<<std::endl;
        InstallMobilityModel_RandomWaypointMobilityModel();
    }else if(mobilityModel == "GaussMarkovMobilityModel"){
        std::cout<<"Mobility model : GaussMarkovMobilityModel"<<std::endl;
        InstallMobilityModel_GaussMarkovMobilityModel();
    }else{
        std::cout << "Install mobility model failed !"<<std::endl;
    }

    std::cout<<"Node speed : "<<nodeSpeedMin<<"--"<<nodeSpeedMax<<std::endl;
}

void RoutingProtocols::InstallMobilityModel_RandomWalk2dMobilityModel(){
    // 设置每个节点的随机移动模型
    MobilityHelper mobility;

    /*2D移动模型*/
    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                            "X", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeXMin) + "|Max=" + std::to_string(simScopeXMax) + "]"),
                            "Y", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeYMin) + "|Max=" + std::to_string(simScopeYMax) + "]"));

    // 设置随机漫步模型，限制节点的移动范围 //
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            "Bounds", RectangleValue(Rectangle(simScopeXMin, simScopeXMax, simScopeYMin, simScopeYMax)),
                            "Speed", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(nodeSpeedMin) + "|Max=" + std::to_string(nodeSpeedMax) + "]"),
                            "Direction", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(nodeDirectionMin) + "|Max=" + std::to_string(nodeDirectionMax) + "]"));

    // 安装移动模型到节点
    mobility.Install(nodes);
}

void RoutingProtocols::InstallMobilityModel_RandomWaypointMobilityModel(){
    // 创建并配置 PositionAllocator
    Ptr<PositionAllocator> positionAlloc = CreateObject<RandomBoxPositionAllocator>();
    positionAlloc->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeXMin) + "|Max=" + std::to_string(simScopeXMax) + "]"));
    positionAlloc->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeYMin) + "|Max=" + std::to_string(simScopeYMax) + "]"));
    positionAlloc->SetAttribute("Z", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeZMin) + "|Max=" + std::to_string(simScopeZMax) + "]"));

    // 设置移动模型
    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                "Speed", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(nodeSpeedMin) + "|Max=" + std::to_string(nodeSpeedMax) + "]"),
                                "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                                "PositionAllocator", PointerValue(positionAlloc));

    // 安装移动模型
    mobility.Install(nodes);

    NS_LOG_UNCOND("simScope: "<< simScopeXMax<<" X "<<simScopeYMax<<" X "<<simScopeZMax);
}

void RoutingProtocols::InstallMobilityModel_GaussMarkovMobilityModel(){
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel", "Bounds",
                              BoxValue(Box(simScopeXMin, simScopeXMax, simScopeYMin,simScopeYMax,simScopeZMin,simScopeZMax)), // 节点运动的区域。通常使用BoxValue类定义区域大小。
                              "TimeStep", TimeValue(Seconds(1)),       // 节点通常设置TimeStep，移动相应长时间后更改当前方向和速度。
                              "Alpha", DoubleValue(0.85),              // 表示高斯-马尔可夫模型中可调参数的常数。初始默认值为1.
                              "MeanVelocity",
                              StringValue("ns3::ConstantRandomVariable[Constant="+std::to_string(nodeSpeedMax)+"]"), // 用于指定平均速度的随机变量。
                              "MeanDirection",
                              StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"), // 用于指定平均方向的随机变量。
                              "MeanPitch",
                              StringValue("ns3::UniformRandomVariable[Min=0.05|Max=0.05]"), // 平均角度：用于指定平均角度的随机变量。
                              "NormalVelocity",
                              StringValue(
                                  "ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
                              // 高斯随机速度：用于计算下一个速度值的高斯随机变量。
                              "NormalDirection",
                              StringValue(
                                  "ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
                              // 高斯随机方向：用于计算下一个方向值的高斯随机变量。
                              "NormalPitch",
                              StringValue(
                                  "ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]"));
    // 高斯随机夹角度：用于计算下一个夹角度的高斯随机变量。
    mobility.SetPositionAllocator(
        "ns3::RandomBoxPositionAllocator", // 随机位置分配器
        "X", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeXMin) + "|Max=" + std::to_string(simScopeXMax) + "]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeYMin) + "|Max=" + std::to_string(simScopeYMax) + "]"),
        "Z", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(simScopeZMin) + "|Max=" + std::to_string(simScopeZMax) + "]"));
    mobility.Install(nodes); // 安装移动模型
}


/*网络设备：wifi设备、物理层、信道、mac层*/
void RoutingProtocols::CreateDevices(){
    std::cout<<"commonrange : "<<commRange<<std::endl;

    WifiHelper wifi;//Wi-Fi设备
    YansWifiPhyHelper wifiPhy;//Wi-Fi物理层（PHY层）
    YansWifiChannelHelper wifiChannel;//Wi-Fi信道
    WifiMacHelper wifiMac;//Wi-Fi的MAC层

    /* 设置接受增益 */
    wifiPhy.Set("RxGain", DoubleValue(-10));
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    /* 设置传播时延 */
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    /* 设置路径损失和最大传输距离 */
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(commRange));
    wifiPhy.SetChannel(wifiChannel.Create());

    /* 设置wifi标准 */
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("OfdmRate6Mbps"),
                                 "RtsCtsThreshold", UintegerValue(packetSizeMax));

    /* 设置自组网物理地址模型 */
    wifiMac.SetType("ns3::AdhocWifiMac","QosSupported", BooleanValue(false));

    /* 安装 */
    devices = wifi.Install(wifiPhy, wifiMac, nodes);
}

/*能量模型安装*/
void RoutingProtocols::InstallEnergyModel(){
    //安装能量模型，初始能量设置为100焦耳
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100.0));
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);
    //配置WIFI设备的发射、接收等能量消耗参数
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.067));
    radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.046));
    radioEnergyHelper.Set ("IdleCurrentA", DoubleValue(0.01));
    radioEnergyHelper.Set ("CcaBusyCurrentA", DoubleValue(0.01));
    radioEnergyHelper.Set ("SwitchingCurrentA", DoubleValue(0.01));
    radioEnergyHelper.Set ("SleepCurrentA", DoubleValue(0.001));
    //将能量模型绑定到具体的无线设备
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
}

/*网络协议栈：安装路由算法至节点*/
void RoutingProtocols::InstallInternetStack(){
    InternetStackHelper stack;
    Ipv4AddressHelper address;

    /*安装路由协议*/
    if(algorithm == "mmgpsr"){
        std::cout<<"Routing algorithm : MMGPSR"<<std::endl;
        MMGpsrHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(nodes);
    }else if(algorithm == "gpsr"){
        std::cout<<"Routing algorithm : GPSR"<<std::endl;
        GpsrHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(nodes);
    }else if(algorithm == "aodv"){
        std::cout<<"Routing algorithm : AODV"<<std::endl;
        AodvHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(nodes);
    }else if(algorithm == "dsdv"){
        std::cout<<"Routing algorithm : DSDV"<<std::endl;
        DsdvHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(nodes);
    }else if(algorithm == "olsr"){
        std::cout<<"Routing algorithm : OLSR"<<std::endl;
        OlsrHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(nodes);
    }else if(algorithm == "intelligent-protocol-stack" || algorithm == "ips"){
        std::cout<<"Routing algorithm : IntelligentProtocolStack"<<std::endl;
        IntelligentProtocolStackHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(nodes);
    }

    // WRMGpsrHelper wrmgpsr;
    // stack.SetRoutingHelper(wrmgpsr);
    // stack.Install(nodes);

    /*设置ip地址*/
    address.SetBase("10.0.0.0", "255.255.0.0");
    interfaces = address.Assign(devices);
}

/*应用：节点通信*/
void RoutingProtocols::InstallApplications(){
    std::cout<<"Seed : "<<seed<<std::endl;

    Time interPacketInterval = Seconds(interPacketIntervalTime);
    UdpEchoServerHelper server1(port);
    ApplicationContainer apps;
    srand(seed);
    int source;
    int dest;
    std::vector<std::pair<int, int>>::iterator it;
    bool exist;
    float start_app;

    for (int i = 0; i < nodesPairs; i++){
        source = 0;
        dest = 0;
        exist = true;
        start_app = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        while ((source == dest) || exist == true){
            source = rand() % size;
            dest = rand() % size;
            it = std::find(source_dest_pairs.begin(), source_dest_pairs.end(), std::make_pair(source, dest));

            if (it != source_dest_pairs.end()){
                exist = true;
            }else{
                source_dest_pairs.push_back(std::make_pair(source, dest));
                exist = false;
            }
        }

        std::cout<<"Communication nodes: "<<source<<"-->"<<dest<<std::endl;

        // 获取目标节点的 IP 地址
        Ipv4Address destAddress = interfaces.GetAddress(dest);

        UdpEchoClientHelper client(destAddress, port);
        client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
        client.SetAttribute("Interval", TimeValue(interPacketInterval));
        client.SetAttribute("PacketSize", UintegerValue(packetSizeMax));

        apps = client.Install(nodes.Get(source));
        apps.Start(Seconds(start_app));
        apps.Stop(Seconds(totalTime - 0.1));
    }

    intelligentprotocolstack::GlobalInformation::GetInstance().setDstNode(dest);
    intelligentprotocolstack::GlobalInformation::GetInstance().setSourceNode(source);
    intelligentprotocolstack::GlobalInformation::GetInstance().setIdentity(source,1);
}

/*初始化路由协议：仅gpsr需要*/
void RoutingProtocols::InitRoutingProtocol(){
    if(algorithm == "mmgpsr"){
        MMGpsrHelper route;
        route.Install();
    }else if(algorithm == "gpsr"){
        GpsrHelper route;
        route.Install();
    }else if(algorithm == "intelligent-protocol-stack" || algorithm == "ips"){
        IntelligentProtocolStackHelper route;
        route.Install();
    }
}

void RoutingProtocols::RunAndCal(){
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    AnimationInterface anim("gpsr_anim.xml");
    anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(totalTime));
    m_CSVfileName = algorithm + ".csv";
    AsciiTraceHelper ascii;
    MobilityHelper::EnableAscii(ascii.CreateFileStream("Fanet_Routing_Comparison.mob"), 2);
    Simulator::Stop(Seconds(totalTime));
    Simulator::Run();


    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    int j = 0;
    float AvgThroughput = 0;
    double Delay = 0;
    double hopCount = 0;
    double convergenceTime = 0; // 收敛时间变量
    bool convergeFlag = true; // 收敛标志
    double flow_delay=0;//包含重传的平均时延
    double test_TotalThroughput = 0;

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter =stats.begin();iter != stats.end(); ++iter){
        SentPackets += (iter->second.txPackets);
        ReceivedPackets += (iter->second.rxPackets);
        AvgThroughput += (iter->second.rxBytes * 8.0 / (iter->second.timeFirstRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1024);

        if (iter->second.rxPackets != 0){
            j = j + 1;
            test_TotalThroughput+=(iter->second.txBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1024);
            Delay += (iter->second.delaySum.GetSeconds()/iter->second.rxPackets);
            hopCount += (iter->second.timesForwarded / iter->second.rxPackets + 1); // 这是是每个数据包平均跳数
            flow_delay+=(iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds());
            if(calConvergeTime&&convergeFlag&&iter->second.timeFirstTxPacket.GetSeconds()>=30){
                convergenceTime = (iter->second.timeFirstRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds())*1000;//ms
                convergeFlag = false;
            }
        }
    }

    // 计算平均值
    double avgDelay = (j > 0) ? (Delay / j) * 1000 : 0;
    double avgHopCount = (j > 0) ? (hopCount / j) : 0;
    double avgThroughput = (j > 0) ? (AvgThroughput / j) : 0;
    double deliveryRatio = (SentPackets > 0) ? ((ReceivedPackets * 100.0) / SentPackets) : 0;

    NS_LOG_UNCOND("--------FlownMonitor Results of the simulation----------" << std::endl);
    NS_LOG_UNCOND("Total Sent Packets  = " << SentPackets);
    NS_LOG_UNCOND("Total Received Packets = " << ReceivedPackets);
    NS_LOG_UNCOND("Flown Number = " << j);
    NS_LOG_UNCOND("Packet Delivery Ratio = " << std::fixed << std::setprecision(3) << deliveryRatio << "%");
    NS_LOG_UNCOND("Average End To End Delay = " << std::fixed << std::setprecision(3) << (avgDelay) << " ms");
    NS_LOG_UNCOND("Average Throughput = " << std::fixed << std::setprecision(3) << avgThroughput << " Kbps");
    NS_LOG_UNCOND("Average Hop Count = " << std::fixed << std::setprecision(3) << avgHopCount);
    if(calConvergeTime){
        NS_LOG_UNCOND("Convergence Time = " << std::fixed << std::setprecision(3) << convergenceTime << " ms");
    }
    NS_LOG_UNCOND("flow delay = "<<flow_delay << std::endl);
    NS_LOG_UNCOND("flow delay/ReceivedPackets = "<<flow_delay/ReceivedPackets << std::endl);

    writeToFile(avgDelay,deliveryRatio,avgThroughput,avgHopCount,convergenceTime,flow_delay,ReceivedPackets);

    monitor->SerializeToXmlFile("fanet-routing-gpsr.xml", true, true);

    // 在 Simulator::Destroy() 之前输出每个 IPS 节点的 ID / IP / 身份，
    // 否则析构时如果 teardown 出现空指针断言，这里的打印就看不到了。
    if (algorithm == "intelligent-protocol-stack" || algorithm == "ips") {
        intelligentprotocolstack::GlobalInformation::GetInstance().print_node_identity();
        intelligentprotocolstack::GlobalInformation::GetInstance().print_path();
    }

    Simulator::Destroy();
}

void RoutingProtocols::writeToFile(double avgDelay,double deliveryRatio,double avgThroughput,double avgHopCount,double convergenceTime,double flowDelay,uint32_t receivedPackets){
    struct stat info;
    std::string folderPath = "RoutingResult";  // 文件夹路径
    std::string fileName = algorithm + "_speed" + std::to_string(nodeSpeedMin) + "-" + std::to_string(nodeSpeedMax) + "_results.txt";
    if (stat(folderPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        // 文件夹不存在，创建它
        if (mkdir(folderPath.c_str(), 0777) != 0) {
            return;
        }
    }

    // 合并文件夹路径和文件名
    std::string filePath = folderPath + "/" + fileName;

    // 打开文件准备写入
    std::ofstream outFile(filePath, std::ios::app);

    // 打开文件准备写入
    if (!outFile.is_open()){
        std::cerr << "Error opening results file!" << std::endl;
        return;
    }

    // 如果是空文件，先写入标题行
    outFile.seekp(0, std::ios::end);
    // 标题行
    if(outFile.tellp() == 0) {
    outFile << "seed\tdelay(ms)\tdelivertRatio(%)\tthroughput(kbps)\tavgHopCount\tconvergenceTime\tflowDelay\n";
    }
    // 数据行
    double flowDelayAvg = (receivedPackets > 0) ? (flowDelay / receivedPackets * 1000) : 0; // 转换为ms平均值
    outFile <<seed<<"\t"
    << avgDelay<< "\t"
        << deliveryRatio << "\t"
        << avgThroughput << "\t"
        << avgHopCount << "\t"
        << convergenceTime << "\t"
        << flowDelayAvg << "\n";
    // 关闭文件
    outFile.close();
}
