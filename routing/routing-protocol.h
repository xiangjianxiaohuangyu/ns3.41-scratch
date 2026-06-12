#ifndef ROUTING_PROTOCOL_H
#define ROUTING_PROTOCOL_H

/**
 * @file routing-protocol.h
 * @brief ns-3 路由协议仿真主类声明
 *
 * 本文件定义了 RoutingProtocols 类，包含:
 * - ns3 网络仿真配置与执行
 * - TCP 服务端通信功能
 * - 仿真结果收集与存储
 */

#include <string>
#include <vector>
#include <cmath>
#include <sys/stat.h>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>

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
#include "ns3/tcp-socket-factory.h"
#include "ns3/simulator.h"

#include "ns3/gpsr-module.h"
#include "ns3/mmgpsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/olsr-module.h"
#include "ns3/intelligent-protocol-stack-module.h"

// 前向声明
// （原先的 CommNodeInfo / getNodeInformation 已迁移到 IPS 协议模块）

using namespace ns3;

/**
 * @brief 数据包传输信息结构体
 *
 * 用于记录数据包的传输路径和时延信息
 */
struct PacketTransmitInfo{
    std::vector<int> AccrossNode;  ///< 数据包经过的节点列表
    double txTime;                   ///< 数据包发送时间
    double rxTime;                   ///< 数据包接收时间
};

/**
 * @brief 路由协议仿真主类
 *
 * 负责管理整个ns-3网络仿真流程，包括:
 * - 节点创建与移动模型配置
 * - 网络设备与协议栈安装
 * - 应用层通信配置
 * - 与远程服务端的TCP通信
 * - 仿真结果收集与输出
 */
class RoutingProtocols {
public:
    /**
     * @brief 构造函数
     * @details 初始化所有成员变量为默认值
     */
    RoutingProtocols();

    /**
     * @brief 配置仿真参数
     * @param argc 命令行参数个数
     * @param argv 命令行参数数组
     * @return 配置是否成功
     */
    bool Configure(int argc, char **argv);

    /**
     * @brief 启动仿真
     * @details 执行TCP连接、节点创建、仿真运行完整流程
     */
    void Run();

    // ============================================
    // 仿真模块友元声明
    // ============================================
    friend void CreateNodes_Sim(RoutingProtocols* rp);
    friend void InstallMobilityModel_Sim(RoutingProtocols* rp);
    friend void InstallMobilityModel_RandomWalk2dMobilityModel_Sim(RoutingProtocols* rp);
    friend void InstallMobilityModel_RandomWaypointMobilityModel_Sim(RoutingProtocols* rp);
    friend void InstallMobilityModel_GaussMarkovMobilityModel_Sim(RoutingProtocols* rp);
    friend void CreateDevices_Sim(RoutingProtocols* rp);
    friend void InstallEnergyModel_Sim(RoutingProtocols* rp);
    friend void InstallInternetStack_Sim(RoutingProtocols* rp);
    friend void InstallApplications_Sim(RoutingProtocols* rp);
    friend void InitRoutingProtocol_Sim(RoutingProtocols* rp);
    friend void RunAndCal_Sim(RoutingProtocols* rp);
    friend void WriteToFile_Sim(RoutingProtocols* rp, double avgDelay, double deliveryRatio,
                                double avgThroughput, double avgHopCount, double convergenceTime,
                                double flowDelay, uint32_t receivedPackets);
    friend void PrintTransmissionPaths_Sim(RoutingProtocols* rp);
    friend void ApplyIpsWeightsToSourceNodes_Sim(RoutingProtocols* rp);

    // ============================================
    // 通信模块友元声明
    // ============================================
    friend void CreateTcpConnection_Comm(RoutingProtocols* rp);
    friend void InitSocket_Comm(RoutingProtocols* rp);
    friend void ReportTopology_Comm(RoutingProtocols* rp);
    friend void DoUpload_Comm(RoutingProtocols* rp);
    friend void UploadLoop_Comm(RoutingProtocols* rp);
    friend bool SendMessageAndAndAndListen_Comm(RoutingProtocols* rp, const std::string& jsonBody, std::string* responseBody);
    friend std::string SerializeSceneParamsToJson_Comm(const std::string& taskId, uint32_t nodeCount, const std::vector<std::pair<int, int>>& commPairs);
    friend void UploadSceneParams_Comm(RoutingProtocols* rp);
    friend void ApplyNewParametersToSourceNode_Comm(RoutingProtocols* rp, const std::string& responseJson);

private:
    // ============================================
    // 仿真配置参数
    // ============================================
    std::string algorithm;              ///< 路由算法名称 (gpsr/mmgpsr/aodv/dsdv/olsr/intelligent-protocol-stack)
    std::string mobilityModel;           ///< 移动模型名称 (RandomWalk2d/RandomWaypoint/GaussMarkov)
    uint32_t seed;                       ///< 随机数种子，用于实验可重复性
    uint32_t size;                       ///< 网络节点数量
    double step;                         ///< 仿真步长（秒）
    double simScopeXMin;                 ///< 仿真区域X轴最小值（米）
    double simScopeXMax;                 ///< 仿真区域X轴最大值（米）
    double simScopeYMin;                 ///< 仿真区域Y轴最小值（米）
    double simScopeYMax;                 ///< 仿真区域Y轴最大值（米）
    double simScopeZMin;                 ///< 仿真区域Z轴最小值（米）
    double simScopeZMax;                 ///< 仿真区域Z轴最大值（米）
    int nodeSpeedMin;                    ///< 节点移动速度最小值（米/秒）
    int nodeSpeedMax;                    ///< 节点移动速度最大值（米/秒）
    int nodeDirectionMin;               ///< 节点移动方向角度最小值（度，0-360）
    int nodeDirectionMax;               ///< 节点移动方向角度最大值（度，0-360）
    double commRange;                    ///< 节点通信半径（米）
    int gridWidth;                      ///< 网格划分宽度（网格数/行）
    int nodesPairs;                      ///< 同时通信的节点对数
    int totalTime;                      ///< 总仿真时间（秒）
    int port;                            ///< UDP通信端口号
    int packetSizeMax;                   ///< 最大数据包大小（字节）
    uint32_t maxPacketCount;             ///< 包最大传输数量
    double interPacketIntervalTime;      ///< 发送包时间间隔（秒）
    bool pcapEnabled;                    ///< 是否生成PCAP抓包文件
    bool newfile;                        ///< 是否创建新文件
    bool calConvergeTime;                ///< 是否计算收敛时间
    uint32_t pathCount;                 ///< 路径条数参数（用于MPMGPSR多路径）
    bool enableMultiRole;                ///< 是否启用多角色功能
    bool enableTcp;                      ///< 是否开启与服务端的TCP连接；false时直接运行仿真

    // ============================================
    // IPS 路由权重参数
    // ============================================
    double weightDistance;              ///< w1: 距离权重
    double weightLinkTime;              ///< w2: 链路保持时间权重
    double weightRelVelocity;           ///< w3: 相对速度权重
    double weightNeighborCount;         ///< w4: 邻居数量权重

    // ============================================
    // 结果统计相关
    // ============================================
    std::string m_CSVfileName;          ///< 仿真结果文件名
    uint32_t packetsReceived = 0;        ///< 数据包接收副本计数

    // ============================================
    // TCP服务端通信相关
    // ============================================
    std::string m_serverIp;              ///< Ubuntu服务端IP地址
    uint16_t m_serverPort;               ///< Ubuntu服务端端口号
    int m_sockfd;                        ///< TCP socket文件描述符
    bool m_socketConnected;              ///< socket连接状态
    std::string m_taskId;                ///< 仿真任务唯一标识符(UUID)

    // ============================================
    // 拓扑上报相关
    // ============================================
    Ptr<UniformRandomVariable> m_jitterRng;  ///< 用于随机延迟的RNG

    // ============================================
    // 暂停/恢复控制相关
    // ============================================
    bool m_waitingForResponse;           ///< 是否等待服务端响应
    double m_uploadInterval;             ///< 上传间隔（秒）
    bool m_uploadEnabled;                ///< 是否启用上传功能
    std::thread* m_listenerThread;       ///< TCP监听线程指针

    std::mutex m_pauseMutex;             ///< 暂停互斥锁
    std::condition_variable m_pauseCond;  ///< 暂停条件变量

    // ============================================
    // ns-3核心组件
    // ============================================
    NodeContainer nodes;                 ///< 节点容器
    NetDeviceContainer devices;          ///< 网络设备容器
    Ipv4InterfaceContainer interfaces;   ///< IP接口容器
    std::vector<std::pair<int, int>> source_dest_pairs;  ///< 发送节点和接收节点对
    FlowMonitorHelper flowHelper;        ///< FlowMonitor辅助类

    // ============================================
    // 仿真方法（私有，作为备选）
    // ============================================
    void CreateNodes();                                ///< 生成节点并命名
    void InstallMobilityModel();                       ///< 安装移动模型
    void CreateDevices();                              ///< 创建网络设备
    void InstallInternetStack();                      ///< 安装网络协议栈
    void InstallEnergyModel();                         ///< 安装能量模型
    void InstallApplications();                        ///< 安装应用层程序
    void InitRoutingProtocol();                        ///< 初始化路由协议
    void RunAndCal();                                 ///< 运行仿真并计算指标

    void InstallMobilityModel_RandomWalk2dMobilityModel();     ///< RandomWalk2d移动模型
    void InstallMobilityModel_RandomWaypointMobilityModel();    ///< RandomWaypoint移动模型
    void InstallMobilityModel_GaussMarkovMobilityModel();      ///< GaussMarkov移动模型

    // ============================================
    // 通信方法（私有，作为备选）
    // ============================================
    void CreateTcpConnection();              ///< 创建TCP连接
    void InitSocket();                       ///< 初始化Socket
    void ReportTopology();                   ///< 上报拓扑信息（已废弃）
    void UploadLoop();                       ///< 上传数据循环
    void SendMessage(const std::string& msg);///< 发送消息到服务端
    void TriggerPause();                     ///< 触发暂停
    void ResumeSimulation();                 ///< 恢复仿真
    void StartListenerThread();              ///< 启动TCP监听线程
    void ListenerThreadFunc();               ///< TCP监听线程函数
    std::string SerializeNodeInfoToJson();  ///< 序列化节点信息为JSON

    /**
     * @brief 将仿真结果写入文件
     * @param avgDelay 平均时延（ms）
     * @param deliveryRatio 交付率（%）
     * @param avgThroughput 平均吞吐量（kbps）
     * @param avgHopCount 平均跳数
     * @param convergenceTime 收敛时间（ms）
     * @param flowDelay 流量时延（秒）
     * @param receivedPackets 接收数据包数量
     */
    void writeToFile(double avgDelay,double deliveryRatio,double avgThroughput,
                     double avgHopCount,double convergenceTime,double flowDelay,
                     uint32_t receivedPackets);

    /**
     * @brief 仿真时间处理器（静态方法）
     * @param arg0 当前仿真时间（秒）
     */
    static void handler(int arg0){
        std::cout << "The simulation is now at: " << arg0 << " seconds" << std::endl;
    }
};

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char **argv);

#endif // ROUTING_PROTOCOL_H
