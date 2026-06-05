#include "routing-protocol-comm.h"
#include <random>
#include <sstream>

/*生成UUID v4格式的字符串*/
std::string GenerateUUID()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 15);
    std::uniform_int_distribution<> dis2(8, 11);

    std::ostringstream oss;
    oss << std::hex;
    for (int i = 0; i < 8; i++) oss << dis(gen);
    oss << "-";
    for (int i = 0; i < 4; i++) oss << dis(gen);
    oss << "-4";
    for (int i = 0; i < 3; i++) oss << dis(gen);
    oss << "-";
    oss << dis2(gen);
    for (int i = 0; i < 3; i++) oss << dis(gen);
    oss << "-";
    for (int i = 0; i < 12; i++) oss << dis(gen);
    return oss.str();
}

/*创建TCP连接到Ubuntu服务端*/
void CreateTcpConnection_Comm(RoutingProtocols* rp)
{
    std::cout << "[TCP] Creating socket connection to " << rp->m_serverIp << ":" << rp->m_serverPort << std::endl;

    rp->m_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (rp->m_sockfd < 0) {
        std::cerr << "[TCP] Socket creation failed!" << std::endl;
        return;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(rp->m_serverPort);

    if (inet_pton(AF_INET, rp->m_serverIp.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "[TCP] Invalid address!" << std::endl;
        close(rp->m_sockfd);
        rp->m_sockfd = -1;
        return;
    }

    if (connect(rp->m_sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "[TCP] Connection failed!" << std::endl;
        close(rp->m_sockfd);
        rp->m_sockfd = -1;
        return;
    }

    rp->m_socketConnected = true;
    std::cout << "[TCP] Connection established successfully!" << std::endl;
}

/*初始化Socket连接*/
void InitSocket_Comm(RoutingProtocols* rp)
{
    CreateTcpConnection_Comm(rp);
}

/*获取节点的位置和速度信息*/
NodePositionInfo GetNodePositionVelocity(Ptr<Node> node)
{
    NodePositionInfo info;
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    if (mobility)
    {
        Vector pos = mobility->GetPosition();
        Vector vel = mobility->GetVelocity();
        info.positionX = pos.x;
        info.positionY = pos.y;
        info.positionZ = pos.z;
        info.velocityX = vel.x;
        info.velocityY = vel.y;
        info.velocityZ = vel.z;
    }
    return info;
}

/*获取节点的剩余能量百分比*/
float GetNodeEnergyPercentage(Ptr<Node> node)
{
    Ptr<EnergySourceContainer> energySourceContainer = node->GetObject<EnergySourceContainer>();
    if (energySourceContainer && energySourceContainer->GetN() > 0)
    {
        Ptr<EnergySource> energySource = energySourceContainer->Get(0);
        float remaining = energySource->GetRemainingEnergy();
        float initial = energySource->GetInitialEnergy();
        return (initial > 0) ? (remaining / initial * 100.0f) : 0.0f;
    }
    return 0.0f;
}

/*获取节点的Hello包发送间隔*/
double GetNodeHelloInterval(Ptr<Node> node)
{
    Ptr<Ipv4RoutingProtocol> routingProtocol = node->GetObject<Ipv4>()->GetRoutingProtocol();
    Ptr<ns3::intelligentprotocolstack::RoutingProtocol> ipsRouting =
        DynamicCast<ns3::intelligentprotocolstack::RoutingProtocol>(routingProtocol);
    if (ipsRouting)
    {
        return ipsRouting->HelloInterval.GetSeconds();
    }
    return 0;
}

/*获取节点选择下一跳时的四个权重参数*/
NodeWeightParams GetNodeWeights(Ptr<Node> node)
{
    NodeWeightParams params;
    params.weightDistance = 0.25f;
    params.weightLinkTime = 0.25f;
    params.weightRelVelocity = 0.25f;
    params.weightNeighborCount = 0.25f;

    Ptr<Ipv4RoutingProtocol> routingProtocol = node->GetObject<Ipv4>()->GetRoutingProtocol();
    Ptr<ns3::intelligentprotocolstack::RoutingProtocol> ipsRouting =
        DynamicCast<ns3::intelligentprotocolstack::RoutingProtocol>(routingProtocol);
    if (ipsRouting)
    {
        params.weightDistance = static_cast<float>(ipsRouting->GetWeightDistance());
        params.weightLinkTime = static_cast<float>(ipsRouting->GetWeightLinkTime());
        params.weightRelVelocity = static_cast<float>(ipsRouting->GetWeightRelVelocity());
        params.weightNeighborCount = static_cast<float>(ipsRouting->GetWeightNeighborCount());
    }
    return params;
}

/*获取节点的多路径数量*/
uint32_t GetNodeMultipathCount(Ptr<Node> node)
{
    Ptr<Ipv4RoutingProtocol> routingProtocol = node->GetObject<Ipv4>()->GetRoutingProtocol();
    Ptr<ns3::intelligentprotocolstack::RoutingProtocol> ipsRouting =
        DynamicCast<ns3::intelligentprotocolstack::RoutingProtocol>(routingProtocol);
    if (ipsRouting)
    {
        return ipsRouting->GetMultipathCount();
    }
    return 1;
}

/*获取节点的队列长度*/
uint32_t GetNodeQueueLength(Ptr<Node> node)
{
    Ptr<Ipv4RoutingProtocol> routingProtocol = node->GetObject<Ipv4>()->GetRoutingProtocol();
    Ptr<ns3::intelligentprotocolstack::RoutingProtocol> ipsRouting =
        DynamicCast<ns3::intelligentprotocolstack::RoutingProtocol>(routingProtocol);
    if (ipsRouting)
    {
        return ipsRouting->GetQueueLength();
    }
    return 0;
}

/*获取所有节点的详细信息*/
std::vector<CommNodeInfo> getNodeInformation(RoutingProtocols* rp)
{
    std::vector<CommNodeInfo> nodeInfoList;
    nodeInfoList.reserve(rp->size);

    for (uint32_t nodeId = 0; nodeId < rp->size; ++nodeId)
    {
        CommNodeInfo nodeInfo;
        nodeInfo.nodeId = nodeId;
        Ptr<Node> node = rp->nodes.Get(nodeId);

        // 获取位置和速度
        NodePositionInfo posVel = GetNodePositionVelocity(node);
        nodeInfo.positionX = posVel.positionX;
        nodeInfo.positionY = posVel.positionY;
        nodeInfo.positionZ = posVel.positionZ;
        nodeInfo.velocityX = posVel.velocityX;
        nodeInfo.velocityY = posVel.velocityY;
        nodeInfo.velocityZ = posVel.velocityZ;

        // 获取能量信息
        nodeInfo.energyPercentage = GetNodeEnergyPercentage(node);

        // 获取Hello间隔
        nodeInfo.helloInterval = GetNodeHelloInterval(node);

        // 获取权重参数
        NodeWeightParams weights = GetNodeWeights(node);
        nodeInfo.weightDistance = weights.weightDistance;
        nodeInfo.weightLinkTime = weights.weightLinkTime;
        nodeInfo.weightRelVelocity = weights.weightRelVelocity;
        nodeInfo.weightNeighborCount = weights.weightNeighborCount;

        // 获取多路径数量
        nodeInfo.multipathCount = GetNodeMultipathCount(node);

        // 获取队列长度
        nodeInfo.queueLength = GetNodeQueueLength(node);

        nodeInfoList.push_back(nodeInfo);
    }
    return nodeInfoList;
}

/*打印所有节点的详细信息*/
void PrintNodeInformation(const std::vector<CommNodeInfo>& nodeInfoList)
{
    std::cout << "[NodeInfo] 当前节点信息 (共 " << nodeInfoList.size() << " 个节点):" << std::endl;
    for (const CommNodeInfo& nodeInfo : nodeInfoList)
    {
        std::cout << "  节点 " << nodeInfo.nodeId
                  << " | 位置: (" << std::fixed << std::setprecision(2)
                  << nodeInfo.positionX << ", " << nodeInfo.positionY << ", " << nodeInfo.positionZ << ")"
                  << " | 速度: (" << nodeInfo.velocityX << ", " << nodeInfo.velocityY << ", " << nodeInfo.velocityZ << ")"
                  << " | 能量: " << nodeInfo.energyPercentage << "%"
                  << " | Hello间隔: " << nodeInfo.helloInterval << "s"
                  << " | 队列长度: " << nodeInfo.queueLength << std::endl;
    }
}

/* 发送 HTTP 请求并同步等待响应（合二为一，避免多线程死锁） */
bool SendMessageAndAndAndListen_Comm(RoutingProtocols* rp, const std::string& jsonBody)
{
    // 1. 每次发送前建立连接，或者保持长连接（此处配合 Connection: close 每次新建连接最稳妥）
    CreateTcpConnection_Comm(rp);
    if (rp->m_sockfd < 0) return false;

    // 2. 构建 HTTP POST 请求
    std::ostringstream oss;
    oss << "POST /api/simulation/callback HTTP/1.1\r\n";
    oss << "Host: " << rp->m_serverIp << ":" << rp->m_serverPort << "\r\n";
    oss << "Content-Type: application/json\r\n";
    oss << "Content-Length: " << jsonBody.size() << "\r\n";
    oss << "Connection: close\r\n"; // 显式声明单次关闭
    oss << "\r\n";
    oss << jsonBody;

    std::string httpRequest = oss.str();

    // 3. 发送数据
    if (send(rp->m_sockfd, httpRequest.c_str(), httpRequest.size(), 0) < 0) {
        std::cerr << "[TCP] Send failed!" << std::endl;
        close(rp->m_sockfd);
        rp->m_sockfd = -1;
        return false;
    }

    // 4. 同步阻塞接收 Windows 端的 HTTP 响应
    char buffer[4096];
    std::string httpResponse = "";
    int n;
    while ((n = recv(rp->m_sockfd, buffer, sizeof(buffer) - 1, 0)) > 0) {
        buffer[n] = '\0';
        httpResponse += buffer;
    }

    close(rp->m_sockfd); // 接收完毕后关闭当前套接字
    rp->m_sockfd = -1;

    if (!httpResponse.empty()) {
        std::cout << "[SyncListener] 成功收到 Windows 响应:\n" << httpResponse << std::endl;
        return true;
    } else {
        std::cerr << "[SyncListener] 未收到任何响应，连接可能被异常断开" << std::endl;
        return false;
    }
}

/* 修改后的数据上传循环 */
void UploadLoop_Comm(RoutingProtocols* rp)
{
    std::vector<CommNodeInfo> nodeInfoList = getNodeInformation(rp);
    PrintNodeInformation(nodeInfoList);

    // 调试输出多路径数量
    // for (const auto& node : nodeInfoList) {
    //     std::cout << "[DEBUG] Node " << node.nodeId << " multipathCount=" << node.multipathCount << std::endl;
    // }

    std::string jsonData = SerializeNodeInfoToJson_Comm(nodeInfoList, rp->m_taskId, Simulator::Now().GetSeconds());

    std::cout << "[UploadLoop] 准备上传数据，主线程同步等待 Win 端处理... (当前仿真时间: "
              << Simulator::Now().GetSeconds() << "s)" << std::endl;

    // 同步发送并等待，由于是在单个网络 I/O 中阻塞，Win 端响应后才会继续往下走
    bool success = SendMessageAndAndAndListen_Comm(rp, jsonData);

    if (success) {
        std::cout << "[UploadLoop] 收到响应，成功唤醒。调度下一次上传。 (当前仿真时间: "
                  << Simulator::Now().GetSeconds() << "s)" << std::endl;
    } else {
        std::cerr << "[UploadLoop] 通信失败，尝试进入下一次循环。" << std::endl;
    }

    // 重新把控制权交还给 ns-3 仿真器，并在指定的仿真时间间隔后再次触发
    Simulator::Schedule(Seconds(rp->m_uploadInterval), &UploadLoop_Comm, rp);
}

/*上传场景参数到服务端*/
void UploadSceneParams_Comm(RoutingProtocols* rp)
{
    std::string jsonData = SerializeSceneParamsToJson_Comm(rp->m_taskId, rp->size, rp->source_dest_pairs);

    std::cout << "[UploadSceneParams] 准备上传场景参数... (当前仿真时间: "
              << Simulator::Now().GetSeconds() << "s)" << std::endl;
    std::cout << "[UploadSceneParams] JSON内容: " << jsonData << std::endl;

    bool success = SendMessageAndAndAndListen_Comm(rp, jsonData);

    if (success) {
        std::cout << "[UploadSceneParams] 场景参数上传成功。" << std::endl;
    } else {
        std::cerr << "[UploadSceneParams] 场景参数上传失败。" << std::endl;
    }
}

/*将节点信息序列化为JSON格式*/
std::string SerializeNodeInfoToJson_Comm(const std::vector<CommNodeInfo>& nodeInfoList, const std::string& taskId, double simulationTime)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{\"type\":\"simulation\",";
    oss << "\"task_id\":\"" << taskId << "\",";
    oss << "\"status\":\"running\",";
    oss << "\"simulation_time\":" << simulationTime << ",";
    oss << "\"nodes\":[";

    for (size_t i = 0; i < nodeInfoList.size(); ++i)
    {
        const CommNodeInfo& nodeInfo = nodeInfoList[i];

        if (i > 0)
            oss << ",";

        oss << "{";
        oss << "\"id\":" << nodeInfo.nodeId << ",";
        oss << "\"simulation_time\":" << simulationTime << ",";
        oss << "\"position\":{\"x\":" << nodeInfo.positionX << ",\"y\":" << nodeInfo.positionY << ",\"z\":" << nodeInfo.positionZ << "},";
        oss << "\"velocity\":{\"x\":" << nodeInfo.velocityX << ",\"y\":" << nodeInfo.velocityY << ",\"z\":" << nodeInfo.velocityZ << "},";
        oss << "\"energy_percentage\":" << nodeInfo.energyPercentage << ",";
        oss << "\"hello_interval\":" << nodeInfo.helloInterval << ",";
        oss << "\"weights\":{";
        oss << "\"distance\":" << nodeInfo.weightDistance << ",";
        oss << "\"linkTime\":" << nodeInfo.weightLinkTime << ",";
        oss << "\"relVelocity\":" << nodeInfo.weightRelVelocity << ",";
        oss << "\"neighborCount\":" << nodeInfo.weightNeighborCount;
        oss << "},";
        oss << "\"multipathCount\":" << nodeInfo.multipathCount << ",";
        oss << "\"queueLength\":" << nodeInfo.queueLength;
        oss << "}";
    }

    oss << "]}";
    return oss.str();
}

/*将场景参数序列化为JSON格式*/
std::string SerializeSceneParamsToJson_Comm(const std::string& taskId, uint32_t nodeCount,
                                            const std::vector<std::pair<int, int>>& commPairs)
{
    std::ostringstream oss;
    oss << "{\"type\":\"scene_params\",";
    oss << "\"task_id\":\"" << taskId << "\",";
    oss << "\"node_count\":" << nodeCount << ",";
    oss << "\"communication_pairs\":[";

    for (size_t i = 0; i < commPairs.size(); ++i)
    {
        if (i > 0)
            oss << ",";
        oss << "{\"source\":" << commPairs[i].first << ",\"destination\":" << commPairs[i].second << "}";
    }

    oss << "]}";
    return oss.str();
}