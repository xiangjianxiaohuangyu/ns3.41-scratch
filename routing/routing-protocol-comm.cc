#include "routing-protocol-comm.h"
#include <random>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <regex>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ns3/simulator.h"
#include "ns3/node.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/intelligent-protocol-stack-module.h"
#include "ns3/node-information.h"

using namespace ns3;
using namespace intelligentprotocolstack;

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

/* 发送 HTTP 请求并同步等待响应（合二为一，避免多线程死锁）
   responseBody 传出 HTTP body 部分的 JSON（不含响应头），供调用方解析
   new_parameters 等控制字段；传 nullptr 表示只关心成功失败。 */
bool SendMessageAndAndAndListen_Comm(RoutingProtocols* rp, const std::string& jsonBody, std::string* responseBody)
{
    // 1. 每次发送前建立连接
    CreateTcpConnection_Comm(rp);
    if (rp->m_sockfd < 0) return false;

    // 2. 构建 HTTP POST 请求
    std::ostringstream oss;
    oss << "POST /api/simulation/callback HTTP/1.1\r\n";
    oss << "Host: " << rp->m_serverIp << ":" << rp->m_serverPort << "\r\n";
    oss << "Content-Type: application/json\r\n";
    oss << "Content-Length: " << jsonBody.size() << "\r\n";
    oss << "Connection: close\r\n";
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

    close(rp->m_sockfd);
    rp->m_sockfd = -1;

    if (httpResponse.empty()) {
        std::cerr << "[SyncListener] 未收到任何响应，连接可能被异常断开" << std::endl;
        return false;
    }

    std::cout << "[SyncListener] 成功收到 Windows 响应:\n" << httpResponse << std::endl;

    // 5. 取出 HTTP body 传给调用方
    if (responseBody != nullptr) {
        std::size_t hdrEnd = httpResponse.find("\r\n\r\n");
        if (hdrEnd != std::string::npos) {
            *responseBody = httpResponse.substr(hdrEnd + 4);
        } else {
            // 容错：没找到 CRLFCRLF 就把整段当 body
            *responseBody = httpResponse;
        }
    }
    return true;
}

/* 把源节点 ni 序列化为 JSON 字符串（内层参数，无 scene_info/para_info/result_info 外层） */
static std::string SerializeNodeInformationToJson(const NodeInformation& ni)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{"
        << "\"speed\":" << ni.GetSpeed () << ","
        << "\"energy\":" << ni.GetEnergy () << ","
        << "\"queue_length\":" << ni.GetQueueLength () << ","
        << "\"neighbor_count\":" << ni.GetNeighborCount () << ","
        << "\"distance_to_destination\":" << ni.GetDistanceToDestination () << ","

        << "\"forward_candidate_ratio\":" << ni.GetForwardCandidateRatio () << ","
        << "\"distance_to_me_mean\":" << ni.GetDistanceToMeMean () << ","
        << "\"distance_to_me_std\":" << ni.GetDistanceToMeStd () << ","
        << "\"distance_to_destination_mean\":" << ni.GetDistanceToDestinationMean () << ","
        << "\"distance_to_destination_std\":" << ni.GetDistanceToDestinationStd () << ","
        << "\"distance_to_destination_min\":" << ni.GetDistanceToDestinationMin () << ","
        << "\"relative_speed_mean\":" << ni.GetRelativeSpeedMean () << ","
        << "\"relative_speed_std\":" << ni.GetRelativeSpeedStd () << ","
        << "\"link_lifetime_mean\":" << ni.GetLinkLifetimeMean () << ","
        << "\"link_lifetime_std\":" << ni.GetLinkLifetimeStd () << ","
        << "\"neighbor_degree_mean\":" << ni.GetNeighborDegreeMean () << ","
        << "\"neighbor_degree_std\":" << ni.GetNeighborDegreeStd () << ","
        << "\"queue_length_mean\":" << ni.GetNeighborQueueLengthMean () << ","
        << "\"queue_length_std\":" << ni.GetNeighborQueueLengthStd () << ","
        << "\"queue_length_max\":" << ni.GetNeighborQueueLengthMax () << ","
        << "\"energy_mean\":" << ni.GetNeighborEnergyMean () << ","
        << "\"energy_std\":" << ni.GetNeighborEnergyStd () << ","
        << "\"energy_min\":" << ni.GetNeighborEnergyMin () << ","

        << "\"hello_interval\":" << ni.GetHelloInterval () << ","
        << "\"path_num\":" << ni.GetPathNum () << ","
        << "\"w_distance\":" << ni.GetWeightDistance () << ","
        << "\"w_linkTime\":" << ni.GetWeightLinkTime () << ","
        << "\"w_relVelocity\":" << ni.GetWeightRelVelocity () << ","
        << "\"w_neighborCount\":" << ni.GetWeightNeighborCount () << ","

        << "\"avg_pdr\":" << ni.GetAvgPdr () << ","
        << "\"avg_delay\":" << ni.GetAvgDelay ()

        << "}";
    return oss.str ();
}

/* 执行一次 JSON 上传（核心逻辑：序列化源节点 ni -> 发送 -> 关闭） */
void DoUpload_Comm(RoutingProtocols* rp)
{
    if (rp == nullptr) return;

    uint32_t srcId = intelligentprotocolstack::GlobalInformation::GetInstance ().getSourceNode ();
    if (srcId < NodeList::GetNNodes ())
    {
        Ptr<Node> srcNode = NodeList::GetNode (srcId);
        Ptr<RoutingProtocol> ips = srcNode->GetObject<RoutingProtocol> ();
        if (ips != nullptr)
        {
            const NodeInformation& ni = ips->GetNodeInformation ();
            std::string jsonBody = SerializeNodeInformationToJson (ni);

            // 打印即将发送给 Windows 端的数据，便于本地排查。
            std::cout << "================ [DoUpload] 即将发送给 Windows 端的数据 ================" << std::endl;
            std::cout << "[DoUpload] 源节点 src=" << srcId
                      << "  taskId=" << rp->m_taskId
                      << "  sim_time=" << Simulator::Now ().GetSeconds () << "s"
                      << "  payload_size=" << jsonBody.size () << " bytes" << std::endl;
            std::cout << "[DoUpload] JSON payload (raw):" << std::endl;
            std::cout << jsonBody << std::endl;

            // 同时输出一份 key/value 形式的可读视图，方便人工核对字段
            std::cout << "[DoUpload] JSON payload (decoded):" << std::endl;
            std::cout << "  speed                       = " << ni.GetSpeed ()                << std::endl;
            std::cout << "  energy                      = " << ni.GetEnergy ()               << std::endl;
            std::cout << "  queue_length                = " << ni.GetQueueLength ()          << std::endl;
            std::cout << "  neighbor_count              = " << ni.GetNeighborCount ()        << std::endl;
            std::cout << "  distance_to_destination     = " << ni.GetDistanceToDestination () << std::endl;
            std::cout << "  forward_candidate_ratio     = " << ni.GetForwardCandidateRatio () << std::endl;
            std::cout << "  distance_to_me_mean         = " << ni.GetDistanceToMeMean ()      << std::endl;
            std::cout << "  distance_to_me_std          = " << ni.GetDistanceToMeStd ()       << std::endl;
            std::cout << "  distance_to_destination_mean= " << ni.GetDistanceToDestinationMean () << std::endl;
            std::cout << "  distance_to_destination_std = " << ni.GetDistanceToDestinationStd ()  << std::endl;
            std::cout << "  distance_to_destination_min = " << ni.GetDistanceToDestinationMin ()  << std::endl;
            std::cout << "  relative_speed_mean         = " << ni.GetRelativeSpeedMean ()     << std::endl;
            std::cout << "  relative_speed_std          = " << ni.GetRelativeSpeedStd ()      << std::endl;
            std::cout << "  link_lifetime_mean          = " << ni.GetLinkLifetimeMean ()      << std::endl;
            std::cout << "  link_lifetime_std           = " << ni.GetLinkLifetimeStd ()       << std::endl;
            std::cout << "  neighbor_degree_mean        = " << ni.GetNeighborDegreeMean ()    << std::endl;
            std::cout << "  neighbor_degree_std         = " << ni.GetNeighborDegreeStd ()     << std::endl;
            std::cout << "  queue_length_mean           = " << ni.GetNeighborQueueLengthMean () << std::endl;
            std::cout << "  queue_length_std            = " << ni.GetNeighborQueueLengthStd ()  << std::endl;
            std::cout << "  queue_length_max            = " << ni.GetNeighborQueueLengthMax ()  << std::endl;
            std::cout << "  energy_mean                 = " << ni.GetNeighborEnergyMean ()   << std::endl;
            std::cout << "  energy_std                  = " << ni.GetNeighborEnergyStd ()    << std::endl;
            std::cout << "  energy_min                  = " << ni.GetNeighborEnergyMin ()    << std::endl;
            std::cout << "  hello_interval              = " << ni.GetHelloInterval ()        << std::endl;
            std::cout << "  path_num                    = " << ni.GetPathNum ()              << std::endl;
            std::cout << "  w_distance                  = " << ni.GetWeightDistance ()       << std::endl;
            std::cout << "  w_linkTime                  = " << ni.GetWeightLinkTime ()       << std::endl;
            std::cout << "  w_relVelocity               = " << ni.GetWeightRelVelocity ()    << std::endl;
            std::cout << "  w_neighborCount             = " << ni.GetWeightNeighborCount ()  << std::endl;
            std::cout << "  avg_pdr                     = " << ni.GetAvgPdr ()               << std::endl;
            std::cout << "  avg_delay                   = " << ni.GetAvgDelay ()             << std::endl;
            std::cout << "==========================================================================" << std::endl;

            if (rp->m_socketConnected)
            {
                std::string responseBody;
                bool ok = SendMessageAndAndAndListen_Comm (rp, jsonBody, &responseBody);
                if (ok && !responseBody.empty ())
                {
                    // 把 Windows 端在 result 中给出的 new_parameters 动态
                    // 套到源节点 IPS RoutingProtocol 上：权重 / 路径数 /
                    // Hello 间隔都会立即生效（下次 Hello timer 触发即按新
                    // 间隔跑）。
                    ApplyNewParametersToSourceNode_Comm (rp, responseBody);
                }
                else if (ok)
                {
                    std::cerr << "[DoUpload] 收到空响应 body，跳过参数动态调整。"
                              << " (src=" << srcId
                              << ", sim_time=" << Simulator::Now ().GetSeconds () << "s)"
                              << std::endl;
                }
            }
            else
            {
                std::cerr << "[DoUpload] socket 未连接，跳过本次上报 (src=" << srcId
                          << ", sim_time=" << Simulator::Now ().GetSeconds () << "s)" << std::endl;
            }
        }
        else
        {
            std::cerr << "[DoUpload] 源节点 " << srcId
                      << " 上未找到 IPS RoutingProtocol，跳过上报。" << std::endl;
        }
    }
    else
    {
        std::cerr << "[DoUpload] 源节点 id=" << srcId
                  << " 超出范围 (NNodes=" << NodeList::GetNNodes () << ")，跳过上报。" << std::endl;
    }
}

/* 周期循环：Run() 启动时按 m_uploadInterval 调度本函数；本函数内完成一次
   上传（DoUpload_Comm，会同步阻塞等待 Windows 端返回的 result）后，
   再按 m_uploadInterval 重新调度自身，从而形成周期循环。*/
void UploadLoop_Comm(RoutingProtocols* rp)
{
    if (rp == nullptr) return;

    DoUpload_Comm (rp);

    Simulator::Schedule (Seconds (rp->m_uploadInterval), &UploadLoop_Comm, rp);
}

/* 在 JSON 文本中取一个 key 的数字子串（key:digits），没找到返回 false。
   只识别 : <number> 这种简单形态，足以覆盖 win 端 new_parameters 这种
   全是扁平数字字段的子对象；解析失败也不会抛异常。 */
static bool ExtractJsonNumber(const std::string& json, const std::string& key, double& out)
{
    // 形如 "key":<number>；key 用字面量匹配，避免误中。
    std::string pat = "\"" + key + "\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?)";
    std::regex re(pat);
    std::smatch m;
    if (std::regex_search(json, m, re))
    {
        try { out = std::stod(m[1].str()); return true; }
        catch (...) { return false; }
    }
    return false;
}

/* 在 JSON 文本中找 "new_parameters" 紧跟的 { ... } 子对象（花括号配对），
   返回该子对象串的左闭右开区间 [begin,end)；没找到返回 std::string::npos 起点。 */
static std::pair<std::size_t, std::size_t> FindNewParametersObject(const std::string& json)
{
    std::regex keyRe("\"new_parameters\"\\s*:\\s*\\{");
    std::smatch m;
    if (!std::regex_search(json, m, keyRe)) return {std::string::npos, std::string::npos};

    std::size_t braceStart = m.position(0) + m.length(0) - 1;  // 指向 '{'
    int depth = 0;
    std::size_t end = braceStart;
    for (std::size_t i = braceStart; i < json.size(); ++i)
    {
        if (json[i] == '{') ++depth;
        else if (json[i] == '}')
        {
            --depth;
            if (depth == 0) { end = i + 1; break; }
        }
    }
    if (depth != 0) return {std::string::npos, std::string::npos};
    return {braceStart, end};
}

/* 把 Windows 端响应 JSON 中的 new_parameters 动态套到源节点 IPS 上。
   若响应里没有 new_parameters（例如 win 端尚未做出决策），本函数静默返回；
   若任何字段缺失/解析失败，仅跳过该字段，不影响其他字段。 */
void ApplyNewParametersToSourceNode_Comm(RoutingProtocols* rp, const std::string& responseJson)
{
    if (rp == nullptr) return;

    auto range = FindNewParametersObject(responseJson);
    if (range.first == std::string::npos)
    {
        std::cout << "[ApplyParams] 响应中未找到 new_parameters 对象，跳过参数动态调整。"
                  << " (taskId=" << rp->m_taskId << ")" << std::endl;
        return;
    }
    std::string subObj = responseJson.substr(range.first, range.second - range.first);

    double w_distance = 0, w_linkTime = 0, w_relVelocity = 0, w_neighborCount = 0;
    double hello_interval = 0;
    double path_num_d = 0;
    bool hasW1 = ExtractJsonNumber(subObj, "w_distance",      w_distance);
    bool hasW2 = ExtractJsonNumber(subObj, "w_linkTime",      w_linkTime);
    bool hasW3 = ExtractJsonNumber(subObj, "w_relVelocity",   w_relVelocity);
    bool hasW4 = ExtractJsonNumber(subObj, "w_neighborCount", w_neighborCount);
    bool hasHi = ExtractJsonNumber(subObj, "hello_interval",  hello_interval);
    bool hasPn = ExtractJsonNumber(subObj, "path_num",        path_num_d);

    if (!hasW1 && !hasW2 && !hasW3 && !hasW4 && !hasHi && !hasPn)
    {
        std::cerr << "[ApplyParams] new_parameters 子对象中未解析出任何已知字段，跳过。"
                  << " (taskId=" << rp->m_taskId << ")" << std::endl;
        return;
    }

    uint32_t srcId = intelligentprotocolstack::GlobalInformation::GetInstance().getSourceNode();
    if (srcId >= NodeList::GetNNodes())
    {
        std::cerr << "[ApplyParams] 源节点 id=" << srcId
                  << " 超出范围，跳过参数动态调整。" << std::endl;
        return;
    }
    Ptr<Node> srcNode = NodeList::GetNode(srcId);
    Ptr<intelligentprotocolstack::RoutingProtocol> ips =
        srcNode->GetObject<intelligentprotocolstack::RoutingProtocol>();
    if (ips == nullptr)
    {
        std::cerr << "[ApplyParams] 源节点 " << srcId
                  << " 上未找到 IPS RoutingProtocol，无法应用 new_parameters。" << std::endl;
        return;
    }

    std::cout << "================ [ApplyParams] 收到 win 端 new_parameters ================" << std::endl;
    std::cout << "  taskId          = " << rp->m_taskId << std::endl;
    std::cout << "  src node        = " << srcId << std::endl;
    std::cout << "  sim_time        = " << Simulator::Now().GetSeconds() << "s" << std::endl;
    if (hasW1) std::cout << "  w_distance      = " << w_distance     << std::endl;
    if (hasW2) std::cout << "  w_linkTime      = " << w_linkTime     << std::endl;
    if (hasW3) std::cout << "  w_relVelocity   = " << w_relVelocity  << std::endl;
    if (hasW4) std::cout << "  w_neighborCount = " << w_neighborCount << std::endl;
    if (hasPn) std::cout << "  path_num        = " << static_cast<uint32_t>(path_num_d) << std::endl;
    if (hasHi) std::cout << "  hello_interval  = " << hello_interval << "s" << std::endl;
    std::cout << "--------------------------------------------------------------------------" << std::endl;

    // 1) 写四个权重；SetAttribute 走的是 TypeId 里 MakeDoubleAccessor 绑定的
    //    m_weight* 成员，写完再调 UpdateNeighborWeights() 把新值同步到
    //    m_neighbors（路由决策实际读它）以及 ni（下次 JSON 上报读它）。
    if (hasW1) ips->SetAttribute("WeightDistance",     DoubleValue(w_distance));
    if (hasW2) ips->SetAttribute("WeightLinkTime",     DoubleValue(w_linkTime));
    if (hasW3) ips->SetAttribute("WeightRelVelocity",  DoubleValue(w_relVelocity));
    if (hasW4) ips->SetAttribute("WeightNeighborCount",DoubleValue(w_neighborCount));
    if (hasW1 || hasW2 || hasW3 || hasW4) ips->UpdateNeighborWeights();

    // 2) path_num：SetAttribute 写 m_multipathCount（IPS TypeId 暴露为
    //    "MultipathCount"），同时把 ni 也同步上，否则下次 JSON 上报里
    //    path_num 字段读到的还是旧值。
    if (hasPn)
    {
        uint32_t pathNum = static_cast<uint32_t>(path_num_d);
        if (pathNum < 1) pathNum = 1;
        ips->SetAttribute("MultipathCount", UintegerValue(pathNum));
        ips->GetNodeInformation().SetPathNum(pathNum);
    }

    // 3) hello_interval：SetAttribute 写 HelloInterval（Time 字段）；
    //    HelloIntervalTimer 是私有的且每次 HelloTimerExpire 才重新
    //    Schedule(HelloInterval+...)，所以新值在下一拍 Hello timer 触发时
    //    自然生效。同时把 ni 同步上，IPSRouting 的构造函数里没主动做这个
    //    同步，这里顺手补上。
    if (hasHi)
    {
        if (hello_interval <= 0.0) hello_interval = 0.001;  // 防呆
        ips->SetAttribute("HelloInterval", TimeValue(Seconds(hello_interval)));
        ips->GetNodeInformation().SetHelloInterval(hello_interval);
    }

    std::cout << "[ApplyParams] 已将 new_parameters 应用到源节点 " << srcId
              << " 的 IPS RoutingProtocol。" << std::endl;
    std::cout << "  current w_distance      = " << ips->GetWeightDistance()      << std::endl;
    std::cout << "  current w_linkTime      = " << ips->GetWeightLinkTime()      << std::endl;
    std::cout << "  current w_relVelocity   = " << ips->GetWeightRelVelocity()   << std::endl;
    std::cout << "  current w_neighborCount = " << ips->GetWeightNeighborCount() << std::endl;
    std::cout << "  current multipathCount  = " << ips->GetMultipathCount()      << std::endl;
    std::cout << "==========================================================================" << std::endl;
}
