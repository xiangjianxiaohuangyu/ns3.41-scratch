#ifndef ROUTING_PROTOCOL_COMM_H
#define ROUTING_PROTOCOL_COMM_H

#include "routing-protocol.h"

/**
 * @file routing-protocol-comm.h
 * @brief 服务端通信功能模块声明
 *
 * 本模块包含所有TCP/IP服务端通信相关的功能:
 * - TCP连接管理与数据收发
 * - 拓扑信息上报
 * - 支持暂停/恢复机制的上传循环
 * - 节点信息JSON序列化
 */

/**
 * @struct NodePositionInfo
 * @brief 节点位置和速度信息结构体
 */
struct NodePositionInfo
{
    float positionX;
    float positionY;
    float positionZ;
    float velocityX;
    float velocityY;
    float velocityZ;

    NodePositionInfo()
        : positionX(0), positionY(0), positionZ(0),
          velocityX(0), velocityY(0), velocityZ(0) {}
};

/**
 * @struct CommNodeInfo
 * @brief 用于通信的节点信息结构体
 *
 * 只包含需要传输给服务端的字段，与协议内部的NodeInfo解耦
 */
struct CommNodeInfo
{
    uint32_t nodeId;           ///< 节点ID
    float positionX;           ///< X坐标
    float positionY;           ///< Y坐标
    float positionZ;           ///< Z坐标
    float velocityX;           ///< X方向速度
    float velocityY;           ///< Y方向速度
    float velocityZ;           ///< Z方向速度
    float energyPercentage;    ///< 能量百分比 (0-100)
    double helloInterval;      ///< Hello包发送间隔 (秒)
    float weightDistance;       ///< 距离权重 (w1)
    float weightLinkTime;       ///< 链路保持时间权重 (w2)
    float weightRelVelocity;    ///< 相对速度权重 (w3)
    float weightNeighborCount;  ///< 邻居数量权重 (w4)
    uint32_t multipathCount;    ///< 多路径数量
    uint32_t queueLength;      ///< 队列长度

    CommNodeInfo()
        : nodeId(0), positionX(0), positionY(0), positionZ(0),
          velocityX(0), velocityY(0), velocityZ(0), energyPercentage(0),
          helloInterval(0), weightDistance(0.25f), weightLinkTime(0.25f),
          weightRelVelocity(0.25f), weightNeighborCount(0.25f), multipathCount(1),
          queueLength(0) {}
};

 /**
 * @brief 创建并建立TCP连接到服务端
 * @param rp 路由协议类指针
 * @details 创建TCP socket并连接到配置的服务器
 * - 使用AF_INET IPv4协议族
 * - SOCK_STREAM流式socket
 * - 连接成功后更新m_socketConnected状态
 */
void CreateTcpConnection_Comm(RoutingProtocols* rp);

/**
 * @brief 初始化Socket连接
 * @param rp 路由协议类指针
 * @details 调用CreateTcpConnection_Comm建立连接
 */
void InitSocket_Comm(RoutingProtocols* rp);

/**
 * @brief 发送消息到服务端
 * @param rp 路由协议类指针
 * @param msg 要发送的消息字符串
 * @details 通过已建立的TCP socket发送数据
 */
void SendMessage_Comm(RoutingProtocols* rp, const std::string& msg);

/**
 * @brief 上报拓扑信息到服务端
 * @param rp 路由协议类指针
 * @warning 已废弃，请使用UploadLoop_Comm代替
 * @details 发送JSON格式的拓扑信息，包含节点位置
 * - 调度下一次上报（带随机延迟）
 */
void ReportTopology_Comm(RoutingProtocols* rp);

/**
 * @brief 上传数据循环（支持暂停/恢复机制）
 * @param rp 路由协议类指针
 * @details 周期性上传节点信息到服务端:
 * - 序列化节点信息为JSON格式
 * - 发送到服务端
 * - 阻塞等待服务端响应
 * - 收到响应后继续执行并调度下一次上传
 *
 * 暂停/恢复机制:
 * - 发送数据后使用条件变量阻塞等待
 * - 收到服务端响应时触发m_pauseCond唤醒
 */
void UploadLoop_Comm(RoutingProtocols* rp);

/**
 * @brief 上传场景参数到服务端
 * @param rp 路由协议类指针
 * @details 在仿真开始时立即发送场景参数JSON，包含:
 * - type: "scene_params"
 * - task_id: 任务唯一标识符
 * - node_count: 节点总数
 * - communication_pairs: 通信节点对列表
 */
void UploadSceneParams_Comm(RoutingProtocols* rp);

/**
 * @brief 触发暂停
 * @param rp 路由协议类指针
 * @details 当收到服务端数据时调用，设置m_waitingForResponse为false并通知条件变量
 */
void TriggerPause_Comm(RoutingProtocols* rp);

/**
 * @brief 恢复仿真
 * @param rp 路由协议类指针
 * @details 服务端处理完数据后调用，唤醒UploadLoop继续执行
 */
void ResumeSimulation_Comm(RoutingProtocols* rp);

/**
 * @brief 启动TCP监听线程
 * @param rp 路由协议类指针
 * @details 创建新线程运行ListenerThreadFunc_Comm
 */
void StartListenerThread_Comm(RoutingProtocols* rp);

/**
 * @brief TCP监听线程函数
 * @param rp 路由协议类指针
 * @details 在独立线程中运行:
 * - 使用select监听socket可读事件
 * - 接收服务端响应数据
 * - 收到响应后调用TriggerPause_Comm唤醒主线程
 * - 连接关闭时退出循环
 */
void ListenerThreadFunc_Comm(RoutingProtocols* rp);

/**
 * @brief 生成UUID v4格式的字符串
 * @return UUID字符串，格式如 "550e8400-e29b-41d4-a716-446655440000"
 */
std::string GenerateUUID();

/**
 * @brief 获取节点的位置和速度信息
 * @param node 节点指针
 * @return 包含位置(xyz)和速度(xyz)的结构体
 */
NodePositionInfo GetNodePositionVelocity(Ptr<Node> node);

/**
 * @brief 获取节点的剩余能量百分比
 * @param node 节点指针
 * @return 能量百分比 (0-100)
 */
float GetNodeEnergyPercentage(Ptr<Node> node);

/**
 * @brief 获取节点的Hello包发送间隔
 * @param node 节点指针
 * @return Hello间隔（秒），如果不是IPS协议返回0
 */
double GetNodeHelloInterval(Ptr<Node> node);

/**
 * @brief 获取节点选择下一跳时的四个权重参数
 * @param node 节点指针
 * @return 包含 weightDistance, weightLinkTime, weightRelVelocity, weightNeighborCount 的结构体
 */
struct NodeWeightParams
{
    float weightDistance;      // w1: 距离权重
    float weightLinkTime;       // w2: 链路保持时间权重
    float weightRelVelocity;    // w3: 相对速度权重
    float weightNeighborCount;  // w4: 邻居数量权重
};
NodeWeightParams GetNodeWeights(Ptr<Node> node);

/**
 * @brief 获取节点的多路径数量
 * @param node 节点指针
 * @return 多路径数量，如果不是IPS协议返回1
 */
uint32_t GetNodeMultipathCount(Ptr<Node> node);

/**
 * @brief 获取节点的队列长度
 * @param node 节点指针
 * @return 队列长度，如果不是IPS协议返回0
 */
uint32_t GetNodeQueueLength(Ptr<Node> node);

/**
 * @brief 获取所有节点的详细信息
 * @param rp 路由协议类指针
 * @return 节点信息列表，按节点ID排序
 */
std::vector<CommNodeInfo> getNodeInformation(RoutingProtocols* rp);

/**
 * @brief 打印所有节点的详细信息
 * @param nodeInfoList 节点信息列表
 * @details 输出每个节点的ID、位置、速度、能量百分比
 */
void PrintNodeInformation(const std::vector<CommNodeInfo>& nodeInfoList);

/**
 * @brief 将节点信息序列化为JSON格式
 * @param nodeInfoList 节点信息列表
 * @param taskId 任务唯一标识符(UUID)
 * @param simulationTime 当前仿真时间（秒）
 * @return JSON字符串，包含task_id、status、simulation_time、节点位置、速度、能量等信息
 * @details 序列化格式:
 * @code
 * {
 *   "task_id": "550e8400-e29b-41d4-a716-446655440000",
 *   "status": "running",
 *   "simulation_time": 12.50,
 *   "nodes": [
 *     {
 *       "id": 0,
 *       "position": {"x": 100.5, "y": 200.3, "z": 0.0},
 *       "velocity": {"x": 1.5, "y": -0.5, "z": 0.0},
 *       "energy_percentage": 85.5
 *     }
 *   ]
 * }
 * @endcode
 */
std::string SerializeNodeInfoToJson_Comm(const std::vector<CommNodeInfo>& nodeInfoList, const std::string& taskId, double simulationTime);

/**
 * @brief 将场景参数序列化为JSON格式
 * @param taskId 任务唯一标识符(UUID)
 * @param nodeCount 节点总数
 * @param commPairs 通信节点对 (源节点, 目的节点)
 * @return JSON字符串，包含type、task_id、node_count、communication_pairs
 * @details 序列化格式:
 * @code
 * {
 *   "type": "scene_params",
 *   "task_id": "550e8400-e29b-41d4-a716-446655440000",
 *   "node_count": 50,
 *   "communication_pairs": [
 *     {"source": 0, "destination": 5},
 *     {"source": 2, "destination": 7}
 *   ]
 * }
 * @endcode
 */
std::string SerializeSceneParamsToJson_Comm(const std::string& taskId, uint32_t nodeCount,
                                             const std::vector<std::pair<int, int>>& commPairs);

#endif // ROUTING_PROTOCOL_COMM_H
