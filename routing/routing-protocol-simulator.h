#ifndef ROUTING_PROTOCOL_SIMULATOR_H
#define ROUTING_PROTOCOL_SIMULATOR_H

#include "routing-protocol.h"

/**
 * @file routing-protocol-simulator.h
 * @brief ns-3 网络仿真功能模块声明
 *
 * 本模块包含所有ns-3网络仿真相关的功能实现:
 * - 节点创建与移动模型安装
 * - 网络设备与协议栈配置
 * - 应用层程序安装
 * - 仿真执行与结果收集
 */
class RoutingProtocols;

/**
 * @brief 创建网络节点
 * @param rp 路由协议类指针
 * @details 根据size参数创建节点，并为每个节点分配名称（如"node-0", "node-1"等）
 */
void CreateNodes_Sim(RoutingProtocols* rp);

/**
 * @brief 安装移动模型
 * @param rp 路由协议类指针
 * @details 根据mobilityModel配置选择并安装对应的移动模型
 * - RandomWalk2dMobilityModel: 随机漫步移动模型
 * - RandomWaypointMobilityModel: 随机路点移动模型
 * - GaussMarkovMobilityModel: 高斯-马尔可夫移动模型
 */
void InstallMobilityModel_Sim(RoutingProtocols* rp);

/**
 * @brief 安装RandomWalk2dMobilityModel
 * @param rp 路由协议类指针
 * @details 2D随机漫步移动模型，节点在指定边界内随机移动
 */
void InstallMobilityModel_RandomWalk2dMobilityModel_Sim(RoutingProtocols* rp);

/**
 * @brief 安装RandomWaypointMobilityModel
 * @param rp 路由协议类指针
 * @details 随机路点移动模型，节点在随机目标点之间移动
 */
void InstallMobilityModel_RandomWaypointMobilityModel_Sim(RoutingProtocols* rp);

/**
 * @brief 安装GaussMarkovMobilityModel
 * @param rp 路由协议类指针
 * @details 高斯-马尔可夫移动模型，基于时间相关的高斯-马尔可夫随机过程
 */
void InstallMobilityModel_GaussMarkovMobilityModel_Sim(RoutingProtocols* rp);

/**
 * @brief 创建WiFi网络设备
 * @param rp 路由协议类指针
 * @details 配置并创建WiFi物理层、信道和MAC层
 * - 设置接收增益
 * - 配置恒定速度传播延迟模型
 * - 配置Range传播损失模型（基于通信范围）
 */
void CreateDevices_Sim(RoutingProtocols* rp);

/**
 * @brief 安装能量模型
 * @param rp 路由协议类指针
 * @details 为所有节点安装能量源和WiFi设备能量消耗模型
 * - 基础能量源初始能量: 100焦耳
 * - 发射/接收/空闲/切换/睡眠电流配置
 */
void InstallEnergyModel_Sim(RoutingProtocols* rp);

/**
 * @brief 安装Internet协议栈
 * @param rp 路由协议类指针
 * @details 根据algorithm配置安装对应的路由协议
 * - gpsr: GPSR贪婪协议周边路由
 * - mmgpsr: MMGPSR移动代理GPSR
 * - aodv: AODV按需距离向量路由
 * - dsdv: DSDV目标序列距离向量
 * - olsr: OLSR最优链路状态路由
 * - intelligent-protocol-stack: 智能协议栈
 */
void InstallInternetStack_Sim(RoutingProtocols* rp);

/**
 * @brief 安装UDP Echo应用
 * @param rp 路由协议类指针
 * @details 创建源-目的节点对并安装UDP Echo客户端/服务器应用
 * - 使用UdpEchoServerHelper和UdpEchoClientHelper
 * - 配置最大包数、包间隔和包大小
 */
void InstallApplications_Sim(RoutingProtocols* rp);

/**
 * @brief 初始化路由协议
 * @param rp 路由协议类指针
 * @details 仅GPSR和intelligent-protocol-stack需要额外初始化
 */
void InitRoutingProtocol_Sim(RoutingProtocols* rp);

/**
 * @brief 运行仿真并计算指标
 * @param rp 路由协议类指针
 * @details 执行完整的仿真流程:
 * - 安装FlowMonitor监控
 * - 启用NetAnim动画
 * - 运行仿真
 * - 收集统计数据（发送/接收包数、时延、吞吐量、跳数、收敛时间）
 * - 输出结果到XML和CSV文件
 */
void RunAndCal_Sim(RoutingProtocols* rp);

/**
 * @brief 将仿真结果写入文件
 * @param rp 路由协议类指针
 * @param avgDelay 平均端到端时延（ms）
 * @param deliveryRatio 数据包交付率（%）
 * @param avgThroughput 平均吞吐量（kbps）
 * @param avgHopCount 平均跳数
 * @param convergenceTime 收敛时间（ms）
 * @param flowDelay 流时延（秒）
 * @param receivedPackets 接收数据包数量
 * @details 将仿真结果追加写入RoutingResult目录下的结果文件
 */
void WriteToFile_Sim(RoutingProtocols* rp, double avgDelay, double deliveryRatio,
                      double avgThroughput, double avgHopCount, double convergenceTime,
                      double flowDelay, uint32_t receivedPackets);

/**
 * @brief 仿真结束后输出所有源节点的数据包转发路径
 * @param rp 路由协议类指针
 * @details 遍历 rp->source_dest_pairs，对每个 (source, dest) 节点对
 *          从 GlobalInformation 单例中读取该 source 已记录的转发路径，
 *          打印路径数、去重转发者数、跳数分布，以及若干条样例路径。
 *          数据来源：intelligent-protocol-stack 在 RouteOutput / Forwarding /
 *          RouteInput 三个钩子里写入 GlobalInformation。
 */
void PrintTransmissionPaths_Sim(RoutingProtocols* rp);

/**
 * @brief 仅对 IPS 协议源节点单独设置自定义权重；中转/目的节点保持默认 0.25
 * @param rp 路由协议类指针
 * @details 在 InstallApplications_Sim 选出 source_dest_pairs 之后调用。
 *          通过 SetAttribute 写回 m_weight* 成员变量，再调用
 *          UpdateNeighborWeights() 同步到 m_neighbors (PositionTable)，
 *          路由决策使用的就是后者。其它非源节点上 m_neighbors 仍保持
 *          构造函数中固化的 0.25，从而实现"只对源节点生效"的语义。
 */
void ApplyIpsWeightsToSourceNodes_Sim(RoutingProtocols* rp);

#endif // ROUTING_PROTOCOL_SIMULATOR_H
