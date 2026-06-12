#ifndef ROUTING_PROTOCOL_COMM_H
#define ROUTING_PROTOCOL_COMM_H

#include "routing-protocol.h"
#include <cstdint>

/**
 * @file routing-protocol-comm.h
 * @brief 服务端通信功能模块声明
 *
 * 本模块包含所有TCP/IP服务端通信相关的功能:
 * - TCP连接管理与数据收发
 * - 上传循环（按节点身份过滤；从源节点 IPS RoutingProtocol 上的
 *   NodeInformation 中读取所有内层参数，序列化为 JSON 发送到 Windows 端）
 */

/**
 * @brief 生成UUID v4格式的字符串
 */
std::string GenerateUUID();

/**
 * @brief 创建并建立TCP连接到服务端
 */
void CreateTcpConnection_Comm(RoutingProtocols* rp);

/**
 * @brief 初始化Socket连接
 */
void InitSocket_Comm(RoutingProtocols* rp);

/**
 * @brief 执行一次 JSON 数据上传（核心逻辑，不做任何调度）
 * @details 从 GlobalInformation 取出源节点 id，在该节点上取出 IPS
 *          RoutingProtocol，读取其 NodeInformation 快照，序列化为 JSON
 *          通过 SendMessageAndAndAndListen_Comm 发往 Windows 端。
 *          本函数只负责"做一次上传"，不调度也不重试，可由周期循环
 *          UploadLoop_Comm 调用，也可由 RunAndCal_Sim 在仿真结束时
 *          直接调用一次以完成最终结果上报。
 */
void DoUpload_Comm(RoutingProtocols* rp);

/**
 * @brief 上传数据循环（周期调度版本）
 * @details Run() 启动仿真时按 m_uploadInterval 调度本函数一次。
 *          内部调用 DoUpload_Comm 完成一次上传（同步阻塞等待 Windows
 *          端返回的 result），再按 m_uploadInterval 重新调度自身，
 *          形成周期循环。RunAndCal_Sim 在仿真结束时仍会再调用一次
 *          DoUpload_Comm 以发送最终结果（含 PDR / 时延）。
 */
void UploadLoop_Comm(RoutingProtocols* rp);

/**
 * @brief 把 Windows 端返回的 JSON 中的 new_parameters 动态应用到源节点
 * @param rp 路由协议类指针（用于定位源节点）
 * @param responseJson Windows 端 HTTP 响应体（已剥离 HTTP 头）
 * @details 解析 responseJson 中的
 *          new_parameters.{w_distance,w_linkTime,w_relVelocity,
 *          w_neighborCount,path_num,hello_interval} 六个字段，调用
 *          SetAttribute 写入源节点 IPS RoutingProtocol；权重类参数
 *          还会调用 UpdateNeighborWeights() 同步到 m_neighbors。
 *          HelloInterval 修改后会取消当前 timer 并立即按新值重新
 *          调度，保证新间隔在下一拍就生效。
 */
void ApplyNewParametersToSourceNode_Comm(RoutingProtocols* rp, const std::string& responseJson);

#endif // ROUTING_PROTOCOL_COMM_H
