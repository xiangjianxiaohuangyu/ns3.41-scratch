#include "routing-protocol-simulator.h"

/*生成节点并命名*/
void CreateNodes_Sim(RoutingProtocols* rp){
    std::cout<<"Size : "<<rp->size<<std::endl;
    rp->nodes.Create(rp->size);

    for (uint32_t i = 0; i < rp->size; ++i){
        std::ostringstream os;
        os << "node-" << i;
        Names::Add(os.str(), rp->nodes.Get(i));
    }
}

void InstallMobilityModel_Sim(RoutingProtocols* rp){
    if(rp->mobilityModel == "RandomWalk2dMobilityModel"){
        std::cout<<"Mobility model : RandomWalk2dMobilityModel"<<std::endl;
        InstallMobilityModel_RandomWalk2dMobilityModel_Sim(rp);
    }else if(rp->mobilityModel == "RandomWaypointMobilityModel"){
        std::cout<<"Mobility model : RandomWaypointMobilityModel"<<std::endl;
        InstallMobilityModel_RandomWaypointMobilityModel_Sim(rp);
    }else if(rp->mobilityModel == "GaussMarkovMobilityModel"){
        std::cout<<"Mobility model : GaussMarkovMobilityModel"<<std::endl;
        InstallMobilityModel_GaussMarkovMobilityModel_Sim(rp);
    }else{
        std::cout << "Install mobility model failed !"<<std::endl;
    }

    std::cout<<"Node speed : "<<rp->nodeSpeedMin<<"--"<<rp->nodeSpeedMax<<std::endl;
}

void InstallMobilityModel_RandomWalk2dMobilityModel_Sim(RoutingProtocols* rp){
    MobilityHelper mobility;

    mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                            "X", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeXMin) + "|Max=" + std::to_string(rp->simScopeXMax) + "]"),
                            "Y", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeYMin) + "|Max=" + std::to_string(rp->simScopeYMax) + "]"));

    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            "Bounds", RectangleValue(Rectangle(rp->simScopeXMin, rp->simScopeXMax, rp->simScopeYMin, rp->simScopeYMax)),
                            "Speed", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->nodeSpeedMin) + "|Max=" + std::to_string(rp->nodeSpeedMax) + "]"),
                            "Direction", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->nodeDirectionMin) + "|Max=" + std::to_string(rp->nodeDirectionMax) + "]"));

    mobility.Install(rp->nodes);
}

void InstallMobilityModel_RandomWaypointMobilityModel_Sim(RoutingProtocols* rp){
    Ptr<PositionAllocator> positionAlloc = CreateObject<RandomBoxPositionAllocator>();
    positionAlloc->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeXMin) + "|Max=" + std::to_string(rp->simScopeXMax) + "]"));
    positionAlloc->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeYMin) + "|Max=" + std::to_string(rp->simScopeYMax) + "]"));
    positionAlloc->SetAttribute("Z", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeZMin) + "|Max=" + std::to_string(rp->simScopeZMax) + "]"));

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                "Speed", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->nodeSpeedMin) + "|Max=" + std::to_string(rp->nodeSpeedMax) + "]"),
                                "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2.0]"),
                                "PositionAllocator", PointerValue(positionAlloc));

    mobility.Install(rp->nodes);

    NS_LOG_UNCOND("simScope: "<< rp->simScopeXMax<<" X "<<rp->simScopeYMax<<" X "<<rp->simScopeZMax);
}

void InstallMobilityModel_GaussMarkovMobilityModel_Sim(RoutingProtocols* rp){
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel", "Bounds",
                              BoxValue(Box(rp->simScopeXMin, rp->simScopeXMax, rp->simScopeYMin, rp->simScopeYMax, rp->simScopeZMin, rp->simScopeZMax)),
                              "TimeStep", TimeValue(Seconds(1)),
                              "Alpha", DoubleValue(0.85),
                              "MeanVelocity",
                              StringValue("ns3::ConstantRandomVariable[Constant="+std::to_string(rp->nodeSpeedMax)+"]"),
                              "MeanDirection",
                              StringValue("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                              "MeanPitch",
                              StringValue("ns3::UniformRandomVariable[Min=0.05|Max=0.05]"),
                              "NormalVelocity",
                              StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
                              "NormalDirection",
                              StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
                              "NormalPitch",
                              StringValue("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]"));
    mobility.SetPositionAllocator(
        "ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeXMin) + "|Max=" + std::to_string(rp->simScopeXMin) + "]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeYMin) + "|Max=" + std::to_string(rp->simScopeYMax) + "]"),
        "Z", StringValue("ns3::UniformRandomVariable[Min=" + std::to_string(rp->simScopeZMin) + "|Max=" + std::to_string(rp->simScopeZMax) + "]"));
    mobility.Install(rp->nodes);
}

/*网络设备：wifi设备、物理层、信道、mac层*/
void CreateDevices_Sim(RoutingProtocols* rp){
    std::cout<<"commonrange : "<<rp->commRange<<std::endl;

    WifiHelper wifi;
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    WifiMacHelper wifiMac;

    wifiPhy.Set("RxGain", DoubleValue(-10));
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(rp->commRange));
    wifiPhy.SetChannel(wifiChannel.Create());

    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                                 StringValue("OfdmRate6Mbps"),
                                 "RtsCtsThreshold", UintegerValue(rp->packetSizeMax));

    wifiMac.SetType("ns3::AdhocWifiMac","QosSupported", BooleanValue(false));

    rp->devices = wifi.Install(wifiPhy, wifiMac, rp->nodes);
}

/*能量模型安装*/
void InstallEnergyModel_Sim(RoutingProtocols* rp){
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100.0));
    EnergySourceContainer sources = basicSourceHelper.Install (rp->nodes);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.067));
    radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.046));
    radioEnergyHelper.Set ("IdleCurrentA", DoubleValue(0.01));
    radioEnergyHelper.Set ("CcaBusyCurrentA", DoubleValue(0.01));
    radioEnergyHelper.Set ("SwitchingCurrentA", DoubleValue(0.01));
    radioEnergyHelper.Set ("SleepCurrentA", DoubleValue(0.001));

    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (rp->devices, sources);
}

/*网络协议栈：安装路由算法至节点*/
void InstallInternetStack_Sim(RoutingProtocols* rp){
    InternetStackHelper stack;
    Ipv4AddressHelper address;

    if(rp->algorithm == "mmgpsr"){
        std::cout<<"Routing algorithm : MMGPSR"<<std::endl;
        MMGpsrHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(rp->nodes);
    }else if(rp->algorithm == "gpsr"){
        std::cout<<"Routing algorithm : GPSR"<<std::endl;
        GpsrHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(rp->nodes);
    }else if(rp->algorithm == "aodv"){
        std::cout<<"Routing algorithm : AODV"<<std::endl;
        AodvHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(rp->nodes);
    }else if(rp->algorithm == "dsdv"){
        std::cout<<"Routing algorithm : DSDV"<<std::endl;
        DsdvHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(rp->nodes);
    }else if(rp->algorithm == "olsr"){
        std::cout<<"Routing algorithm : OLSR"<<std::endl;
        OlsrHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(rp->nodes);
    }else if(rp->algorithm == "intelligent-protocol-stack"){
        std::cout<<"Routing algorithm : IntelligentProtocolStack"<<std::endl;
        IntelligentProtocolStackHelper route;
        stack.SetRoutingHelper(route);
        stack.Install(rp->nodes);
    }

    address.SetBase("10.0.0.0", "255.255.0.0");
    rp->interfaces = address.Assign(rp->devices);
}

/*应用：节点通信*/
void InstallApplications_Sim(RoutingProtocols* rp){
    std::cout<<"Seed : "<<rp->seed<<std::endl;

    Time interPacketInterval = Seconds(rp->interPacketIntervalTime);
    UdpEchoServerHelper server1(rp->port);
    ApplicationContainer apps;
    srand(rp->seed);
    int source;
    int dest;
    std::vector<std::pair<int, int>>::iterator it;
    bool exist;
    float start_app;

    for (int i = 0; i < rp->nodesPairs; i++){
        source = 0;
        dest = 0;
        exist = true;
        start_app = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // 循环直到找到源节点≠目的节点且节点对不重复的组合
        while (source == dest || exist == true){
            source = rand() % rp->size;
            dest = rand() % rp->size;

            // 检查源节点和目的节点是否相同
            if (source == dest){
                exist = true;
                continue;
            }

            // 检查该节点对是否已存在
            it = std::find(rp->source_dest_pairs.begin(), rp->source_dest_pairs.end(), std::make_pair(source, dest));
            if (it != rp->source_dest_pairs.end()){
                exist = true;  // 已存在，继续查找
            } else {
                exist = false;  // 不重复，退出循环
            }
        }

        // 确保源节点和目的节点不重复不相同
        rp->source_dest_pairs.push_back(std::make_pair(source, dest));

        std::cout<<"Communication nodes: "<<source<<"-->"<<dest<<std::endl;


        Ipv4Address destAddress = rp->interfaces.GetAddress(dest);

        UdpEchoClientHelper client(destAddress, rp->port);
        client.SetAttribute("MaxPackets", UintegerValue(rp->maxPacketCount));
        client.SetAttribute("Interval", TimeValue(interPacketInterval));
        client.SetAttribute("PacketSize", UintegerValue(rp->packetSizeMax));

        apps = client.Install(rp->nodes.Get(source));
        apps.Start(Seconds(start_app));
        apps.Stop(Seconds(rp->totalTime - 0.1));
    }
}

/*初始化路由协议：仅gpsr需要*/
void InitRoutingProtocol_Sim(RoutingProtocols* rp){
    if(rp->algorithm == "mmgpsr"){
        MMGpsrHelper route;
        route.Install();
    }else if(rp->algorithm == "gpsr"){
        GpsrHelper route;
        route.Install();
    }else if(rp->algorithm == "intelligent-protocol-stack"){
        IntelligentProtocolStackHelper route;
        route.Install();
    }
}

void RunAndCal_Sim(RoutingProtocols* rp){
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    AnimationInterface anim("gpsr_anim.xml");
    anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(rp->totalTime));
    rp->m_CSVfileName = rp->algorithm + ".csv";
    AsciiTraceHelper ascii;
    MobilityHelper::EnableAscii(ascii.CreateFileStream("Fanet_Routing_Comparison.mob"), 2);
    Simulator::Stop(Seconds(rp->totalTime));
    Simulator::Run();


    uint32_t SentPackets = 0;
    uint32_t ReceivedPackets = 0;
    int j = 0;
    float AvgThroughput = 0;
    double Delay = 0;
    double hopCount = 0;
    double convergenceTime = 0;
    bool convergeFlag = true;
    double flow_delay=0;
    double test_TotalThroughput = 0;

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin(); iter != stats.end(); ++iter){
        SentPackets += (iter->second.txPackets);
        ReceivedPackets += (iter->second.rxPackets);
        AvgThroughput += (iter->second.rxBytes * 8.0 / (iter->second.timeFirstRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1024);

        if (iter->second.rxPackets != 0){
            j = j + 1;
            test_TotalThroughput += (iter->second.txBytes * 8.0 / (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds()) / 1024);
            Delay += (iter->second.delaySum.GetSeconds()/iter->second.rxPackets);
            hopCount += (iter->second.timesForwarded / iter->second.rxPackets + 1);
            flow_delay += (iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds());
            if(rp->calConvergeTime && convergeFlag && iter->second.timeFirstTxPacket.GetSeconds() >= 30){
                convergenceTime = (iter->second.timeFirstRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds())*1000;
                convergeFlag = false;
            }
        }
    }

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
    if(rp->calConvergeTime){
        NS_LOG_UNCOND("Convergence Time = " << std::fixed << std::setprecision(3) << convergenceTime << " ms");
    }
    NS_LOG_UNCOND("flow delay = "<<flow_delay << std::endl);
    NS_LOG_UNCOND("flow delay/ReceivedPackets = "<<flow_delay/ReceivedPackets << std::endl);

    WriteToFile_Sim(rp, avgDelay, deliveryRatio, avgThroughput, avgHopCount, convergenceTime, flow_delay, ReceivedPackets);

    monitor->SerializeToXmlFile("fanet-routing-gpsr.xml", true, true);

    Simulator::Destroy();
}

void WriteToFile_Sim(RoutingProtocols* rp, double avgDelay, double deliveryRatio,
                      double avgThroughput, double avgHopCount, double convergenceTime,
                      double flowDelay, uint32_t receivedPackets){
    struct stat info;
    std::string folderPath = "RoutingResult";
    std::string fileName = rp->algorithm + "_speed" + std::to_string(rp->nodeSpeedMin) + "-" + std::to_string(rp->nodeSpeedMax) + "_results.txt";
    if (stat(folderPath.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        if (mkdir(folderPath.c_str(), 0777) != 0) {
            return;
        }
    }

    std::string filePath = folderPath + "/" + fileName;

    std::ofstream outFile(filePath, std::ios::app);

    if (!outFile.is_open()){
        std::cerr << "Error opening results file!" << std::endl;
        return;
    }

    outFile.seekp(0, std::ios::end);
    if(outFile.tellp() == 0) {
        outFile << "seed\tdelay(ms)\tdelivertRatio(%)\tthroughput(kbps)\tavgHopCount\tconvergenceTime\tflowDelay\n";
    }
    double flowDelayAvg = (receivedPackets > 0) ? (flowDelay / receivedPackets * 1000) : 0;
    outFile <<rp->seed<<"\t"
    << avgDelay<< "\t"
        << deliveryRatio << "\t"
        << avgThroughput << "\t"
        << avgHopCount << "\t"
        << convergenceTime << "\t"
        << flowDelayAvg << "\n";
    outFile.close();
}
