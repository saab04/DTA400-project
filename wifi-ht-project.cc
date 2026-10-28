/*
 * Copyright (c) 2010 IITP RAS
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Pavel Boyko <boyko@iitp.ru>
 *
 * Classical hidden terminal problem and its RTS/CTS solution.
 *
 * Topology: [node 0] <-- 50 dB --> [node 1] <-- 50 dB --> [node 2]
 */

#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/on-off-helper.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/string.h"
#include "ns3/udp-echo-helper.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include <vector>
#include <string>
#include <cstdlib>

using namespace ns3;

/**
 * Run a single 10-second hidden terminal experiment.
 *
 * @param enableCtsRts     If true, RTS/CTS is activated (threshold 100 bytes).
 * @param wifiManager      The WiFi rate control manager to use (fixed to "Arf").
 * @param lossDb           The symmetrical signal loss in decibels to the center node.
 * @param dataRateStr      The transmission data rate for Flow 1 (Node 0 -> Node 1).
 * @param skewedDataRateStr The slightly modified data rate for Flow 2 (Node 2 -> Node 1).
 * @param folder           The directory where the output XML files will be stored.
 */
void
experiment(bool enableCtsRts, std::string wifiManager, double lossDb, 
           std::string dataRateStr, std::string skewedDataRateStr, std::string folder)
{
    // 0. Configure RTS/CTS threshold based on user input
    UintegerValue ctsThr = (enableCtsRts ? UintegerValue(100) : UintegerValue(2200));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

    // 1. Create the 3 network nodes (Node 0, Node 1, Node 2)
    NodeContainer nodes;
    nodes.Create(3);

    // 2. Assign static mobility models
    for (uint8_t i = 0; i < 3; ++i)
    {
        nodes.Get(i)->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
    }

    // 3. Define the propagation loss matrix to create the hidden terminal environment
    Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel>();
    lossModel->SetDefaultLoss(200); // Symmetrical isolation between Node 0 and Node 2
    lossModel->SetLoss(nodes.Get(0)->GetObject<MobilityModel>(),
                       nodes.Get(1)->GetObject<MobilityModel>(),
                       lossDb); // Loss between Node 0 and Node 1
    lossModel->SetLoss(nodes.Get(2)->GetObject<MobilityModel>(),
                       nodes.Get(1)->GetObject<MobilityModel>(),
                       lossDb); // Loss between Node 2 and Node 1

    // 4. Create and configure the physical wireless channel
    Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel>();
    wifiChannel->SetPropagationLossModel(lossModel);
    wifiChannel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

    // 5. Install wireless network interface cards
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::" + wifiManager + "WifiManager");
    
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel);
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac"); 
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // 6. Install the TCP/IP network stack and assign static IP addresses
    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.0.0.0");
    ipv4.Assign(devices);

    // 7. Configure Constant Bit Rate (CBR) traffic applications
    ApplicationContainer cbrApps;
    uint16_t cbrPort = 12345;
    OnOffHelper onOffHelper("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address("10.0.0.2"), cbrPort));
    onOffHelper.SetAttribute("PacketSize", UintegerValue(1400));
    onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    // Flow 1: Node 0 -> Node 1 (Starts at 1.0 second)
    onOffHelper.SetAttribute("DataRate", StringValue(dataRateStr));
    onOffHelper.SetAttribute("StartTime", TimeValue(Seconds(1.000000)));
    cbrApps.Add(onOffHelper.Install(nodes.Get(0)));

    // Flow 2: Node 2 -> Node 1 (Starts at 1.001 seconds to break perfect simulation synchronization)
    onOffHelper.SetAttribute("DataRate", StringValue(skewedDataRateStr));
    onOffHelper.SetAttribute("StartTime", TimeValue(Seconds(1.001)));
    cbrApps.Add(onOffHelper.Install(nodes.Get(2)));

    // Send single initial packets to populate ARP tables (Workaround for perfect ARP lack)
    uint16_t echoPort = 9;
    UdpEchoClientHelper echoClientHelper(InetSocketAddress(Ipv4Address("10.0.0.2"), echoPort));
    echoClientHelper.SetAttribute("MaxPackets", UintegerValue(1));
    echoClientHelper.SetAttribute("Interval", TimeValue(Seconds(0.1)));
    echoClientHelper.SetAttribute("PacketSize", UintegerValue(10));
    ApplicationContainer pingApps;

    echoClientHelper.SetAttribute("StartTime", TimeValue(Seconds(0.001)));
    pingApps.Add(echoClientHelper.Install(nodes.Get(0)));
    echoClientHelper.SetAttribute("StartTime", TimeValue(Seconds(0.006)));
    pingApps.Add(echoClientHelper.Install(nodes.Get(2)));

    // 8. Initialize FlowMonitor to collect statistics
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // 9. Execute the simulation loop for 10 seconds
    Simulator::Stop(Seconds(10));
    Simulator::Run();

    // 10. Export collected statistics to an external XML file
    std::string mode = (enableCtsRts ? "enabled" : "disabled");
    std::string fileName = folder + "/results-Arf-" + mode + "-" + dataRateStr + ".xml";
    flowmon.SerializeToXmlFile(fileName, true, true);

    // 11. Clear the simulation memory layout
    Simulator::Destroy();
}

int
main(int argc, char** argv)
{
    std::string wifiManager("Arf"); // Fixed to Arf to keep the analysis simple
    bool enableRtsCts = false; 
    std::string folder = "output"; 

    CommandLine cmd(__FILE__);
    cmd.AddValue("enableRtsCts", "True to enable RTS/CTS, False to disable", enableRtsCts);
    cmd.Parse(argc, argv);

    // Automatically create the output directory if it does not exist
    std::string mkdirCommand = "mkdir -p " + folder;
    std::system(mkdirCommand.c_str());

    // Keep path loss constant at 50 dB so nodes can always reach the center node
    double constantLossDb = 50.0;

    // Define the network loads to test (from 0.5 Mbps up to 4.0 Mbps)
    std::vector<long long> dataRates = {500000, 1000000, 2000000, 3000000, 4000000};
    
    std::cout << "Starting simulation with RTS/CTS: " << (enableRtsCts ? "Enabled" : "Disabled") << "\n";
    
    for (long long rate : dataRates) {
        // Construct the data rate string cleanly as bits per second
        std::string dataRateStr = std::to_string(rate) + "bps";
        
        // Calculate the microscopic skew using pure integer math to avoid string parsing bugs
        long long skewedRate = rate + (rate * 0.000366);
        std::string skewedRateStr = std::to_string(skewedRate) + "bps";

        std::cout << "Simulating at traffic load: " << (rate / 1000000.0) << " Mbps...\n";
        
        // Run the 10-second experiment for this specific load
        experiment(enableRtsCts, wifiManager, constantLossDb, dataRateStr, skewedRateStr, folder);
    }
    
    std::cout << "Simulation loop complete. XML data files saved in '" << folder << "/'.\n";

    return 0;
}

