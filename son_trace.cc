// SON TRACE - ns-3.49 scratch program (copy to ns-3.49/scratch/son_trace.cc)
//
// Purpose: direct timestamp evidence for the mechanism.
//  - One node (the "leaver", default 12 = grid centre of a 5x5 grid) leaves at T_CHANGE and returns at T_RESTORE.
//  - Every neighbour ("observer") logs:
//      last HELLO received from the leaver before it left, and the validity time (VTime) carried by it
//      predicted_expiry = last_rx + VTime
//      purge_time       = first time the leaver is no longer a SYM neighbour (50 ms polling)
//      first_rx_after   = first HELLO received from the leaver after it returns
//      relist_time      = first time the leaver is again a SYM neighbour
//  - No application traffic, no fading: only OLSR control traffic, to isolate the timers.
// Same policies and the same NeighbHoldFactor patch as son_study2.cc.

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/olsr-module.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace ns3;

// Departure / return times are randomised per seed (set in main) so that they are
// NOT phase-locked to the HELLO timer grid. Same times for every policy with the same seed.
static double T_END = 95.0;
static double T_CHANGE = 45.0;
static double T_RESTORE = 70.0;
static constexpr double SPACING = 35.0;
static constexpr double RANGE_M = 65.0;
static constexpr double POLL = 0.05;

struct Obs
{
    uint32_t id = 0;
    bool prevListed = false;
    bool listedBeforeChange = false;
    double lastRx = -1;
    double lastV = -1;
    double firstRxAfter = -1;
    double purge = -1;
    double relist = -1;
};

class Trace
{
  public:
    Trace(std::vector<Ptr<olsr::RoutingProtocol>> p, Ipv4Address leaverAddr, std::vector<uint32_t> observers)
        : protocols(p),
          leaver(leaverAddr)
    {
        for (auto o : observers)
        {
            Obs s;
            s.id = o;
            obs[o] = s;
        }
    }

    void OnHello(uint32_t observer, const olsr::MessageList& messages)
    {
        auto it = obs.find(observer);
        if (it == obs.end())
        {
            return;
        }
        double now = Simulator::Now().GetSeconds();
        for (const auto& msg : messages)
        {
            if (msg.GetMessageType() != olsr::MessageHeader::HELLO_MESSAGE ||
                msg.GetOriginatorAddress() != leaver)
            {
                continue;
            }
            if (now < T_CHANGE)
            {
                it->second.lastRx = now;
                it->second.lastV = msg.GetVTime().GetSeconds();
            }
            else if (now >= T_RESTORE && it->second.firstRxAfter < 0)
            {
                it->second.firstRxAfter = now;
            }
        }
    }

    void Poll()
    {
        double now = Simulator::Now().GetSeconds();
        if (now > T_END - 0.5)
        {
            return;
        }

        for (auto& kv : obs)
        {
            Obs& s = kv.second;
            bool listed = false;
            for (const auto& t : protocols[s.id]->GetNeighbors())
            {
                if (t.neighborMainAddr == leaver && t.status == olsr::NeighborTuple::STATUS_SYM)
                {
                    listed = true;
                    break;
                }
            }

            if (now < T_CHANGE)
            {
                s.listedBeforeChange = listed;
            }
            else if (now < T_RESTORE)
            {
                if (s.prevListed && !listed && s.purge < 0)
                {
                    s.purge = now;
                }
            }
            else if (listed && s.relist < 0)
            {
                s.relist = now;
            }
            s.prevListed = listed;
        }

        Simulator::Schedule(Seconds(POLL), &Trace::Poll, this);
    }

    std::map<uint32_t, Obs> obs;

  private:
    std::vector<Ptr<olsr::RoutingProtocol>> protocols;
    Ipv4Address leaver;
};

class RxProbe
{
  public:
    RxProbe(Trace* t, uint32_t o)
        : trace(t),
          observer(o)
    {
    }

    void Rx(const olsr::PacketHeader& header, const olsr::MessageList& messages)
    {
        (void)header;
        trace->OnHello(observer, messages);
    }

  private:
    Trace* trace;
    uint32_t observer;
};

static Ptr<olsr::RoutingProtocol>
GetOlsr(Ptr<Node> node)
{
    auto root = node->GetObject<Ipv4>()->GetRoutingProtocol();
    auto direct = DynamicCast<olsr::RoutingProtocol>(root);
    if (direct)
    {
        return direct;
    }
    auto list = DynamicCast<Ipv4ListRouting>(root);
    if (list)
    {
        for (uint32_t k = 0; k < list->GetNRoutingProtocols(); ++k)
        {
            int16_t priority = 0;
            auto found = DynamicCast<olsr::RoutingProtocol>(list->GetRoutingProtocol(k, priority));
            if (found)
            {
                return found;
            }
        }
    }
    NS_ABORT_MSG("OLSR routing protocol not available");
    return nullptr;
}

static Vector
Position(uint32_t k, uint32_t columns)
{
    return Vector((k % columns) * SPACING, (k / columns) * SPACING, 0);
}

static void
MoveNode(Ptr<MobilityModel> m, Vector v)
{
    m->SetPosition(v);
}

struct PolicySpec
{
    double hello;
    double factor;
    bool valid;
};

static PolicySpec
ParsePolicy(const std::string& name)
{
    if (name == "olsr_default")
    {
        return {2.0, 3.0, true};
    }
    if (name == "olsr_slow")
    {
        return {5.0, 3.0, true};
    }
    double h = 0;
    double k = 0;
    if (std::sscanf(name.c_str(), "h%lf_k%lf", &h, &k) == 2 && h > 0 && k >= 1.0)
    {
        return {h, k, true};
    }
    return {0, 0, false};
}

static const char* CSV_HEADER =
    "policy,hello_s,hold_factor,seed,observer,listed_before,last_rx_s,last_vtime_s,"
    "predicted_expiry_s,purge_s,purge_minus_predicted_s,first_rx_after_s,relist_s,"
    "arrival_first_rx_delay_s,arrival_relist_delay_s,depart_s,restore_s\n";

int
main(int argc, char** argv)
{
    std::string policy = "olsr_default";
    uint32_t seed = 3001;
    uint32_t leaverIdx = 12;
    std::string output = "trace_one.csv";
    bool headerOnly = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("policy", "olsr_default|olsr_slow|h<hello>_k<factor>", policy);
    cmd.AddValue("seed", "RNG run number", seed);
    cmd.AddValue("leaver", "Index of the node that leaves and returns", leaverIdx);
    cmd.AddValue("output", "CSV file (rows appended)", output);
    cmd.AddValue("header", "Write CSV header and exit", headerOnly);
    cmd.Parse(argc, argv);

    {
        // low-discrepancy offsets (golden-ratio and plastic-number sequences)
        double f1 = std::fmod(seed * 0.6180339887498949, 1.0);
        double f2 = std::fmod(seed * 0.7548776662466927, 1.0);
        T_CHANGE = 40.0 + 30.0 * f1;          // leaves in [40, 70)
        T_RESTORE = T_CHANGE + 30.0 + 30.0 * f2; // returns 30-60 s later
        T_END = T_RESTORE + 40.0;
    }

    if (headerOnly)
    {
        std::ofstream h(output);
        h << CSV_HEADER;
        return 0;
    }

    PolicySpec spec = ParsePolicy(policy);
    if (!spec.valid)
    {
        std::cerr << "Unknown policy: " << policy << "\n";
        return 2;
    }

    const uint32_t n = 25;
    const uint32_t columns = 5;

    RngSeedManager::SetSeed(12345);
    RngSeedManager::SetRun(seed);

    NodeContainer nodes;
    nodes.Create(n);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("DsssRate1Mbps"),
                                 "ControlMode",
                                 StringValue("DsssRate1Mbps"));

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent",
                               DoubleValue(3.0),
                               "ReferenceLoss",
                               DoubleValue(40.05));
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange",
                               DoubleValue(RANGE_M));
    Ptr<YansWifiChannel> wifiChannel = channel.Create();

    YansWifiPhyHelper phy;
    phy.SetChannel(wifiChannel);

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < n; ++i)
    {
        positions->Add(Position(i, columns));
    }

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    OlsrHelper olsr;
    olsr.Set("HelloInterval", TimeValue(Seconds(spec.hello)));
    olsr.Set("TcInterval", TimeValue(Seconds(5)));
    if (std::fabs(spec.factor - 3.0) > 1e-9)
    {
        olsr.Set("NeighbHoldFactor", DoubleValue(spec.factor));
    }

    Ipv4ListRoutingHelper routes;
    routes.Add(olsr, 100);

    InternetStackHelper internet;
    internet.SetRoutingHelper(routes);
    internet.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.0.0");
    auto interfaces = address.Assign(devices);

    int64_t stream = 0;
    stream += wifi.AssignStreams(devices, stream);
    stream += channel.AssignStreams(wifiChannel, stream);
    stream += olsr.AssignStreams(nodes, stream);

    std::vector<Ptr<olsr::RoutingProtocol>> protocols;
    for (uint32_t i = 0; i < n; ++i)
    {
        protocols.push_back(GetOlsr(nodes.Get(i)));
    }

    // Observers: nodes within radio range of the leaver's initial position
    Vector leaverPos = Position(leaverIdx, columns);
    std::vector<uint32_t> observers;
    for (uint32_t i = 0; i < n; ++i)
    {
        if (i != leaverIdx && CalculateDistance(Position(i, columns), leaverPos) <= RANGE_M)
        {
            observers.push_back(i);
        }
    }

    Trace trace(protocols, interfaces.GetAddress(leaverIdx), observers);

    std::vector<std::unique_ptr<RxProbe>> probes;
    for (auto o : observers)
    {
        probes.push_back(std::make_unique<RxProbe>(&trace, o));
        protocols[o]->TraceConnectWithoutContext("Rx",
                                                 MakeCallback(&RxProbe::Rx, probes.back().get()));
    }

    Ptr<MobilityModel> leaverMob = nodes.Get(leaverIdx)->GetObject<MobilityModel>();
    Simulator::Schedule(Seconds(T_CHANGE),
                        &MoveNode,
                        leaverMob,
                        Vector(leaverPos.x + 2000.0, leaverPos.y + 2000.0, 0));
    Simulator::Schedule(Seconds(T_RESTORE), &MoveNode, leaverMob, leaverPos);

    Simulator::Schedule(Seconds(20.0), &Trace::Poll, &trace);

    Simulator::Stop(Seconds(T_END));
    Simulator::Run();

    std::ofstream out(output, std::ios::app);
    if (!out)
    {
        std::cerr << "Cannot write " << output << "\n";
        Simulator::Destroy();
        return 3;
    }
    out << std::fixed << std::setprecision(3);
    for (const auto& kv : trace.obs)
    {
        const Obs& s = kv.second;
        double predicted = (s.lastRx >= 0 && s.lastV >= 0) ? s.lastRx + s.lastV : -1.0;
        out << policy << "," << spec.hello << "," << spec.factor << "," << seed << "," << s.id << ","
            << (s.listedBeforeChange ? 1 : 0) << "," << s.lastRx << "," << s.lastV << ","
            << predicted << "," << s.purge << ","
            << ((predicted >= 0 && s.purge >= 0) ? s.purge - predicted : -999.0) << ","
            << s.firstRxAfter << "," << s.relist << ","
            << (s.firstRxAfter >= 0 ? s.firstRxAfter - T_RESTORE : -1.0) << ","
            << (s.relist >= 0 ? s.relist - T_RESTORE : -1.0) << "," << T_CHANGE << ","
            << T_RESTORE << "\n";
    }
    out.close();

    Simulator::Destroy();
    return 0;
}
