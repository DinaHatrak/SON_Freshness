// SON STUDY V2 - ns-3.49 scratch program (copy to ns-3.49/scratch/son_study2.cc)
//
// Differences from son_study.cc:
//  1. ONE run per process (command-line scenario/policy/nodes/seed). This makes
//     RNG stream indices identical across policies for the same seed.
//  2. Explicit AssignStreams on Wi-Fi devices, channel and OLSR.
//  3. Policies are static (hello interval, neighbor-hold factor):
//       olsr_default = h2_k3, olsr_slow = h5_k3, or "h<hello>_k<factor>"
//     Factors other than 3 need the NeighbHoldFactor patch (see
//     olsr_patch_NeighbHoldFactor.txt). Factor 3 runs on stock ns-3.
//  4. Evaluator-only neighbor audit: counts stale / missing symmetric-neighbor
//     entries using node positions. Controllers do NOT see ground truth
//     (there are no controllers in this version).
//  5. CSV row has a "valid" column; failed sanity checks are kept, not dropped.

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/olsr-module.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace ns3;

static constexpr double T_END = 95.0;
static constexpr double T_START = 25.0;
static constexpr double T_CHANGE = 45.0;
static constexpr double T_RESTORE = 70.0;
static constexpr double T_STOP = 90.0;

static constexpr double SPACING = 35.0;
static constexpr double RANGE_M = 65.0;
static constexpr double UDP_PERIOD = 0.25;
static constexpr uint32_t PAYLOAD = 256;

struct Metrics
{
    uint64_t attempts = 0;
    uint64_t delivered = 0;
    uint64_t socketErrors = 0;

    uint64_t controlBytes = 0;
    uint64_t controlPackets = 0;
    uint64_t helloTx = 0;
    uint64_t helloRx = 0;

    double delayTotal = 0;

    std::array<uint64_t, 3> attemptedByPhase{};
    std::array<uint64_t, 3> deliveredByPhase{};

    // Evaluator-only neighbor audit (1 s sampling)
    double staleEntrySeconds = 0;
    double missingEntrySeconds = 0;
    bool staleSeen = false;
    bool missingSeen = false;
    double staleClearLag = -1;   // s after T_CHANGE until no stale entry (-1 = n/a or censored)
    double missingClearLag = -1; // s after T_RESTORE until no missing entry (-1 = n/a or censored)
};

static Metrics* g_stats = nullptr;

static int
Phase(double t)
{
    if (t < T_CHANGE)
    {
        return 0;
    }
    return t < T_RESTORE ? 1 : 2;
}

static double
Rate(uint64_t numerator, uint64_t denominator)
{
    if (!denominator)
    {
        return -1.0;
    }
    return 100.0 * numerator / denominator;
}

// ---------------- application traffic ----------------

static void
OnUdpRx(Ptr<const Packet> incoming, const Address& sender)
{
    (void)sender;

    Ptr<Packet> p = incoming->Copy();
    SeqTsHeader h;

    if (p->GetSize() < h.GetSerializedSize())
    {
        return;
    }

    p->RemoveHeader(h);

    double sent = h.GetTs().GetSeconds();

    if (sent < T_START || sent >= T_STOP)
    {
        return;
    }

    ++g_stats->delivered;
    ++g_stats->deliveredByPhase[Phase(sent)];
    g_stats->delayTotal += std::max(0.0, Simulator::Now().GetSeconds() - sent);
}

static void
SendUdp(Ptr<Socket> socket, uint32_t seq)
{
    double now = Simulator::Now().GetSeconds();

    if (now >= T_STOP)
    {
        return;
    }

    SeqTsHeader h;
    h.SetSeq(seq);

    Ptr<Packet> p = Create<Packet>(PAYLOAD - h.GetSerializedSize());
    p->AddHeader(h);

    ++g_stats->attempts;
    ++g_stats->attemptedByPhase[Phase(now)];

    if (socket->Send(p) < 0)
    {
        ++g_stats->socketErrors;
    }

    Simulator::Schedule(Seconds(UDP_PERIOD), &SendUdp, socket, seq + 1);
}

// ---------------- OLSR trace observer ----------------

class ControlObserver
{
  public:
    explicit ControlObserver(Metrics* m)
        : metrics(m)
    {
    }

    void Rx(const olsr::PacketHeader& header, const olsr::MessageList& messages)
    {
        (void)header;

        double now = Simulator::Now().GetSeconds();
        if (now < T_START || now >= T_STOP)
        {
            return;
        }

        for (const auto& msg : messages)
        {
            if (msg.GetMessageType() == olsr::MessageHeader::HELLO_MESSAGE)
            {
                ++metrics->helloRx;
            }
        }
    }

    void Tx(const olsr::PacketHeader& header, const olsr::MessageList& messages)
    {
        double now = Simulator::Now().GetSeconds();
        if (now < T_START || now >= T_STOP)
        {
            return;
        }

        ++metrics->controlPackets;
        metrics->controlBytes += header.GetPacketLength();

        for (const auto& msg : messages)
        {
            if (msg.GetMessageType() == olsr::MessageHeader::HELLO_MESSAGE)
            {
                ++metrics->helloTx;
            }
        }
    }

  private:
    Metrics* metrics;
};

// ---------------- evaluator-only neighbor audit ----------------

class NeighborAudit
{
  public:
    NeighborAudit(NodeContainer n,
                  std::vector<Ptr<olsr::RoutingProtocol>> p,
                  std::map<Ipv4Address, uint32_t> idx,
                  Metrics* m,
                  bool harshChannel)
        : nodes(n),
          protocols(p),
          index(idx),
          metrics(m),
          harsh(harshChannel)
    {
    }

    void Sample()
    {
        double now = Simulator::Now().GetSeconds();
        if (now >= T_STOP)
        {
            return;
        }

        uint64_t stale = 0;
        uint64_t missing = 0;
        const uint32_t n = nodes.GetN();

        for (uint32_t u = 0; u < n; ++u)
        {
            Vector pu = nodes.Get(u)->GetObject<MobilityModel>()->GetPosition();
            std::set<uint32_t> listed;

            for (const auto& t : protocols[u]->GetNeighbors())
            {
                if (t.status != olsr::NeighborTuple::STATUS_SYM)
                {
                    continue;
                }

                auto it = index.find(t.neighborMainAddr);
                if (it == index.end())
                {
                    continue;
                }

                listed.insert(it->second);

                Vector pv = nodes.Get(it->second)->GetObject<MobilityModel>()->GetPosition();
                if (CalculateDistance(pu, pv) > RANGE_M)
                {
                    ++stale; // listed as symmetric but physically out of range
                }
            }

            if (!harsh)
            {
                // Only meaningful without fading: in-range nodes should be listed.
                for (uint32_t v = 0; v < n; ++v)
                {
                    if (v == u || listed.count(v))
                    {
                        continue;
                    }
                    Vector pv = nodes.Get(v)->GetObject<MobilityModel>()->GetPosition();
                    if (CalculateDistance(pu, pv) <= RANGE_M)
                    {
                        ++missing;
                    }
                }
            }
        }

        metrics->staleEntrySeconds += static_cast<double>(stale);
        metrics->missingEntrySeconds += static_cast<double>(missing);

        if (now >= T_CHANGE && now < T_RESTORE)
        {
            if (stale > 0)
            {
                metrics->staleSeen = true;
            }
            else if (metrics->staleSeen && metrics->staleClearLag < 0)
            {
                metrics->staleClearLag = now - T_CHANGE;
            }
        }

        if (!harsh && now >= T_RESTORE)
        {
            if (missing > 0)
            {
                metrics->missingSeen = true;
            }
            else if (metrics->missingSeen && metrics->missingClearLag < 0)
            {
                metrics->missingClearLag = now - T_RESTORE;
            }
        }

        Simulator::Schedule(Seconds(1.0), &NeighborAudit::Sample, this);
    }

  private:
    NodeContainer nodes;
    std::vector<Ptr<olsr::RoutingProtocol>> protocols;
    std::map<Ipv4Address, uint32_t> index;
    Metrics* metrics;
    bool harsh;
};

// ---------------- helpers ----------------

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
            auto child = list->GetRoutingProtocol(k, priority);
            auto found = DynamicCast<olsr::RoutingProtocol>(child);
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
Topology(NodeContainer nodes, std::string scenario, uint32_t columns, bool changed)
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Vector p = Position(i, columns);

        if (changed)
        {
            if ((scenario == "outage" || scenario == "stress") && i % 13 == 0)
            {
                p.x += 2000.0;
                p.y += 2000.0;
            }
            else if ((scenario == "churn" || scenario == "stress") && i % 7 == 0)
            {
                p.x += 70.0;
                p.y += 25.0;
            }
        }

        nodes.Get(i)->GetObject<MobilityModel>()->SetPosition(p);
    }
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
    "scenario,policy,nodes,seed,valid,"
    "app_attempted,app_received,pdr_percent,delay_ms,"
    "pdr_before,pdr_during,pdr_after,socket_errors,"
    "olsr_packets,olsr_bytes,olsr_bytes_per_node_min,"
    "hello_tx,hello_rx,"
    "stale_sym_entry_s,missing_sym_entry_s,"
    "stale_clear_lag_s,missing_clear_lag_s\n";

// ---------------- single run ----------------

static int
RunOne(const std::string& scenario,
       const std::string& policy,
       uint32_t n,
       uint32_t seed,
       const std::string& outPath)
{
    PolicySpec spec = ParsePolicy(policy);
    if (!spec.valid)
    {
        std::cerr << "Unknown policy: " << policy << "\n";
        return 2;
    }

    const std::set<std::string> known = {"stable", "fading", "churn", "outage", "stress"};
    if (!known.count(scenario))
    {
        std::cerr << "Unknown scenario: " << scenario << "\n";
        return 2;
    }

    Metrics stats;
    g_stats = &stats;

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

    bool harsh = scenario == "fading" || scenario == "stress";

    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent",
                               DoubleValue(harsh ? 3.5 : 3.0),
                               "ReferenceLoss",
                               DoubleValue(40.05));

    if (harsh)
    {
        channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
    }

    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange",
                               DoubleValue(RANGE_M));

    Ptr<YansWifiChannel> wifiChannel = channel.Create();

    YansWifiPhyHelper phy;
    phy.SetChannel(wifiChannel);

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    uint32_t columns = static_cast<uint32_t>(std::ceil(std::sqrt(double(n))));

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
        // Requires the NeighbHoldFactor patch; aborts with a clear message if missing.
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

    // Fixed random-stream assignment (same for every policy).
    int64_t stream = 0;
    stream += wifi.AssignStreams(devices, stream);
    stream += channel.AssignStreams(wifiChannel, stream);
    stream += olsr.AssignStreams(nodes, stream);

    std::vector<std::unique_ptr<ControlObserver>> observers;
    std::vector<Ptr<olsr::RoutingProtocol>> protocols;
    std::map<Ipv4Address, uint32_t> index;

    for (uint32_t i = 0; i < n; ++i)
    {
        auto protocol = GetOlsr(nodes.Get(i));
        protocols.push_back(protocol);
        index[interfaces.GetAddress(i)] = i;

        observers.push_back(std::make_unique<ControlObserver>(&stats));

        protocol->TraceConnectWithoutContext(
            "Tx",
            MakeCallback(&ControlObserver::Tx, observers.back().get()));
        protocol->TraceConnectWithoutContext(
            "Rx",
            MakeCallback(&ControlObserver::Rx, observers.back().get()));
    }

    NeighborAudit audit(nodes, protocols, index, &stats, harsh);
    Simulator::Schedule(Seconds(T_START + 0.5), &NeighborAudit::Sample, &audit);

    // Identical application traffic for every policy.
    const uint16_t startPort = 9000;
    uint32_t flowCount = std::min<uint32_t>(5, n / 5);

    for (uint32_t j = 0; j < flowCount; ++j)
    {
        uint32_t src = j * 5;
        uint32_t dst = n - 1 - j * 3;

        if (src == dst)
        {
            dst = (dst + 1) % n;
        }

        uint16_t port = startPort + j;

        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));

        auto apps = sink.Install(nodes.Get(dst));
        apps.Start(Seconds(1));
        apps.Stop(Seconds(T_END));

        DynamicCast<PacketSink>(apps.Get(0))
            ->TraceConnectWithoutContext("Rx", MakeCallback(&OnUdpRx));

        auto socket = Socket::CreateSocket(nodes.Get(src), UdpSocketFactory::GetTypeId());
        socket->Bind();
        socket->Connect(InetSocketAddress(interfaces.GetAddress(dst), port));

        Simulator::Schedule(Seconds(T_START + 0.2 * j), &SendUdp, socket, 0u);
    }

    if (scenario == "churn" || scenario == "outage" || scenario == "stress")
    {
        Simulator::Schedule(Seconds(T_CHANGE), &Topology, nodes, scenario, columns, true);
        Simulator::Schedule(Seconds(T_RESTORE), &Topology, nodes, scenario, columns, false);
    }

    Simulator::Stop(Seconds(T_END));
    Simulator::Run();

    bool ok = stats.attempts > 0 && stats.delivered <= stats.attempts &&
              stats.controlPackets > 0 && stats.helloTx > 0 && stats.helloRx > 0;

    double ctrlRate = 60.0 * stats.controlBytes / (n * (T_STOP - T_START));

    std::ofstream out(outPath, std::ios::app);
    if (!out)
    {
        std::cerr << "Cannot write " << outPath << "\n";
        Simulator::Destroy();
        return 3;
    }

    out << std::fixed << std::setprecision(4);
    out << scenario << "," << policy << "," << n << "," << seed << "," << (ok ? 1 : 0) << ","
        << stats.attempts << "," << stats.delivered << ","
        << Rate(stats.delivered, stats.attempts) << ","
        << (stats.delivered ? 1000.0 * stats.delayTotal / stats.delivered : -1.0) << ","
        << Rate(stats.deliveredByPhase[0], stats.attemptedByPhase[0]) << ","
        << Rate(stats.deliveredByPhase[1], stats.attemptedByPhase[1]) << ","
        << Rate(stats.deliveredByPhase[2], stats.attemptedByPhase[2]) << ","
        << stats.socketErrors << "," << stats.controlPackets << "," << stats.controlBytes << ","
        << ctrlRate << "," << stats.helloTx << "," << stats.helloRx << ","
        << stats.staleEntrySeconds << ","
        << (harsh ? -1.0 : stats.missingEntrySeconds) << ","
        << stats.staleClearLag << "," << stats.missingClearLag << "\n";
    out.close();

    Simulator::Destroy();
    g_stats = nullptr;

    return 0;
}

int
main(int argc, char** argv)
{
    std::string scenario = "stable";
    std::string policy = "olsr_default";
    uint32_t nodes = 25;
    uint32_t seed = 2003;
    std::string output = "one_run.csv";
    bool headerOnly = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("scenario", "stable|fading|churn|outage|stress", scenario);
    cmd.AddValue("policy", "olsr_default|olsr_slow|h<hello>_k<factor>", policy);
    cmd.AddValue("nodes", "Number of nodes", nodes);
    cmd.AddValue("seed", "RNG run number", seed);
    cmd.AddValue("output", "CSV file (row appended)", output);
    cmd.AddValue("header", "Write CSV header to --output and exit", headerOnly);
    cmd.Parse(argc, argv);

    if (headerOnly)
    {
        std::ofstream h(output);
        h << CSV_HEADER;
        return 0;
    }

    return RunOne(scenario, policy, nodes, seed, output);
}
