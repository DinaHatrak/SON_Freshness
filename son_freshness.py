
import csv
import random
import math
from pathlib import Path
from collections import defaultdict
from statistics import mean, median

# SELF-ORGANIZING NETWORKS
# V3: Budgeted Adaptive Recovery
# No external Python libraries required.

SIZES = (25, 50, 100, 200)

SEEDS_DEV = (11, 29, 47)
SEEDS_TEST = (101, 203, 307)

SCENARIOS = (
    "stable", "lossy", "churn", "outage", "stress"
)

POLICIES = (
    "fixed_fast",
    "fixed_slow",
    "freshness_v2",
    "budgeted_v3"
)

DURATION = 150
WARMUP = 20


def graph(n, seed):
    rng = random.Random(10301 * seed + 53 * n)
    adj = [set() for _ in range(n)]

    for u in range(n):
        v = (u + 1) % n
        adj[u].add(v)
        adj[v].add(u)

    while sum(map(len, adj)) // 2 < 2 * n:
        u, v = rng.sample(range(n), 2)
        adj[u].add(v)
        adj[v].add(u)

    return adj


def rewire(adj, rng):
    edges = [
        (u, v)
        for u in range(len(adj))
        for v in adj[u]
        if u < v
    ]

    removed = rng.sample(
        edges, max(1, round(.15 * len(edges)))
    )

    for u, v in removed:
        adj[u].remove(v)
        adj[v].remove(u)

    old = set(removed)
    done = 0

    while done < len(removed):
        u, v = sorted(
            rng.sample(range(len(adj)), 2)
        )

        if v in adj[u] or (u, v) in old:
            continue

        adj[u].add(v)
        adj[v].add(u)
        done += 1


def received(seed, t, u, v, kind, drop):
    x = (
        seed * 2246822519
        + (t + 1) * 3266489917
        + (u + 1) * 668265263
        + (v + 1) * 374761393
        + kind * 1274126177
    ) & 0xffffffff

    x ^= x >> 16
    x = (x * 0x7FEB352D) & 0xffffffff
    x ^= x >> 15
    x = (x * 0x846CA68B) & 0xffffffff
    x ^= x >> 16

    return x / 4294967296 >= drop


def run(n, seed, scenario, policy):
    adj = graph(n, seed)
    rng = random.Random(7919 * seed + 97 * n)

    offline = set(
        random.Random(3001 * seed + n).sample(
            range(n), max(1, round(.10 * n))
        )
    )

    drop = {
        "stable": .02,
        "lossy": .25,
        "churn": .05,
        "outage": .05,
        "stress": .25
    }[scenario]

    churn = scenario in ("churn", "stress")
    outage = scenario in ("outage", "stress")

    period = {
        "fixed_fast": 2,
        "fixed_slow": 4,
        "freshness_v2": 4,
        "budgeted_v3": 5
    }[policy]

    hold = {
        "fixed_fast": 6,
        "fixed_slow": 12,
        "freshness_v2": 8
    }.get(policy)

    # Each neighbor record stores:
    # last_seen, interval, sequence, missed, observed

    cache = [{} for _ in range(n)]
    last_probe = [{} for _ in range(n)]

    next_hello = [
        (seed + 3 * u) % period
        for u in range(n)
    ]

    last_hello = [-1000] * n
    fast_until = [-1] * n

    # Token-bucket control budget
    tokens = [3.0] * n

    # Local observed packet-loss statistics
    global_misses = [0] * n
    global_intervals = [0] * n

    seq = [0] * n
    urgent = [False] * n

    inbox = defaultdict(list)

    prev_links = set()
    prev_online = [True] * n

    down_pending = {}
    up_pending = {}

    down_delays = []
    up_delays = []

    c_down = 0
    c_up = 0

    tp = fp = fn = 0
    control = hello = probe = reply = active = 0

    for t in range(DURATION):

        # 1. Network changes
        if churn and t in (50, 90, 125):
            rewire(adj, rng)

        online = [
            not (
                outage
                and 75 <= t < 100
                and u in offline
            )
            for u in range(n)
        ]

        for u in range(n):

            if policy == "budgeted_v3":
                tokens[u] = min(
                    3.0, tokens[u] + 0.4
                )

            if not online[u] and prev_online[u]:
                cache[u].clear()
                last_probe[u].clear()
                urgent[u] = False

            elif online[u] and not prev_online[u]:
                next_hello[u] = t
                last_hello[u] = -1000

            prev_online[u] = online[u]

        links = {
            (u, v)
            for u in range(n)
            if online[u]
            for v in adj[u]
            if online[v]
        }

        # 2. Ground-truth tracking for evaluation
        # Ground truth is NOT given to the protocol.

        if t:
            for pair in sorted(prev_links - links):

                if pair in up_pending:
                    up_pending.pop(pair)
                    c_up += 1

                if t >= WARMUP and online[pair[0]]:
                    if pair[1] in cache[pair[0]]:
                        down_pending[pair] = t
                    else:
                        down_delays.append(0)

            for pair in sorted(links - prev_links):

                if pair in down_pending:
                    down_pending.pop(pair)
                    c_down += 1

                if t >= WARMUP:
                    if pair[1] in cache[pair[0]]:
                        up_delays.append(0)
                    else:
                        up_pending[pair] = t

        prev_links = links

        # 3. Control message transmission

        def emit(kind, u, v=-1, adv=0, count=0):
            nonlocal control, hello, probe, reply

            if t >= WARMUP:
                control += 1

                if kind == 0:
                    hello += 1
                elif kind == 1:
                    probe += 1
                else:
                    reply += 1

            if kind != 0 and (u, v) in links:
                inbox[t + 1].append(
                    (kind, u, v, t, adv, count)
                )

        # 4. Update local neighbor information

        def note(receiver, sender, adv, count):
            old = cache[receiver].get(sender)

            if policy == "budgeted_v3":

                if old:
                    (
                        past, old_period, old_seq,
                        missing, total
                    ) = old

                    gap = max(1, count - old_seq)

                    missing += max(0, gap - 1)
                    total += gap

                    global_misses[receiver] += max(
                        0, gap - 1
                    )
                    global_intervals[receiver] += gap

                else:
                    missing = 0
                    total = 0

                    fast_until[receiver] = max(
                        fast_until[receiver], t + 8
                    )

                    urgent[receiver] = True

                cache[receiver][sender] = [
                    t, adv, count, missing, total
                ]

            else:
                if (
                    old is None
                    and policy == "freshness_v2"
                ):
                    urgent[receiver] = True

                cache[receiver][sender] = [
                    t, adv, count, 0, 0
                ]

        # 5. Process received messages

        for kind, u, v, sent, adv, count in inbox.pop(
            t, []
        ):

            if (u, v) not in links:
                continue

            if not received(
                seed, sent, u, v, kind, drop
            ):
                continue

            note(v, u, adv, count)

            if kind == 1:
                emit(2, v, u)

        # 6. Adaptive neighbor expiration

        for u in range(n):

            if not online[u]:
                continue

            gone = []

            for v, item in cache[u].items():

                (
                    last_seen, adv, last_seq,
                    missed, total
                ) = item

                if policy == "budgeted_v3":

                    global_p = (
                        (1.0 + global_misses[u])
                        / (10.0 + global_intervals[u])
                    )

                    p = max(
                        (1.0 + missed) / (12.0 + total),
                        global_p
                    )

                    if p < .16:
                        ttl = max(
                            9, math.ceil(1.8 * adv)
                        )
                    else:
                        ttl = max(
                            11, math.ceil(2.1 * adv)
                        )

                    ttl = min(22, ttl)

                else:
                    ttl = hold

                if t - last_seen > ttl:
                    gone.append(v)

            for v in gone:
                del cache[u][v]
                last_probe[u].pop(v, None)

            if gone:

                if policy == "freshness_v2":
                    urgent[u] = True

                if policy == "budgeted_v3":
                    fast_until[u] = max(
                        fast_until[u], t + 8
                    )
                    urgent[u] = True

        # 7. V2 reference probing

        if policy == "freshness_v2":

            for u in range(n):

                if not online[u]:
                    continue

                for v, item in sorted(
                    cache[u].items()
                ):

                    last = item[0]

                    if (
                        t - last >= 5
                        and t - last_probe[u].get(
                            v, -1000
                        ) >= 8
                    ):
                        last_probe[u][v] = t
                        emit(1, u, v)

        # 8. V3: Budgeted adaptive scheduling

        for u in range(n):

            if not online[u]:
                continue

            due = t >= next_hello[u]

            early = (
                urgent[u]
                and t - last_hello[u] >= 2
                and policy in (
                    "freshness_v2", "budgeted_v3"
                )
            )

            if not (due or early):
                continue

            if (
                policy == "budgeted_v3"
                and tokens[u] < 1.0
            ):
                continue

            if policy == "budgeted_v3":
                tokens[u] -= 1.0

            global_p = (
                (1.0 + global_misses[u])
                / (10.0 + global_intervals[u])
            )

            # Increase refresh frequency only
            # when locally observed loss is high.

            base_period = (
                3
                if (
                    global_intervals[u] >= 8
                    and global_p >= .16
                )
                else 5
            )

            if policy == "budgeted_v3":
                new_period = (
                    2 if t < fast_until[u]
                    else base_period
                )
            else:
                new_period = period

            seq[u] += 1

            emit(
                0, u,
                adv=new_period,
                count=seq[u]
            )

            last_hello[u] = t
            next_hello[u] = t + new_period
            urgent[u] = False

            for v in sorted(adj[u]):
                if online[v]:
                    inbox[t + 1].append(
                        (
                            0, u, v, t,
                            new_period, seq[u]
                        )
                    )

        # 9. Recovery delay measurements

        for pair, start in list(
            down_pending.items()
        ):
            u, v = pair

            if v not in cache[u]:
                down_delays.append(t - start)
                del down_pending[pair]

        for pair, start in list(
            up_pending.items()
        ):
            u, v = pair

            if v in cache[u]:
                up_delays.append(t - start)
                del up_pending[pair]

        # 10. Neighbor-state evaluation

        if t >= WARMUP:

            active += sum(online)

            for u in range(n):

                if not online[u]:
                    continue

                actual = {
                    v
                    for v in adj[u]
                    if online[v]
                }

                found = set(cache[u])

                tp += len(actual & found)
                fp += len(found - actual)
                fn += len(actual - found)

    c_down += len(down_pending)
    c_up += len(up_pending)

    precision = (
        tp / (tp + fp) if tp + fp else 0
    )

    recall = (
        tp / (tp + fn) if tp + fn else 0
    )

    f1 = (
        2 * tp / (2 * tp + fp + fn)
        if 2 * tp + fp + fn else 0
    )

    return dict(
        scenario=scenario,
        nodes=n,
        seed=seed,
        policy=policy,

        precision=round(precision, 5),
        recall=round(recall, 5),
        f1=round(f1, 5),

        stale=round(
            fp / (tp + fp), 5
        ) if tp + fp else 0,

        tx_per_node_min=round(
            60 * control / active, 3
        ),

        hello_tx=hello,
        probe_tx=probe,
        reply_tx=reply,

        down_delay_median=round(
            median(down_delays), 2
        ) if down_delays else "",

        up_delay_median=round(
            median(up_delays), 2
        ) if up_delays else "",

        down_events=len(down_delays),
        up_events=len(up_delays),

        down_censored=c_down,
        up_censored=c_up
    )


def main():

    print("=" * 65)
    print("SON - BUDGETED ADAPTIVE RECOVERY V3")
    print("=" * 65)

    rows = []

    for phase, seeds in (
        ("exploratory", SEEDS_DEV),
        ("held_out", SEEDS_TEST)
    ):

        for sc in SCENARIOS:
            for n in SIZES:
                for s in seeds:
                    for p in POLICIES:

                        result = run(
                            n, s, sc, p
                        )

                        rows.append({
                            "phase": phase,
                            **result
                        })

    output = (
        Path(__file__).resolve().parent
        / "results_v3.csv"
    )

    with output.open(
        "w",
        newline="",
        encoding="utf-8"
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=list(rows[0])
        )

        writer.writeheader()
        writer.writerows(rows)

    print(
        f"\nSaved {len(rows)} runs to {output}"
    )

    for phase in ("exploratory", "held_out"):

        print("\n" + phase.upper())

        print(
            f"{'Scenario':<10} "
            f"{'Policy':<16} "
            f"{'F1':>8} "
            f"{'Stale':>8} "
            f"{'TX/min':>9} "
            f"{'Down(s)':>9} "
            f"{'Up(s)':>8}"
        )

        for sc in SCENARIOS:
            for p in POLICIES:

                selected = [
                    r for r in rows
                    if r["phase"] == phase
                    and r["scenario"] == sc
                    and r["policy"] == p
                ]

                def metric(key):
                    values = [
                        r[key] for r in selected
                        if r[key] != ""
                    ]

                    return (
                        mean(values)
                        if values else float("nan")
                    )

                print(
                    f"{sc:<10} "
                    f"{p:<16} "
                    f"{mean(r['f1'] for r in selected):8.3f} "
                    f"{mean(r['stale'] for r in selected):8.3f} "
                    f"{mean(r['tx_per_node_min'] for r in selected):9.1f} "
                    f"{metric('down_delay_median'):9.1f} "
                    f"{metric('up_delay_median'):8.1f}"
                )

    print("\n" + "=" * 65)
    print("V3 COMPLETED")
    print("=" * 65)

    print(
        "Abstract one-hop control-plane simulation."
    )
    print(
        "No routing throughput or physical "
        "energy measurements."
    )


if __name__ == "__main__":
    main()
