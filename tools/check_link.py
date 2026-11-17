#!/usr/bin/env python3
"""Check a `motor_tool control ... --log file.csv` log for lost and late CAN replies.

    python3 tools/check_link.py /tmp/pos.csv [more.csv ...] [--hist]
    python3 tools/check_link.py --trace /tmp/trace.txt      (from motor_tool control / rate_test --trace)

--trace: every frame sent (tx) and received (rx) with a timestamp.  Replies carry no sequence number,
but a motor answers in order, so the k-th reply of a motor answers its k-th request.  Per motor:
  * requests / replies   exact counts; requests - replies = replies that NEVER arrived
  * latency              request -> its reply (p50 / p99 / max)
  * overlapped           replies that arrived only AFTER the next request to the same motor was sent
                         (late across the next command: a timing issue, not a loss)
  * lost                 if counts differ: which requests got no reply, and whether the next request was
                         already on its way (overlap) at that moment

Per motor:
  * no reply in time  cycles whose reply did not arrive within the timeout (STALE in the loop)
  * late              of those, replies that did arrive later (the driver found and dropped them)
  * never seen        no reply in time - late: replies that never arrived (CAN errors / lost frames)
  * longest run       most consecutive stale cycles (the loop stops all motors at 3)
Per cycle:
  * round trip        send all commands -> all replies (or timeout); percentiles, cycles over the period

Verdict (README "CAN 丢包"): a healthy, terminated bus must show 0 for every motor.  "late" means a
timing problem (round trip longer than the period: lower the rate or pipeline); "never seen" points at
the bus (termination, wiring, sample point) and must also show up as adapter errors.
Exit code: 0 = OK, 1 = lost or late replies, 2 = bad input.
"""
import csv
import sys


def percentile(sorted_values, p):
    if not sorted_values:
        return float("nan")
    return sorted_values[min(len(sorted_values) - 1, int(p * len(sorted_values)))]


def check(path, hist):
    meta = {}
    rows = []
    with open(path, newline="") as f:
        lines = []
        for line in f:
            if line.startswith("#"):
                for item in line[1:].split():
                    if "=" in item:
                        k, v = item.split("=", 1)
                        meta[k] = v
            else:
                lines.append(line)
    for r in csv.DictReader(lines):
        rows.append(r)
    if not rows or "stale" not in rows[0]:
        print(f"{path}: not a motor_tool control log (no 'stale' column)")
        return 2
    has_late = "late" in rows[0]
    has_rtt = "rtt_us" in rows[0]

    rate = float(meta["rate_hz"]) if "rate_hz" in meta else None
    times = sorted({float(r["t"]) for r in rows})
    if rate is None and len(times) > 1:
        rate = (len(times) - 1) / (times[-1] - times[0])
    period_us = 1e6 / rate if rate else float("nan")
    timeout_us = float(meta.get("timeout_us", "nan"))

    print(f"== {path}")
    print(f"   mode {meta.get('mode', '?')}, rate {rate:.1f} Hz (period {period_us:.0f} us), reply timeout "
          f"{timeout_us:.0f} us, {len(times)} cycles, {times[-1] - times[0]:.1f} s")

    ok = True
    by_id = {}
    for r in rows:
        by_id.setdefault(r["id"], []).append(r)
    for mid in sorted(by_id, key=int):
        rr = by_id[mid]
        stale = [int(r["stale"]) for r in rr]
        n_stale = sum(stale)
        n_late = sum(int(r["late"]) for r in rr) if has_late else None
        run = best = 0
        for s in stale:
            run = run + 1 if s else 0
            best = max(best, run)
        first = [f"{float(r['t']):.2f}" for r, s in zip(rr, stale) if s][:8]
        line = f"   ID {mid}: no reply in time {n_stale}/{len(rr)} ({100.0 * n_stale / len(rr):.3f}%)"
        if n_late is not None:
            line += f" = late {n_late} + never seen {max(0, n_stale - n_late)}"
        line += f", longest run {best}"
        print(line)
        if first:
            print(f"         first stale at t = {', '.join(first)} s")
        if n_stale:
            ok = False

    if has_rtt:
        per_cycle = {}
        for r in rows:
            per_cycle.setdefault(r["t"], float(r["rtt_us"]))
        rtt = sorted(per_cycle.values())
        over = sum(1 for v in rtt if v > period_us)
        print(f"   round trip: p50 {percentile(rtt, 0.5):.0f} / p90 {percentile(rtt, 0.9):.0f} / p99 "
              f"{percentile(rtt, 0.99):.0f} / p99.9 {percentile(rtt, 0.999):.0f} / max {rtt[-1]:.0f} us; "
              f"longer than the period: {over} ({100.0 * over / len(rtt):.3f}%)")
        headroom = period_us - percentile(rtt, 0.999)
        print(f"   headroom at p99.9: {headroom:+.0f} us "
              f"({'fits' if headroom > 0 else 'does NOT fit'} in one period)")
        if hist:
            bins = {}
            for v in rtt:
                b = int(v // 100)
                bins[b] = bins.get(b, 0) + 1
            peak = max(bins.values())
            for b in sorted(bins):
                bar = "#" * max(1, round(50 * bins[b] / peak))
                print(f"     {b * 100:5d}-{b * 100 + 100:5d} us {bins[b]:7d} {bar}")
    else:
        print("   (old log without rtt_us / late columns: rerun with the current motor_tool)")

    print(f"   => {'OK: no lost or late replies' if ok else 'NOT OK (a healthy bus must show 0)'}")
    return 0 if ok else 1


def check_trace(path):
    tx, rx = {}, {}
    rate = None
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                for item in line[1:].split():
                    if item.startswith("rate_hz="):
                        rate = float(item.split("=", 1)[1])
                continue
            parts = line.split()
            if len(parts) != 3:
                continue
            kind, t, mid = parts[0], int(parts[1]), int(parts[2])
            (tx if kind == "tx" else rx).setdefault(mid, []).append(t)
    print(f"== {path}" + (f"  (loop {rate:.0f} Hz, period {1e6 / rate:.0f} us)" if rate else ""))
    ok = True
    for mid in sorted(set(tx) | set(rx)):
        t_tx, t_rx = tx.get(mid, []), rx.get(mid, [])
        lost = len(t_tx) - len(t_rx)
        print(f"   ID {mid}: {len(t_tx)} requests, {len(t_rx)} replies -> "
              + ("none lost" if lost == 0 else f"{lost} NEVER arrived" if lost > 0 else f"{-lost} extra replies (?)"))
        # Where were replies lost?  After reply k, open(k) = requests sent so far - (k + 1) replies.  Normally it
        # goes 0 / 1 / ...; every lost reply raises its FLOOR for the rest of the run by one.  floor(k) = min over
        # the remaining replies; a step up of the floor between reply k-1 and k = a loss just before reply k.
        import bisect
        n_open = [bisect.bisect_right(t_tx, r) - (k + 1) for k, r in enumerate(t_rx)]
        floor, cur = [0] * len(n_open), 10 ** 9
        for k in range(len(n_open) - 1, -1, -1):
            cur = min(cur, n_open[k])
            floor[k] = cur
        skips, prev = {}, 0
        for k, fl in enumerate(floor):
            if fl > prev:
                skips[k] = fl - prev
            prev = max(prev, fl)
        # Pair in order (the k-th reply answers the k-th request), skipping the oldest open request at each loss.
        # Exact when nothing was lost; otherwise which request lost its reply is a best guess.
        pairs, unmatched, j = [], [], 0
        for k, r in enumerate(t_rx):
            for _ in range(skips.get(k, 0)):
                if j < len(t_tx):
                    unmatched.append(j)
                    j += 1
            if j < len(t_tx):
                pairs.append((j, r - t_tx[j]))
                j += 1
        unmatched += list(range(j, len(t_tx)))[: max(0, lost - len(unmatched))]
        lat = sorted(l for _, l in pairs)
        if lat:
            over = sum(1 for k, l in pairs if k + 1 < len(t_tx) and t_tx[k + 1] < t_tx[k] + l)
            print(f"         latency p50 {percentile(lat, 0.5):.0f} / p99 {percentile(lat, 0.99):.0f} / max {lat[-1]} us; "
                  f"{over} replies arrived after the NEXT request was already sent"
                  f"{' (pairing exact)' if lost == 0 else ' (pairing approximate: counts differ)'}")
        if lost > 0:
            ok = False
            for k in unmatched[:10]:
                nxt = t_tx[k + 1] - t_tx[k] if k + 1 < len(t_tx) else None
                print(f"         no reply to request #{k} at {t_tx[k] / 1e6:.4f} s"
                      + (f", next request {nxt} us later" if nxt is not None else ", last request"))
        if lat and any(k + 1 < len(t_tx) and t_tx[k + 1] < t_tx[k] + l for k, l in pairs):
            ok = False
    print(f"   => {'OK' if ok else 'NOT OK'}")
    return 0 if ok else 1


def main(argv):
    if "--trace" in argv:
        files = [a for a in argv if not a.startswith("--")]
        return max([check_trace(f) for f in files] or [2])
    files = [a for a in argv if not a.startswith("--")]
    if not files:
        print(__doc__)
        return 2
    rc = 0
    for f in files:
        try:
            rc = max(rc, check(f, "--hist" in argv))
        except (OSError, ValueError, KeyError) as e:
            print(f"{f}: {e}")
            rc = 2
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
