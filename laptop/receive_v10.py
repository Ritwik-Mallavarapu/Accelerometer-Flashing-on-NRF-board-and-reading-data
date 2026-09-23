"""
TYRE_SENSOR receiver  (v10 firmware - signed decode)
=====================================================

Packet layout, little-endian:
    offset 0  uint16  sequence      (0 = handshake)
    offset 2  uint16  value count   (0, 1 or 2)
    offset 4  int16 x count         averaged X, Y, Z

Each value averages 10 raw sensor samples (50 ms), so the sensor samples
at 200 Hz while the reported rate is 20 Hz.

Counts -> g at +/-8 g:  256 counts per g.

OUTPUT FILES
------------
Every run creates a NEW, timestamped file. Nothing is overwritten:

    tyre_data_2026-09-23_18-42-07.csv
    tyre_data_2026-09-23_18-42-07_REJECTED.csv   (only if needed)

Override the base name with --out if you want something specific.

    pip install bleak
    python receive_v10.py --seconds 20
    python receive_v10.py --seconds 20 --out bench_test
"""

import argparse
import asyncio
import csv
import os
import struct
import sys
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "TYRE_SENSOR"
DATA_UUID = "6b1f0002-7a3c-4d2e-9f10-8a5c1e200002"
DIAG_UUID = "6b1f0003-7a3c-4d2e-9f10-8a5c1e200003"

HEADER_BYTES = 4
AXIS_BYTES = 6
COUNTS_PER_G = 256.0
REPORT_DT = 0.05
MAX_LEGAL_COUNTS = 2048          # 8 g at 256 counts/g

DATA_DIR = "data"                # all recordings go here


def make_filenames(base=None, folder=DATA_DIR):
    """Return (data_path, reject_path) with a unique timestamped name.

    If `base` is given, it is used as the stem (still timestamped).
    Files that already exist are never reused - a numeric suffix is
    added so nothing is ever overwritten.
    """
    if folder:
        os.makedirs(folder, exist_ok=True)

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    stem = f"{base}_{stamp}" if base else f"tyre_data_{stamp}"

    data_path = os.path.join(folder, stem + ".csv")
    reject_path = os.path.join(folder, stem + "_REJECTED.csv")

    # Safety net: never overwrite, even if two runs start in the same second
    n = 1
    while os.path.exists(data_path):
        data_path = os.path.join(folder, f"{stem}_{n}.csv")
        reject_path = os.path.join(folder, f"{stem}_{n}_REJECTED.csv")
        n += 1

    return data_path, reject_path


def parse(payload: bytes):
    if len(payload) < HEADER_BYTES:
        return None

    seq, count = struct.unpack_from("<HH", payload, 0)

    available = (len(payload) - HEADER_BYTES) // AXIS_BYTES
    if count > available:
        count = available

    out = []
    off = HEADER_BYTES
    for _ in range(count):
        x, y, z = struct.unpack_from("<hhh", payload, off)
        out.append((x, y, z))
        off += AXIS_BYTES

    return seq, out


async def run(seconds, base_name, folder):
    print(f"Scanning for '{DEVICE_NAME}' ...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print("Device not found. J-Link unplugged? Board powered?")
        return 1

    print(f"Found {device.address}")

    rows = []
    rejects = []
    stats = {"packets": 0, "values": 0, "lost": 0, "empty": 0,
             "lens": {}, "last_seq": None, "t0": None}

    def on_notify(_h, data: bytes):
        stats["lens"][len(data)] = stats["lens"].get(len(data), 0) + 1

        parsed = parse(data)
        if parsed is None:
            return
        seq, values = parsed

        if seq == 0:
            print(f"  handshake received ({len(data)} bytes)")
            return

        stats["packets"] += 1

        if stats["last_seq"] is not None:
            gap = (seq - stats["last_seq"]) & 0xFFFF
            if gap != 1:
                stats["lost"] += gap - 1
        stats["last_seq"] = seq

        if not values:
            stats["empty"] += 1
            return

        stats["values"] += len(values)

        if stats["t0"] is None:
            stats["t0"] = time.time()
        base = stats["t0"]
        pkt_t = time.time() - base
        span = REPORT_DT * len(values)

        for i, (x, y, z) in enumerate(values):
            t = pkt_t - span + (i * REPORT_DT)
            row = (f"{t:.6f}", seq, i, x, y, z,
                   f"{x / COUNTS_PER_G:.4f}",
                   f"{y / COUNTS_PER_G:.4f}",
                   f"{z / COUNTS_PER_G:.4f}")

            if (abs(x) > MAX_LEGAL_COUNTS or abs(y) > MAX_LEGAL_COUNTS
                    or abs(z) > MAX_LEGAL_COUNTS):
                rejects.append(row)
            else:
                rows.append(row)

    async with BleakClient(device) as client:
        print("Connected.")

        try:
            raw = await client.read_gatt_char(DIAG_UUID)
            report = raw.decode("ascii", "replace").strip("\x00").strip()
            print("Device report:", report)
            f = {}
            for part in report.split():
                if "=" in part:
                    k, v = part.split("=", 1)
                    f[k] = v
            if f.get("who") == "2A":
                print("  sensor  : detected (WHO_AM_I = 0x2A)")
            if f.get("nfail") not in (None, "0"):
                print(f"  warning : {f['nfail']} notify failures")
        except Exception as e:
            print(f"(diagnostic read failed: {e})")

        await client.start_notify(DATA_UUID, on_notify)
        print(f"Recording {seconds} s ...")

        deadline = time.time() + seconds
        try:
            while time.time() < deadline:
                await asyncio.sleep(0.5)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass

        try:
            await client.stop_notify(DATA_UUID)
        except Exception:
            pass

    print("Disconnected.")

    if not rows:
        print("No valid data received.")
        print("Packet lengths seen:", stats["lens"] or "none")
        print(f"Rejected values: {len(rejects)}")
        return 1

    data_path, reject_path = make_filenames(base_name, folder)

    with open(data_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "seq", "n", "x_raw", "y_raw", "z_raw",
                    "x_g", "y_g", "z_g"])
        w.writerows(rows)

    wrote_reject = False
    if rejects:
        with open(reject_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["time_s", "seq", "n", "x_raw", "y_raw", "z_raw",
                        "x_g", "y_g", "z_g"])
            w.writerows(rejects)
        wrote_reject = True

    dur = float(rows[-1][0]) - float(rows[0][0]) if len(rows) > 1 else 0.0
    rate = stats["values"] / dur if dur > 0 else 0.0

    print()
    print(f"Saved {len(rows)} values -> {data_path}")
    if wrote_reject:
        print(f"Saved {len(rejects)} flagged values -> {reject_path}")
    print(f"Packets          : {stats['packets']}")
    print(f"Packet lengths   : {stats['lens']}")
    print(f"Packet gaps      : {stats['lost']}")
    print(f"Duration         : {dur:.2f} s")
    print(f"Reported rate    : {rate:.1f} Hz  (target 20)")

    xs = [float(r[6]) for r in rows]
    ys = [float(r[7]) for r in rows]
    zs = [float(r[8]) for r in rows]

    def rng(a):
        return f"{min(a):+.2f} .. {max(a):+.2f} g"

    print()
    print("Range over the recording:")
    print(f"  X   {rng(xs)}")
    print(f"  Y   {rng(ys)}")
    print(f"  Z   {rng(zs)}")

    mag = [(x * x + y * y + z * z) ** 0.5 for x, y, z in zip(xs, ys, zs)]
    print(f"  |A| {min(mag):.2f} .. {max(mag):.2f} g  (mean {sum(mag)/len(mag):.2f} g)")

    print()
    if wrote_reject:
        print(f"!! {len(rejects)} values were outside the +/-8 g range.")
        print("   They are in the reject file, NOT in the main dataset.")
        print("   First few:", rejects[:3])
    else:
        print("All values within the physical +/-8 g range. Decode verified.")

    print()
    print(f"File: {data_path}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out", default=None,
                    help="base name for the file (still timestamped). "
                         "Default: tyre_data")
    ap.add_argument("--dir", default=DATA_DIR,
                    help=f"folder for recordings (default: {DATA_DIR})")
    a = ap.parse_args()
    try:
        return asyncio.run(run(a.seconds, a.out, a.dir))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
