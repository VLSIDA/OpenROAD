#!/usr/bin/env python3
"""
Parse CTS log output to extract sink locations
CTS logs each sink as: "CTS_SINK_DATA: <name> <x> <y> <net>"
"""

import re
import sys
import csv

def parse_cts_log(log_file, output_csv):
    """Extract CTS_SINK_DATA lines from log and write to CSV"""

    sinks = []

    with open(log_file, 'r') as f:
        for line in f:
            # Look for lines like: "CTS_SINK_DATA: inst/pin 1234 5678 clk"
            match = re.search(r'CTS_SINK_DATA:\s+(\S+)\s+(\d+)\s+(\d+)\s+(\S+)', line)
            if match:
                name = match.group(1)
                x = match.group(2)
                y = match.group(3)
                net = match.group(4)
                sinks.append((x, y, name, net))

    # Write to CSV
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['x', 'y', 'name', 'net', 'source'])
        for x, y, name, net in sinks:
            writer.writerow([x, y, name, net, 'cts'])

    print(f"Parsed {len(sinks)} sinks from CTS log")
    print(f"Output: {output_csv}")

    return len(sinks)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <cts_log_file> <output_csv>")
        sys.exit(1)

    log_file = sys.argv[1]
    output_csv = sys.argv[2]

    count = parse_cts_log(log_file, output_csv)

    if count == 0:
        print("WARNING: No CTS_SINK_DATA lines found in log!")
        print("Make sure CTS was rebuilt with logging changes")
        sys.exit(1)
