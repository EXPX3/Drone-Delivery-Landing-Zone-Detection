#!/usr/bin/env bash
set -euo pipefail

# Run on the Jetson host at boot, not inside the container.
sysctl -w net.core.rmem_max=2147483647
sysctl -w net.core.rmem_default=2147483647
sysctl -w net.ipv4.ipfrag_time=3
sysctl -w net.ipv4.ipfrag_high_thresh=134217728
