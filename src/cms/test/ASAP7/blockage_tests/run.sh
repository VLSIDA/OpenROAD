#!/usr/bin/env bash
# Driver — runs every scenario in its own openroad process for clean state.
set -u
cd "$(dirname "$0")"

BIN=${OPENROAD_BIN:-/home/wajid/OpenROAD/build/bin/openroad}

pass=0; fail=0; summaries=()
for t in 0*.tcl; do
    echo ""
    echo "========================================"
    echo "RUN: $t"
    echo "========================================"
    out=$("$BIN" "$t" 2>&1 | tail -20)
    echo "$out"
    last=$(echo "$out" | grep -E "^(PASS|FAIL):" | tail -1)
    if [[ "$last" == PASS:* ]]; then
        pass=$((pass+1)); summaries+=("PASS $t")
    else
        fail=$((fail+1)); summaries+=("FAIL $t")
    fi
done

echo ""
echo "========================================"
echo "Results"
echo "========================================"
printf '%s\n' "${summaries[@]}"
echo "Total: $pass passed, $fail failed"
if [[ $fail -gt 0 ]]; then exit 1; fi
