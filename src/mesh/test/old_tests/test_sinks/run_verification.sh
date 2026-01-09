#!/bin/bash
# Master script to run complete sink verification workflow
# Compares MESH vs actual CTS execution (using CTS log output)

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="/home/wajid/OpenROAD/build"
OPENROAD="${BUILD_DIR}/bin/openroad"

echo "========================================"
echo "MESH vs CTS Sink Verification"
echo "========================================"
echo ""

# Check if OpenROAD is built
if [ ! -f "$OPENROAD" ]; then
    echo "ERROR: OpenROAD binary not found at: $OPENROAD"
    echo "Please build OpenROAD first"
    exit 1
fi

echo "Using OpenROAD: $OPENROAD"
echo ""

# Step 1: Run MESH with logging and capture output
echo "Step 1/5: Running MESH with sink logging..."
cd "$BUILD_DIR"
"$OPENROAD" "$SCRIPT_DIR/extract_mesh_sinks.tcl" 2>&1 | tee "$SCRIPT_DIR/mesh_output.log"
echo ""

# Step 2: Parse MESH log to extract sink data
echo "Step 2/5: Parsing MESH log to extract sink locations..."
cd "$SCRIPT_DIR"
python3 parse_mesh_log.py mesh_output.log mesh_sinks.csv
echo ""

# Step 3: Run CTS with logging and capture output
echo "Step 3/5: Running CTS with sink logging..."
cd "$BUILD_DIR"
"$OPENROAD" "$SCRIPT_DIR/run_cts_with_logging.tcl" 2>&1 | tee "$SCRIPT_DIR/cts_output.log"
echo ""

# Step 4: Parse CTS log to extract sink data
echo "Step 4/5: Parsing CTS log to extract sink locations..."
cd "$SCRIPT_DIR"
python3 parse_cts_log.py cts_output.log cts_sinks.csv
echo ""

# Step 5: Compare and plot
echo "Step 5/5: Comparing sinks and generating plots..."
cd "$SCRIPT_DIR"
python3 compare_sinks.py
echo ""

echo "========================================"
echo "Verification Complete!"
echo "========================================"
echo ""
echo "Output files in: $SCRIPT_DIR"
echo "  - mesh_sinks.csv          : MESH sink locations"
echo "  - cts_sinks.csv           : CTS sink locations (from actual CTS execution)"
echo "  - cts_output.log          : Full CTS log output"
echo "  - comparison_results.csv  : Detailed comparison"
echo "  - sink_comparison.png     : Visualization plots"
echo "  - sink_differences.png    : Position difference arrows"
echo ""
echo "This verifies MESH against ACTUAL CTS code execution!"
echo ""
