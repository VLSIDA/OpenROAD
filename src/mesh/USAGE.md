# Clock Mesh Module - Usage Guide

## Overview
This module provides clock sink finding functionality for building clock mesh structures in OpenROAD. It identifies all clock sinks (flip-flops and macros) in your design based on Liberty library information and SDC clock definitions.

## Installation

The mesh module is integrated into OpenROAD. After making the changes, rebuild OpenROAD:

```bash
cd /home/wajid/OpenROAD
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## Available Tcl Commands

### 1. `mesh [-name clock_name]`
Main command that finds clock sinks and prints a summary.

```tcl
# Run mesh for all clocks
mesh

# Run mesh for a specific clock
mesh -name clk
```

### 2. `find_clock_sinks`
Find all clock sinks in the design without printing.

```tcl
find_clock_sinks
```

### 3. `print_clock_sinks`
Print a detailed summary of all found clock sinks.

```tcl
print_clock_sinks
```

### 4. `get_num_clocks`
Get the number of clocks found in the design.

```tcl
set num_clocks [get_num_clocks]
puts "Found $num_clocks clocks"
```

### 5. `get_num_sinks clock_name`
Get the number of sinks for a specific clock.

```tcl
set num_sinks [get_num_sinks "clk"]
puts "Clock 'clk' has $num_sinks sinks"
```

## Example Usage

Here's a complete example of using the mesh module:

```tcl
# Load your design
read_lef tech.lef
read_lef cells.lef
read_def design.def

# Load Liberty files
read_liberty cells.lib

# Define clocks
create_clock -period 10 [get_ports clk]

# Find clock sinks
find_clock_sinks

# Print summary
print_clock_sinks

# Get statistics
set num_clocks [get_num_clocks]
puts "Total clocks: $num_clocks"

# For each clock, get sink count
set num_sinks [get_num_sinks "clk"]
puts "Clock 'clk' has $num_sinks sinks"

# Or use the all-in-one command
mesh -name clk
```

## Output Example

```
[MESH-3] Starting clock sink discovery...
[MESH-5] Processing clock: clk
[MESH-6]   Processing net: clk
[MESH-8]     Found sink: reg1/CLK at (1000, 2000) cap=1.234e-15 macro=0
[MESH-8]     Found sink: reg2/CLK at (1500, 2000) cap=1.234e-15 macro=0
[MESH-8]     Found sink: macro1/CLK at (3000, 4000) cap=5.678e-14 macro=1
[MESH-7] Clock sink discovery complete. Found 1 clock(s)

========== Clock Sink Summary ==========

Clock: clk
  Total sinks: 3
  Registers: 2
  Macros: 1
  Total capacitance: 5.802e-14
  Bounding box: (1000, 2000) to (3000, 4000)
  Width: 2000 Height: 2000
========================================
```

## How It Works

### Clock Sink Identification

The module identifies clock sinks using the following criteria:

1. **Liberty Library Check**: The pin must be marked as a register clock pin (`isRegClk()` = true)
2. **Placement Check**: The instance must be placed
3. **Signal Type**: Must be an input signal
4. **Macro Detection**: Hard macros/blocks are always considered sinks

### Key Functions

From `ClockMesh.hh`:

```cpp
struct ClockSink {
  std::string name;           // Instance/pin name (e.g., "reg1/CLK")
  int x;                      // X coordinate (DBU)
  int y;                      // Y coordinate (DBU)
  float inputCap;             // Input capacitance
  double insertionDelay;      // Insertion delay for macros
  odb::dbITerm* iterm;        // Pointer to database terminal
  bool isMacro;               // Is this a macro/block?
};
```

### Algorithm Flow

1. **Find Clock Roots**: Uses STA to identify clock nets from SDC constraints
2. **Traverse Networks**: Recursively follows clock nets, including gated clocks
3. **Identify Sinks**: For each terminal on clock nets:
   - Check if it's an input signal on a placed instance
   - Query Liberty library to verify it's a register clock pin
   - Compute position from pin shapes
   - Extract capacitance from Liberty
   - Determine if it's a macro with insertion delay
4. **Store Results**: Organize sinks by clock name for easy access

### Gated Clock Handling

The module automatically handles gated clocks by:
- Detecting non-sink terminals (like clock gates)
- Following their outputs recursively
- Building a complete clock tree including all gated branches

## Integration Points

The mesh module integrates with:

- **ODB**: Database access for design netlist and geometry
- **OpenSTA**: Static timing analysis for clock definitions and Liberty data
- **Liberty**: Cell library information for pin characteristics
- **SDC**: Clock constraints and definitions

## Next Steps

This module provides the foundation for building clock mesh structures. You can extend it to:

1. **Generate mesh grid coordinates**
2. **Create mesh routing topology**
3. **Insert mesh drivers and buffers**
4. **Optimize mesh sizing**
5. **Perform mesh-aware CTS**

## Files Modified

- `include/mesh/ClockMesh.hh` - Header with declarations
- `src/ClockMesh.cc` - Implementation with sink finding logic
- `src/MakeClockMesh.cc` - Module initialization
- `tcl/Mesh.i` - SWIG interface for Tcl
- `tcl/Mesh.tcl` - Tcl command definitions

## Troubleshooting

### "No block found in database"
Make sure you've loaded a design with `read_def` before running mesh commands.

### "STA not initialized"
Ensure you've loaded Liberty files with `read_liberty` before finding sinks.

### "No SDC constraints found"
Define clocks using `create_clock` before running sink finding.

### No sinks found
- Verify your Liberty files have proper `clock` attributes on sequential cell pins
- Check that instances are placed (not just in the netlist)
- Ensure clock nets are properly connected

## Reference

Based on TritonCTS implementation:
- `src/cts/src/TritonCTS.cpp:1912` - `isSink()` function
- `src/cts/src/TritonCTS.cpp:988` - `populateTritonCTS()` function
- `src/cts/src/TritonCTS.cpp:1881` - `findClockRoots()` function
