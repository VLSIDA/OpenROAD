# Clock Mesh Synthesis

The clock mesh synthesis module in OpenROAD (`cms`) builds a clock mesh: a
grid of horizontal and vertical stripes on two routing layers, shorted at
every crossing, driven by a buffer at each grid intersection. The buffers are
fed from the clock root by a TritonCTS tree, and the clock sinks attach to the
nearest mesh stripe. Because every sink sees many drivers in parallel, a mesh
has lower skew and lower sensitivity to variation than a tree, at the cost of
higher clock power.

The flow is split into commands so that OpenROAD's own placement and routing
tools can be used between the steps:

1. `create_clock_mesh` draws the mesh, places the mesh buffers and runs
   TritonCTS from the clock root to the buffer inputs.
2. `detailed_placement` legalizes the mesh buffers.
3. `setup_proxy_bterms` and `connect_sinks_to_mesh` create a block terminal on
   the mesh for every buffer output and every sink, each on its own net.
4. `global_route` and `detailed_route` connect the buffer outputs and sinks to
   those terminals.
5. `connect_proxy_bterms_to_mesh` drops via stacks from the terminals to the
   mesh stripes.
6. `capture_mesh_arrivals`, `merge_mesh_nets`, `convert_mesh_swire` and
   `write_mesh_spice` prepare the result for parasitic extraction and SPICE
   simulation. A shorted mesh is a cyclic timing graph that static timing
   analysis cannot evaluate, so skew and power are measured in SPICE.

## Commands

```{note}
- Parameters in square brackets `[-param param]` are optional.
- Parameters without square brackets `-param2 param2` are required.
```

### Create Clock Mesh

Create the mesh stripes for a clock, place a buffer at every grid
intersection and build a TritonCTS tree from the clock root to the buffer
inputs. The stripe pitch is rounded to the least common multiple of the
routing track pitches of the two layers so that stripes stay on track. Stripe
width is the default width of each layer.

```tcl
create_clock_mesh
    -clock clock_name
    -h_layer layer
    -v_layer layer
    -pitch pitch
    [-buffers buffers]
    [-cts_buffers buffers]
    [-macro_halo halo]
    [-mesh_strategy strategy]
    [-buffer_gain gain]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock, as defined by `create_clock`. |
| `-h_layer` | Routing layer for the horizontal stripes. |
| `-v_layer` | Routing layer for the vertical stripes. |
| `-pitch` | Stripe pitch in microns. |
| `-buffers` | List of buffer masters placed at the grid intersections. With a single master every intersection gets that buffer. With several masters the buffer size at each intersection is chosen from the sink load nearby. |
| `-cts_buffers` | List of buffer masters TritonCTS may use for the tree from the clock root to the mesh buffers. If omitted, TritonCTS infers the buffers from the loaded Liberty libraries. |
| `-macro_halo` | Halo in microns around macros and placement blockages. No stripes, vias or buffers are created inside a macro plus its halo. Default is 0. |
| `-mesh_strategy` | Buffer placement strategy: `uniform` places the first buffer master at every intersection, `adaptive` sizes each intersection buffer from the sink load it drives, `set_cover` keeps only the intersections needed to cover every sink. The default is `uniform` for a single buffer master and `adaptive` for several. |
| `-buffer_gain` | Load a buffer may drive, as a multiple of its own Liberty input capacitance. Used by the `adaptive` and `set_cover` strategies to size buffers. The default is 3. |

### Setup Proxy BTerms

Create a block terminal on the proxy layer above every mesh buffer output,
each on its own net. The router connects the buffer output to the terminal,
and `connect_proxy_bterms_to_mesh` later ties the terminal to the mesh. Call
this before `connect_sinks_to_mesh`.

```tcl
setup_proxy_bterms
    -clock clock_name
    -proxy_layer layer
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |
| `-proxy_layer` | Routing layer for the terminals, normally the layer above the mesh. |

### Connect Sinks To Mesh

Move every clock sink to its nearest grid intersection. A sink whose
intersection already has a buffer terminal joins that terminal's net.
Otherwise a new terminal and net are created at the intersection. The router
then connects each sink to its terminal.

```tcl
connect_sinks_to_mesh
    -clock clock_name
    -proxy_layer layer
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |
| `-proxy_layer` | Routing layer for the sink terminals. |

### Connect Proxy BTerms To Mesh

After routing, create via stacks from each terminal placed by
`setup_proxy_bterms` and `connect_sinks_to_mesh` down to the mesh stripe
beneath it, electrically shorting the buffer and sink nets to the mesh.

```tcl
connect_proxy_bterms_to_mesh
    -clock clock_name
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |

### Capture Mesh Arrivals

Record the arrival time and slew at the input of every mesh buffer from
static timing analysis of the TritonCTS tree. `write_mesh_spice` uses these
values to drive each mesh buffer with a clock source that has the real tree
delay. Call this before `merge_mesh_nets`, while the timing graph is still
acyclic.

```tcl
capture_mesh_arrivals
    -clock clock_name
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |

### Merge Mesh Nets

Move the routing of the buffer output nets and sink nets onto the mesh net
and remove their terminals, so that the mesh is a single net for parasitic
extraction.

```tcl
merge_mesh_nets
    -clock clock_name
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |

### Convert Mesh SWire

Convert the mesh stripes from special wires to regular wires. OpenRCX only
extracts regular wires, so this is required before `extract_parasitics`. The
stripes are broken at every point where a routed wire or via stack meets them
so that the extracted network has a node there.

```tcl
convert_mesh_swire
    -clock clock_name
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |

### Write Mesh SPICE

Write a SPICE netlist of the clock mesh from the extracted parasitics: the
mesh RC network, the mesh buffers, the sink pin capacitances and a clock
source per mesh buffer. The supply voltage and clock period are taken from
the Liberty library and the SDC constraints unless overridden.

```tcl
write_mesh_spice
    -clock clock_name
    -output file
    [-vdd voltage]
    [-rise_time time]
    [-fall_time time]
    [-spice_models files]
    [-zero_delay]
    [-full_tree]
```

#### Options

| Switch Name | Description |
| ----- | ----- |
| `-clock` | Name of the clock. |
| `-output` | Output SPICE file. |
| `-vdd` | Supply voltage in volts. Default is the nominal voltage of the Liberty library. |
| `-rise_time` | Rise time in nanoseconds of the clock sources. Default is the slew captured by `capture_mesh_arrivals`. |
| `-fall_time` | Fall time in nanoseconds of the clock sources. Default is the slew captured by `capture_mesh_arrivals`. |
| `-spice_models` | List of SPICE model or CDL files to include for the buffer and sink subcircuits. |
| `-zero_delay` | Drive every mesh buffer from an ideal source at time zero instead of the captured tree arrival. |
| `-full_tree` | Also write the TritonCTS tree from the clock root to the mesh buffers, so the clock source is at the root. |

## Example scripts

Build a mesh on metal4 and metal5 with a buffer at every intersection and
connect the sinks through the router:

```
create_clock_mesh -clock clk -h_layer metal5 -v_layer metal4 -pitch 2.0 \
    -buffers CLKBUF_X3 -cts_buffers CLKBUF_X3
detailed_placement
setup_proxy_bterms -clock clk -proxy_layer metal6
connect_sinks_to_mesh -clock clk -proxy_layer metal6
global_route
detailed_route
connect_proxy_bterms_to_mesh -clock clk
capture_mesh_arrivals -clock clk
merge_mesh_nets -clock clk
convert_mesh_swire -clock clk
extract_parasitics -ext_model_file rcx.rules
write_mesh_spice -clock clk -output mesh.sp
```

## Regression tests

There are a set of regression tests in `./test`. For more information, refer to this [section](../../README.md#regression-tests).

Simply run the following script:

```shell
./test/regression
```

## Limitations

- A clock mesh is a cyclic timing graph, so `report_checks` and `report_power`
  cannot evaluate it. Skew and power are measured in SPICE.
- One mesh per clock. Hierarchical and multi-level meshes are not supported.
- The proxy and sink block terminals are each on their own net during
  routing, separate from the mesh net, even though they are electrically
  shorted to the mesh stripes once `connect_proxy_bterms_to_mesh` drops the
  via stacks. The router treats these nets as distinct, so it reports
  short and spacing violations where a terminal net meets the mesh net.
  These violations are expected and do not affect the SPICE result, but in a
  full flow they appear in the detailed routing DRC report.

## License

BSD 3-Clause License. See [LICENSE](../../LICENSE) file.
