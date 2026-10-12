// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, The OpenROAD Authors

#pragma once

#include <string>
#include <vector>

#include "cms/ClockMesh.hh"

namespace odb {
class dbBlock;
class dbTechLayer;
}  // namespace odb

namespace sta {
class dbNetwork;
}

namespace utl {
class Logger;
}

namespace cms {

// Load-adaptive buffer placement at mesh grid intersections.
//
// A buffer is placed at every intersection. Each intersection picks the
// smallest buffer master whose load limit covers the local load there.
//
// Local load per intersection = (half of each adjacent mesh-wire segment cap)
// + (Liberty input pin caps of the sinks nearest to this intersection).
//
// A master's load limit is buffer_gain times its own Liberty input pin
// capacitance. `buffer_masters` is the candidate library; order does not
// matter, the function sorts by ascending load limit. Sets `has_buffer` and
// `buffer_inst` on each intersection it places.
void placeBuffersLoadAdaptive(odb::dbBlock* block,
                              sta::dbNetwork* network,
                              utl::Logger* logger,
                              odb::dbTechLayer* h_layer,
                              odb::dbTechLayer* v_layer,
                              const std::vector<MeshWire>& mesh_wires,
                              const std::vector<ClockSink>& sinks,
                              std::vector<GridIntersection>& grid_intersections,
                              const std::vector<std::string>& buffer_masters,
                              float buffer_gain);

// Set-cover buffer placement at mesh grid intersections.
//
// A greedy loop picks (intersection, master) pairs that cover the most
// uncovered sinks per unit of load limit, until every sink is covered or no
// pair adds coverage. Intersections that add no coverage get no buffer.
//
// For each candidate (intersection, master), the mesh-wire graph is flooded
// from that intersection in order of lumped resistance, accumulating the
// per-node local load (wire halves plus nearest-sink pin caps) until the
// master's load limit is exhausted. Sinks whose host intersection lies inside
// the flooded region are covered by that pair.
void placeBuffersSetCover(odb::dbBlock* block,
                          sta::dbNetwork* network,
                          utl::Logger* logger,
                          odb::dbTechLayer* h_layer,
                          odb::dbTechLayer* v_layer,
                          const std::vector<MeshWire>& mesh_wires,
                          const std::vector<ClockSink>& sinks,
                          std::vector<GridIntersection>& grid_intersections,
                          const std::vector<std::string>& buffer_masters,
                          float buffer_gain);

}  // namespace cms
