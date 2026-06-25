// SPDX-License-Identifier: BSD-3-Clause

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
// Mirrors clocksyn's grid_t::buffer_grid() in `buffer_entire_grid = true`
// mode (clocksyn/src/grid.C:186): a buffer is dropped at every intersection,
// but each intersection picks the *smallest* buffer master whose Liberty
// max_capacitance can drive that intersection's local load.
//
// Local load per intersection = (half of each adjacent mesh-wire segment cap)
// + (input pin caps of sinks Voronoi-assigned to this intersection).
//
// `buffer_masters` is the candidate library. Order doesn't matter — the
// function sorts internally by ascending max-load. Sets `has_buffer`
// and `buffer_inst` on each intersection it places.
void placeBuffersLoadAdaptive(
    odb::dbBlock* block,
    sta::dbNetwork* network,
    utl::Logger* logger,
    odb::dbTechLayer* h_layer,
    odb::dbTechLayer* v_layer,
    const std::vector<MeshWire>& mesh_wires,
    const std::vector<ClockSink>& sinks,
    std::vector<GridIntersection>& grid_intersections,
    const std::vector<std::string>& buffer_masters);

// Set-cover buffer placement at mesh grid intersections.
//
// Mirrors clocksyn's grid_t::buffer_grid() in `buffer_entire_grid = false`
// mode (clocksyn/src/grid.C:235): a greedy outer loop picks (intersection,
// master) pairs that cover the most uncovered sinks per unit of buffer
// capacity, until every sink is covered (or no progress can be made).
// Intersections that don't add new coverage get no buffer.
//
// For each candidate (intersection, master), the algorithm BFS-floods from
// that intersection through the mesh-wire graph, accumulating per-node local
// caps (wire halves + Voronoi-assigned sink caps) until the master's
// max-capacitance budget is exhausted. Sinks whose host intersection lies
// inside the flooded region are "covered" by that (intersection, master).
//
// The outer greedy loop picks the pair with the smallest
// `max_load_pF / new_covered_sinks` ratio (mirrors clocksyn's
// `bufsizes[j] / unique_cov.size()`), places the buffer, marks those sinks
// covered, and repeats.
void placeBuffersSetCover(
    odb::dbBlock* block,
    sta::dbNetwork* network,
    utl::Logger* logger,
    odb::dbTechLayer* h_layer,
    odb::dbTechLayer* v_layer,
    const std::vector<MeshWire>& mesh_wires,
    const std::vector<ClockSink>& sinks,
    std::vector<GridIntersection>& grid_intersections,
    const std::vector<std::string>& buffer_masters);

}  // namespace cms
