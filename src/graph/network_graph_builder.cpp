#include "graph/network_graph.hpp"

#include <vector>
#include <utility>

namespace geodis {

// ─── Forward declarations of build_spatial_index (used by load_binary) ──────

// NetworkGraph uses a private helper to rebuild the spatial index after load.
// This is declared in the header as a private method and defined here.

// (The actual definition is in network_graph.cpp — this file exists for
//  future expansion: graph simplification, edge contraction, etc.)

} // namespace geodis
