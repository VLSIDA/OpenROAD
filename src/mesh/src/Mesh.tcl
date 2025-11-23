# Mesh.tcl
# Minimal Tcl interface for Mesh tool
# SPDX-License-Identifier: BSD-3-Clause

package provide mesh 1.0

# Register the SWIG-generated commands
namespace eval mesh {
    # SWIG will add commands here at runtime
}

puts "Mesh Tcl package loaded"

