// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"

namespace utl {
class Logger;
}

namespace sta {
class dbSta;
class dbNetwork;
class Clock;
}  // namespace sta

namespace ord {
class OpenRoad;
}

namespace mesh {

// Data structure to hold clock sink information
struct ClockSink {
  std::string name;          
  int x;                
  int y;                    
  float inputCap;             
  double insertionDelay;    
  odb::dbITerm* iterm;       
  bool isMacro;             

  ClockSink(const std::string& n, int px, int py, float cap,
            double insDelay, odb::dbITerm* term, bool macro)
    : name(n), x(px), y(py), inputCap(cap),
      insertionDelay(insDelay), iterm(term), isMacro(macro) {}
};

class ClockMesh
{
 public:
  ClockMesh();

  void init(ord::OpenRoad* openroad);
  void run(const char* name);
  bool meshGenerated() const { return mesh_generated_; }

 
  void findClockSinks();
  void testPrintSinks();

 private:

  void findClockRoots(sta::Clock* clk, std::set<odb::dbNet*>& clockNets);
  bool isSink(odb::dbITerm* iterm);
  float getInputPinCap(odb::dbITerm* iterm);
  bool hasInsertionDelay(odb::dbInst* inst, odb::dbMTerm* mterm);
  double computeInsertionDelay(const std::string& name,
                               odb::dbInst* inst,
                               odb::dbMTerm* mterm);
  void computeITermPosition(odb::dbITerm* term, int& x, int& y) const;

  // Helper for CTS-style sink finding
  bool separateSinks(odb::dbNet* net,
                     std::vector<ClockSink>& sinks);

  // Member variables
  ord::OpenRoad* openroad_ = nullptr;
  bool mesh_generated_ = false;

  // Database pointers
  odb::dbDatabase* db_ = nullptr;
  odb::dbBlock* block_ = nullptr;
  sta::dbSta* sta_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;

  // Storage for found sinks
  std::map<std::string, std::vector<ClockSink>> clockToSinks_;
  std::set<odb::dbNet*> staClockNets_;
  std::set<odb::dbNet*> visitedClockNets_;
};

void initClockMesh(ord::OpenRoad* openroad);

}  // namespace mesh
