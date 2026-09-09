// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2025, The OpenROAD Authors

#pragma once

#include "db_sta/dbNetwork.hh"
#include "odb/db.h"
#include "odb/dbBlockCallBackObj.h"
#include "sta/Network.hh"

namespace est {

class EstimateParasitics;

class OdbCallBack : public odb::dbBlockCallBackObj
{
 public:
  OdbCallBack(EstimateParasitics* estimate_parasitics,
              sta::Network* network,
              sta::dbNetwork* db_network);

  void inDbInstCreate(odb::dbInst* inst) override;
  void inDbNetCreate(odb::dbNet* net) override;
  void inDbNetDestroy(odb::dbNet* net) override;
  void inDbITermPostConnect(odb::dbITerm* iterm) override;
  void inDbITermPostDisconnect(odb::dbITerm* iterm, odb::dbNet* net) override;
  void inDbInstSwapMasterAfter(odb::dbInst* inst) override;
  void inDbPostMoveInst(odb::dbInst* inst) override;

 private:
  // Invalidate the estimated parasitics of every signal net connected to
  // inst.  Relocating (or reverting the relocation of) a gate changes the
  // geometry of each net it touches, so the placement wire RC must be
  // re-derived from the new pin locations before the next timing update.
  void invalidateInstConnectedNets(odb::dbInst* inst);

  EstimateParasitics* estimate_parasitics_;
  sta::Network* network_;
  sta::dbNetwork* db_network_;
};

}  // namespace est
