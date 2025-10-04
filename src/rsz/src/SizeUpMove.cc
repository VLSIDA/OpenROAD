// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include "SizeUpMove.hh"

#include <cmath>
#include <string>

#include "BaseMove.hh"
#include "CloneMove.hh"
#include "sta/GraphDelayCalc.hh"

namespace rsz {

using std::string;

using utl::RSZ;

using sta::ArcDelay;
using sta::DcalcAnalysisPt;
using sta::Instance;
using sta::InstancePinIterator;
using sta::LibertyCell;
using sta::LibertyPort;
using sta::LoadPinIndexMap;
using sta::NetConnectedPinIterator;
using sta::Pin;
using sta::Slack;
using sta::Slew;

bool SizeUpMove::doMove(const Pin* drvr_pin, float setup_slack_margin)
{
  Instance* drvr = network_->instance(drvr_pin);
  const DcalcAnalysisPt* dcalc_ap = resizer_->tgt_slew_dcalc_ap_;
  const float load_cap = graph_delay_calc_->loadCap(drvr_pin, dcalc_ap);
  Pin* prev_drvr_pin;
  Pin* drvr_input_pin;
  Pin* load_pin;
  getPrevNextPins(drvr_pin, prev_drvr_pin, drvr_input_pin, load_pin);
  // Unconstrained driver
  if (drvr_input_pin == nullptr) {
    return false;
  }
  LibertyPort* in_port = network_->libertyPort(drvr_input_pin);

  // We always size the cloned gates for some reason, but it would be good if we
  // also down-sized here instead since we might want smaller original.
  if (resizer_->dontTouch(drvr)) {
    return false;
  }
  if (!resizer_->isLogicStdCell(drvr)) {
    return false;
  }

  float prev_drive = 0.0;
  if (prev_drvr_pin) {
    LibertyPort* prev_drvr_port = network_->libertyPort(prev_drvr_pin);
    if (prev_drvr_port) {
      prev_drive = prev_drvr_port->driveResistance();
    }
  }

  LibertyPort* drvr_port = network_->libertyPort(drvr_pin);
  LibertyCell* upsize
      = upsizeCell(in_port, drvr_port, load_cap, prev_drive, dcalc_ap);

  if (upsize && replaceCell(drvr, upsize)) {
    debugPrint(logger_,
               RSZ,
               "opt_moves",
               1,
               "ACCEPT size_up {} {} -> {}",
               network_->pathName(drvr_pin),
               drvr_port->libertyCell()->name(),
               upsize->name());
    debugPrint(logger_,
               RSZ,
               "repair_setup",
               3,
               "size_up {} {} -> {}",
               network_->pathName(drvr_pin),
               drvr_port->libertyCell()->name(),
               upsize->name());
    addMove(drvr);
    return true;
  }
  debugPrint(logger_,
             RSZ,
             "opt_moves",
             3,
             "REJECT size_up {} {}",
             network_->pathName(drvr_pin),
             drvr_port->libertyCell()->name());
  return false;
}
LibertyCell* SizeUpMove::upsizeCell(LibertyPort* in_port,
                                    LibertyPort* drvr_port,
                                    const float load_cap,
                                    const float prev_drive,
                                    const DcalcAnalysisPt* dcalc_ap)
{
  const int lib_ap = dcalc_ap->libertyIndex();
  LibertyCell* cell = drvr_port->libertyCell();
  LibertyCellSeq swappable_cells = resizer_->getSwappableCells(cell);
  if (!swappable_cells.empty()) {
    const char* in_port_name = in_port->name();
    const char* drvr_port_name = drvr_port->name();
    sort(swappable_cells,
         [=](const LibertyCell* cell1, const LibertyCell* cell2) {
           LibertyPort* port1
               = cell1->findLibertyPort(drvr_port_name)->cornerPort(lib_ap);
           LibertyPort* port2
               = cell2->findLibertyPort(drvr_port_name)->cornerPort(lib_ap);
           const float drive1 = port1->driveResistance();
           const float drive2 = port2->driveResistance();
           const ArcDelay intrinsic1 = port1->intrinsicDelay(this);
           const ArcDelay intrinsic2 = port2->intrinsicDelay(this);
           const float capacitance1 = port1->capacitance();
           const float capacitance2 = port2->capacitance();
           return std::tie(drive2, intrinsic1, capacitance1)
                  < std::tie(drive1, intrinsic2, capacitance2);
         });
    const float drive = drvr_port->cornerPort(lib_ap)->driveResistance();
    const float delay
        = resizer_->gateDelay(drvr_port, load_cap, resizer_->tgt_slew_dcalc_ap_)
          + (prev_drive * in_port->cornerPort(lib_ap)->capacitance());

    for (LibertyCell* swappable : swappable_cells) {
      // Ignore if swappable is actually the same cell
      if (swappable == cell) {
        continue;
      }
      LibertyCell* swappable_corner = swappable->cornerCell(lib_ap);
      LibertyPort* swappable_drvr
          = swappable_corner->findLibertyPort(drvr_port_name);
      LibertyPort* swappable_input
          = swappable_corner->findLibertyPort(in_port_name);
      const float swappable_drive = swappable_drvr->driveResistance();
      // Include delay of previous driver into swappable gate.
      const float swappable_delay
          = resizer_->gateDelay(swappable_drvr, load_cap, dcalc_ap)
            + (prev_drive * swappable_input->capacitance());
      if (swappable_drive < drive && swappable_delay < delay) {
        return swappable;
      }
    }
  }
  return nullptr;
};

// Compare drive strength with previous stage
// and match if needed
// For example, if BUFX16 drives BUFX1, replace BUFX1 with BUFX16
bool SizeUpMatchMove::doMove(const Pin* drvr_pin, float setup_slack_margin)
{
  if (drvr_pin == nullptr) {
    return false;
  }

  Instance* drvr = network_->instance(drvr_pin);
  if (drvr == nullptr) {
    return false;
  }

  Pin* prev_drvr_pin;
  Pin* drvr_input_pin;
  Pin* load_pin;
  getPrevNextPins(drvr_pin, prev_drvr_pin, drvr_input_pin, load_pin);
  if (prev_drvr_pin == nullptr) {
    return false;
  }

  if (resizer_->dontTouch(drvr) || !resizer_->isLogicStdCell(drvr)) {
    return false;
  }

  LibertyPort* drvr_port = network_->libertyPort(drvr_pin);
  if (drvr_port == nullptr) {
    return false;
  }
  const LibertyCell* drvr_cell = drvr_port->libertyCell();
  if (drvr_cell == nullptr) {
    return false;
  }

  LibertyPort* prev_drvr_port = network_->libertyPort(prev_drvr_pin);
  if (prev_drvr_port == nullptr) {
    return false;
  }
  const LibertyCell* prev_cell = prev_drvr_port->libertyCell();
  if (prev_cell == nullptr) {
    return false;
  }

  if (prev_cell == drvr_cell) {
    return false;
  }

  if (((prev_cell->isBuffer() && drvr_cell->isBuffer())
          || (prev_cell->isInverter() && drvr_cell->isInverter()))
      && (resizer_->bufferDriveResistance(prev_cell)
          < resizer_->bufferDriveResistance(drvr_cell))) {
    if (replaceCell(drvr, prev_cell)) {
      debugPrint(logger_,
                 RSZ,
                 "opt_moves",
                 1,
                 "ACCEPT size_up_match {} {} -> {}",
                 network_->pathName(drvr_pin),
                 drvr_cell->name(),
                 prev_cell->name());
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 3,
                 "size_up_match {} {} -> {}",
                 network_->pathName(drvr_pin),
                 drvr_cell->name(),
                 prev_cell->name());
      addMove(drvr);
      return true;
    }
  }
  debugPrint(logger_,
             RSZ,
             "opt_moves",
             1,
             "REJECT size_up_match {} {}",
             network_->pathName(drvr_pin),
             drvr_cell->name());

  return false;
}

}  // namespace rsz
