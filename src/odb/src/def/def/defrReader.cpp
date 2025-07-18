// *****************************************************************************
// *****************************************************************************
// Copyright 2013 - 2017, Cadence Design Systems
//
// This  file  is  part  of  the  Cadence  LEF/DEF  Open   Source
// Distribution,  Product Version 5.8.
//
// Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//    implied. See the License for the specific language governing
//    permissions and limitations under the License.
//
// For updates, support, or to become part of the LEF/DEF Community,
// check www.openeda.org for details.
//
//  $Author: dell $
//  $Revision: #1 $
//  $Date: 2020/09/29 $
//  $State:  $
// *****************************************************************************
// *****************************************************************************

#include "defrReader.hpp"

#include <sys/stat.h>

#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <string_view>

#include "defiDebug.hpp"
#include "defiMisc.hpp"
#include "defiProp.hpp"
#include "defiPropType.hpp"
#include "defiUtil.hpp"
#include "defrCallBacks.hpp"
#include "defrData.hpp"
#include "defrSettings.hpp"

#define NODEFMSG 4013  // (9012 + 1) - 5000, def msg starts at 5000

#define DEF_INIT def_init(__FUNCTION__)

BEGIN_DEF_PARSER_NAMESPACE

extern int defyyparse(defrData* data);

extern defrContext defContext;

void def_init(const char* func)
{
  // Need for debugging config re-owning;
  if (defContext.ownConfig) {
    return;
  }

  if (defContext.settings == nullptr) {
    defContext.settings = new defrSettings();
    defContext.init_call_func = func;
  }

  if (defContext.callbacks == nullptr) {
    defContext.callbacks = new defrCallbacks();
    defContext.init_call_func = func;
  }

  if (defContext.session == nullptr) {
    defContext.session = new defrSession();
    defContext.init_call_func = func;
  }
}

int defrCountUnused(defrCallbackType_e e, void* v, defiUserData d)
{
  DEF_INIT;
  int i;
  if (defiDebug(23)) {
    printf("Count %d, 0x%p, 0x%p\n", (int) e, v, d);
  }
  i = (int) e;
  if (i <= 0 || i >= CBMAX) {
    return 1;
  }

  defContext.settings->UnusedCallbacks[i] += 1;

  return 0;
}

const char* typeToString(defrCallbackType_e num)
{
  switch ((int) num) {
    case defrUnspecifiedCbkType:
      return "Unspecified";
    case defrDesignStartCbkType:
      return "Design Start";
    case defrTechNameCbkType:
      return "Tech Name";
    case defrPropCbkType:
      return "Property";
    case defrPropDefEndCbkType:
      return "Property Definitions Section End";
    case defrPropDefStartCbkType:
      return "Property Definitions Section Start";
    case defrFloorPlanNameCbkType:
      return "FloorPlanName";
    case defrArrayNameCbkType:
      return "Array Name";
    case defrUnitsCbkType:
      return "Units";
    case defrDividerCbkType:
      return "Divider";
    case defrBusBitCbkType:
      return "BusBit Character";
    case defrSiteCbkType:
      return "Site";
    case defrComponentMaskShiftLayerCbkType:
      return "ComponentMaskShiftLayer";
    case defrComponentStartCbkType:
      return "Components Section Start";
    case defrComponentCbkType:
      return "Component";
    case defrComponentEndCbkType:
      return "Components Section End";
    case defrNetStartCbkType:
      return "Nets Section Start";
    case defrNetCbkType:
      return "Net";
    case defrNetNameCbkType:
      return "Net Name";
    case defrNetNonDefaultRuleCbkType:
      return "Net Nondefaultrule";
    case defrNetSubnetNameCbkType:
      return "Net Subnet Name";
    case defrNetEndCbkType:
      return "Nets Section End";
    case defrPathCbkType:
      return "Path";
    case defrVersionCbkType:
      return "Version";
    case defrVersionStrCbkType:
      return "Version";
    case defrComponentExtCbkType:
      return "Component User Extention";
    case defrPinExtCbkType:
      return "Pin User Extension";
    case defrViaExtCbkType:
      return "Via User Extension";
    case defrNetConnectionExtCbkType:
      return "NetConnection User Extention";
    case defrNetExtCbkType:
      return "Net User Extension";
    case defrGroupExtCbkType:
      return "Group User Extension";
    case defrScanChainExtCbkType:
      return "ScanChain User Extension";
    case defrIoTimingsExtCbkType:
      return "IoTimings User Extension";
    case defrPartitionsExtCbkType:
      return "Partitions User Extension";
    case defrHistoryCbkType:
      return "History";
    case defrDieAreaCbkType:
      return "DieArea";
    case defrCanplaceCbkType:
      return "Canplace";
    case defrCannotOccupyCbkType:
      return "CannotOccupy";
    case defrPinCapCbkType:
      return "PinCap";
    case defrDefaultCapCbkType:
      return "DefaultCap";
    case defrStartPinsCbkType:
      return "Start Pins Section";
    case defrPinCbkType:
      return "Pin";
    case defrPinEndCbkType:
      return "End Pins Section";
    case defrRowCbkType:
      return "Row";
    case defrTrackCbkType:
      return "Track";
    case defrGcellGridCbkType:
      return "GcellGrid";
    case defrViaStartCbkType:
      return "Start Vias Section";
    case defrViaCbkType:
      return "Via";
    case defrViaEndCbkType:
      return "End Vias Section";
    case defrRegionStartCbkType:
      return "Region Section Start";
    case defrRegionCbkType:
      return "Region";
    case defrRegionEndCbkType:
      return "Region Section End";
    case defrSNetStartCbkType:
      return "Special Net Section Start";
    case defrSNetCbkType:
      return "Special Net";
    case defrSNetEndCbkType:
      return "Special Net Section End";
    case defrGroupsStartCbkType:
      return "Groups Section Start";
    case defrGroupNameCbkType:
      return "Group Name";
    case defrGroupMemberCbkType:
      return "Group Member";
    case defrGroupCbkType:
      return "Group";
    case defrGroupsEndCbkType:
      return "Groups Section End";
    case defrAssertionsStartCbkType:
      return "Assertions Section Start";
    case defrAssertionCbkType:
      return "Assertion";
    case defrAssertionsEndCbkType:
      return "Assertions Section End";
    case defrConstraintsStartCbkType:
      return "Constraints Section Start";
    case defrConstraintCbkType:
      return "Constraint";
    case defrConstraintsEndCbkType:
      return "Constraints Section End";
    case defrScanchainsStartCbkType:
      return "Scanchains Section Start";
    case defrScanchainCbkType:
      return "Scanchain";
    case defrScanchainsEndCbkType:
      return "Scanchains Section End";
    case defrIOTimingsStartCbkType:
      return "IOTimings Section Start";
    case defrIOTimingCbkType:
      return "IOTiming";
    case defrIOTimingsEndCbkType:
      return "IOTimings Section End";
    case defrFPCStartCbkType:
      return "Floor Plan Constraints Section Start";
    case defrFPCCbkType:
      return "Floor Plan Constraint";
    case defrFPCEndCbkType:
      return "Floor Plan Constraints Section End";
    case defrTimingDisablesStartCbkType:
      return "TimingDisables Section Start";
    case defrTimingDisableCbkType:
      return "TimingDisable";
    case defrTimingDisablesEndCbkType:
      return "TimingDisables Section End";
    case defrPartitionsStartCbkType:
      return "Partitions Section Start";
    case defrPartitionCbkType:
      return "Partition";
    case defrPartitionsEndCbkType:
      return "Partitions Section End";
    case defrPinPropStartCbkType:
      return "PinProp Section Start";
    case defrPinPropCbkType:
      return "PinProp";
    case defrPinPropEndCbkType:
      return "PinProp Section End";
    case defrCaseSensitiveCbkType:
      return "CaseSensitive";
    case defrBlockageStartCbkType:
      return "Blockage Section Start";
    case defrBlockageCbkType:
      return "Blockage";
    case defrBlockageEndCbkType:
      return "Blockage Section End";
    case defrSlotStartCbkType:
      return "Slots Section Start";
    case defrSlotCbkType:
      return "Slots";
    case defrSlotEndCbkType:
      return "Slots Section End";
    case defrFillStartCbkType:
      return "Fills Section Start";
    case defrFillCbkType:
      return "Fills";
    case defrFillEndCbkType:
      return "Fills Section End";
    case defrNonDefaultStartCbkType:
      return "NonDefaultRule Section Start";
    case defrNonDefaultCbkType:
      return "NonDefault";
    case defrNonDefaultEndCbkType:
      return "NonDefaultRule Section End";
    case defrStylesStartCbkType:
      return "Styles Section Start";
    case defrStylesCbkType:
      return "Styles";
    case defrStylesEndCbkType:
      return "Styles Section End";
    case defrExtensionCbkType:
      return "Extension";

      // NEW CALLBACK - If you created a new callback then add the
      // type enums that you created here for debug printing.

    case defrDesignEndCbkType:
      return "DesignEnd";
    default:
      break;
  }
  return "BOGUS";
}

int defrCatchAll(defrCallbackType_e typ, void*, defiUserData)
{
  DEF_INIT;

  if ((int) typ >= 0 && (int) typ < CBMAX) {
    defContext.settings->UnusedCallbacks[(int) typ] += 1;
  } else {
    defContext.settings->UnusedCallbacks[0] += 1;
    return 1;
  }

  return 0;
}

// *****************************************************************
// Wrapper functions.
//
// These functions provide access to the class member functions
// for compatibility with previous parser kits. Returns non-zero
// status if the initialization is failed.
// *****************************************************************
int defrInit()
{
  return defrInitSession(0);
}

int defrInitSession(int startSession)
{
  if (startSession) {
    if (defContext.init_call_func != nullptr) {
      fprintf(stderr,
              "ERROR: Attempt to call configuration function '%s' in DEF "
              "parser before defrInit() call in session-based mode.\n",
              defContext.init_call_func);
      return 1;
    }

    delete defContext.settings;
    defContext.settings = new defrSettings();

    delete defContext.callbacks;
    defContext.callbacks = new defrCallbacks();

    delete defContext.session;
    defContext.session = new defrSession();
  } else {
    if (defContext.callbacks == nullptr) {
      defContext.callbacks = new defrCallbacks();
    }

    if (defContext.settings == nullptr) {
      defContext.settings = new defrSettings();
    }

    if (defContext.session == nullptr) {
      defContext.session = new defrSession();
    } else {
      memset(defContext.settings->UnusedCallbacks, 0, CBMAX * sizeof(int));
    }
  }

  defContext.ownConfig = 0;
  defContext.init_call_func = nullptr;

  return 0;
}

// obsoleted now
int defrReset()
{
  return 0;
}

int defrClear()
{
  delete defContext.callbacks;
  defContext.callbacks = nullptr;

  delete defContext.settings;
  defContext.settings = nullptr;

  delete defContext.session;
  defContext.session = nullptr;

  delete defContext.data;
  defContext.data = nullptr;

  defContext.init_call_func = nullptr;
  defContext.ownConfig = 0;

  return 0;
}

void defrSetRegisterUnusedCallbacks()
{
  DEF_INIT;
  defrSetUnusedCallbacks(defrCountUnused);
}

void defrSetUnusedCallbacks(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SetUnusedCallbacks(f);
}

void defrUnsetCallbacks()
{
  DEF_INIT;
  delete defContext.callbacks;
  defContext.callbacks = new defrCallbacks();
}

void defrPrintUnusedCallbacks(FILE* log)
{
  int i;
  int first = 1;

  for (i = 0; i < CBMAX; i++) {
    if (defContext.settings->UnusedCallbacks[i]) {
      if (first) {
        fprintf(log,
                "WARNING (DEFPARS-5001): DEF statement found in the def file "
                "with no callback set.\n");
        first = 0;
      }
      fprintf(log,
              "%5d %s\n",
              defContext.settings->UnusedCallbacks[i],
              typeToString((defrCallbackType_e) i));
    }
  }
}

// obsoleted
int defrReleaseNResetMemory()
{
  return 0;
}

void defrUnsetArrayNameCbk()
{
  DEF_INIT;
  defContext.callbacks->ArrayNameCbk = nullptr;
}

void defrUnsetAssertionCbk()
{
  DEF_INIT;
  defContext.callbacks->AssertionCbk = nullptr;
}

void defrUnsetAssertionsStartCbk()
{
  DEF_INIT;
  defContext.callbacks->AssertionsStartCbk = nullptr;
}

void defrUnsetAssertionsEndCbk()
{
  DEF_INIT;
  defContext.callbacks->AssertionsEndCbk = nullptr;
}

void defrUnsetBlockageCbk()
{
  DEF_INIT;
  defContext.callbacks->BlockageCbk = nullptr;
}

void defrUnsetBlockageStartCbk()
{
  DEF_INIT;
  defContext.callbacks->BlockageStartCbk = nullptr;
}

void defrUnsetBlockageEndCbk()
{
  DEF_INIT;
  defContext.callbacks->BlockageEndCbk = nullptr;
}

void defrUnsetBusBitCbk()
{
  DEF_INIT;
  defContext.callbacks->BusBitCbk = nullptr;
}

void defrUnsetCannotOccupyCbk()
{
  DEF_INIT;
  defContext.callbacks->CannotOccupyCbk = nullptr;
}

void defrUnsetCanplaceCbk()
{
  DEF_INIT;
  defContext.callbacks->CanplaceCbk = nullptr;
}

void defrUnsetCaseSensitiveCbk()
{
  DEF_INIT;
  defContext.callbacks->CaseSensitiveCbk = nullptr;
}

void defrUnsetComponentCbk()
{
  DEF_INIT;
  defContext.callbacks->ComponentCbk = nullptr;
}

void defrUnsetComponentExtCbk()
{
  DEF_INIT;
  defContext.callbacks->ComponentExtCbk = nullptr;
}

void defrUnsetComponentStartCbk()
{
  DEF_INIT;
  defContext.callbacks->ComponentStartCbk = nullptr;
}

void defrUnsetComponentEndCbk()
{
  DEF_INIT;
  defContext.callbacks->ComponentEndCbk = nullptr;
}

void defrUnsetConstraintCbk()
{
  DEF_INIT;
  defContext.callbacks->ConstraintCbk = nullptr;
}

void defrUnsetConstraintsStartCbk()
{
  DEF_INIT;
  defContext.callbacks->ConstraintsStartCbk = nullptr;
}

void defrUnsetConstraintsEndCbk()
{
  DEF_INIT;
  defContext.callbacks->ConstraintsEndCbk = nullptr;
}

void defrUnsetDefaultCapCbk()
{
  DEF_INIT;
  defContext.callbacks->DefaultCapCbk = nullptr;
}

void defrUnsetDesignCbk()
{
  DEF_INIT;
  defContext.callbacks->DesignCbk = nullptr;
}

void defrUnsetDesignEndCbk()
{
  DEF_INIT;
  defContext.callbacks->DesignEndCbk = nullptr;
}

void defrUnsetDieAreaCbk()
{
  DEF_INIT;
  defContext.callbacks->DieAreaCbk = nullptr;
}

void defrUnsetDividerCbk()
{
  DEF_INIT;
  defContext.callbacks->DividerCbk = nullptr;
}

void defrUnsetExtensionCbk()
{
  DEF_INIT;
  defContext.callbacks->ExtensionCbk = nullptr;
}

void defrUnsetFillCbk()
{
  DEF_INIT;
  defContext.callbacks->FillCbk = nullptr;
}

void defrUnsetFillStartCbk()
{
  DEF_INIT;
  defContext.callbacks->FillStartCbk = nullptr;
}

void defrUnsetFillEndCbk()
{
  DEF_INIT;
  defContext.callbacks->FillEndCbk = nullptr;
}

void defrUnsetFPCCbk()
{
  DEF_INIT;
  defContext.callbacks->FPCCbk = nullptr;
}

void defrUnsetFPCStartCbk()
{
  DEF_INIT;
  defContext.callbacks->FPCStartCbk = nullptr;
}

void defrUnsetFPCEndCbk()
{
  DEF_INIT;
  defContext.callbacks->FPCEndCbk = nullptr;
}

void defrUnsetFloorPlanNameCbk()
{
  DEF_INIT;
  defContext.callbacks->FloorPlanNameCbk = nullptr;
}

void defrUnsetGcellGridCbk()
{
  DEF_INIT;
  defContext.callbacks->GcellGridCbk = nullptr;
}

void defrUnsetGroupCbk()
{
  DEF_INIT;
  defContext.callbacks->GroupCbk = nullptr;
}

void defrUnsetGroupExtCbk()
{
  DEF_INIT;
  defContext.callbacks->GroupExtCbk = nullptr;
}

void defrUnsetGroupMemberCbk()
{
  DEF_INIT;
  defContext.callbacks->GroupMemberCbk = nullptr;
}

void defrUnsetComponentMaskShiftLayerCbk()
{
  DEF_INIT;
  defContext.callbacks->ComponentMaskShiftLayerCbk = nullptr;
}

void defrUnsetGroupNameCbk()
{
  DEF_INIT;
  defContext.callbacks->GroupNameCbk = nullptr;
}

void defrUnsetGroupsStartCbk()
{
  DEF_INIT;
  defContext.callbacks->GroupsStartCbk = nullptr;
}

void defrUnsetGroupsEndCbk()
{
  DEF_INIT;
  defContext.callbacks->GroupsEndCbk = nullptr;
}

void defrUnsetHistoryCbk()
{
  DEF_INIT;
  defContext.callbacks->HistoryCbk = nullptr;
}

void defrUnsetIOTimingCbk()
{
  DEF_INIT;
  defContext.callbacks->IOTimingCbk = nullptr;
}

void defrUnsetIOTimingsStartCbk()
{
  DEF_INIT;
  defContext.callbacks->IOTimingsStartCbk = nullptr;
}

void defrUnsetIOTimingsEndCbk()
{
  DEF_INIT;
  defContext.callbacks->IOTimingsEndCbk = nullptr;
}

void defrUnsetIOTimingsExtCbk()
{
  DEF_INIT;
  defContext.callbacks->IoTimingsExtCbk = nullptr;
}

void defrUnsetNetCbk()
{
  DEF_INIT;
  defContext.callbacks->NetCbk = nullptr;
}

void defrUnsetNetNameCbk()
{
  DEF_INIT;
  defContext.callbacks->NetNameCbk = nullptr;
}

void defrUnsetNetNonDefaultRuleCbk()
{
  DEF_INIT;
  defContext.callbacks->NetNonDefaultRuleCbk = nullptr;
}

void defrUnsetNetConnectionExtCbk()
{
  DEF_INIT;
  defContext.callbacks->NetConnectionExtCbk = nullptr;
}

void defrUnsetNetExtCbk()
{
  DEF_INIT;
  defContext.callbacks->NetExtCbk = nullptr;
}

void defrUnsetNetPartialPathCbk()
{
  DEF_INIT;
  defContext.callbacks->NetPartialPathCbk = nullptr;
}

void defrUnsetNetSubnetNameCbk()
{
  DEF_INIT;
  defContext.callbacks->NetSubnetNameCbk = nullptr;
}

void defrUnsetNetStartCbk()
{
  DEF_INIT;
  defContext.callbacks->NetStartCbk = nullptr;
}

void defrUnsetNetEndCbk()
{
  DEF_INIT;
  defContext.callbacks->NetEndCbk = nullptr;
}

void defrUnsetNonDefaultCbk()
{
  DEF_INIT;
  defContext.callbacks->NonDefaultCbk = nullptr;
}

void defrUnsetNonDefaultStartCbk()
{
  DEF_INIT;
  defContext.callbacks->NonDefaultStartCbk = nullptr;
}

void defrUnsetNonDefaultEndCbk()
{
  DEF_INIT;
  defContext.callbacks->NonDefaultEndCbk = nullptr;
}

void defrUnsetPartitionCbk()
{
  DEF_INIT;
  defContext.callbacks->PartitionCbk = nullptr;
}

void defrUnsetPartitionsExtCbk()
{
  DEF_INIT;
  defContext.callbacks->PartitionsExtCbk = nullptr;
}

void defrUnsetPartitionsStartCbk()
{
  DEF_INIT;
  defContext.callbacks->PartitionsStartCbk = nullptr;
}

void defrUnsetPartitionsEndCbk()
{
  DEF_INIT;
  defContext.callbacks->PartitionsEndCbk = nullptr;
}

void defrUnsetPathCbk()
{
  DEF_INIT;
  defContext.callbacks->PathCbk = nullptr;
}

void defrUnsetPinCapCbk()
{
  DEF_INIT;
  defContext.callbacks->PinCapCbk = nullptr;
}

void defrUnsetPinCbk()
{
  DEF_INIT;
  defContext.callbacks->PinCbk = nullptr;
}

void defrUnsetPinEndCbk()
{
  DEF_INIT;
  defContext.callbacks->PinEndCbk = nullptr;
}

void defrUnsetPinExtCbk()
{
  DEF_INIT;
  defContext.callbacks->PinExtCbk = nullptr;
}

void defrUnsetPinPropCbk()
{
  DEF_INIT;
  defContext.callbacks->PinPropCbk = nullptr;
}

void defrUnsetPinPropStartCbk()
{
  DEF_INIT;
  defContext.callbacks->PinPropStartCbk = nullptr;
}

void defrUnsetPinPropEndCbk()
{
  DEF_INIT;
  defContext.callbacks->PinPropEndCbk = nullptr;
}

void defrUnsetPropCbk()
{
  DEF_INIT;
  defContext.callbacks->PropCbk = nullptr;
}

void defrUnsetPropDefEndCbk()
{
  DEF_INIT;
  defContext.callbacks->PropDefEndCbk = nullptr;
}

void defrUnsetPropDefStartCbk()
{
  DEF_INIT;
  defContext.callbacks->PropDefStartCbk = nullptr;
}

void defrUnsetRegionCbk()
{
  DEF_INIT;
  defContext.callbacks->RegionCbk = nullptr;
}

void defrUnsetRegionStartCbk()
{
  DEF_INIT;
  defContext.callbacks->RegionStartCbk = nullptr;
}

void defrUnsetRegionEndCbk()
{
  DEF_INIT;
  defContext.callbacks->RegionEndCbk = nullptr;
}

void defrUnsetRowCbk()
{
  DEF_INIT;
  defContext.callbacks->RowCbk = nullptr;
}

void defrUnsetScanChainExtCbk()
{
  DEF_INIT;
  defContext.callbacks->ScanChainExtCbk = nullptr;
}

void defrUnsetScanchainCbk()
{
  DEF_INIT;
  defContext.callbacks->ScanchainCbk = nullptr;
}

void defrUnsetScanchainsStartCbk()
{
  DEF_INIT;
  defContext.callbacks->ScanchainsStartCbk = nullptr;
}

void defrUnsetScanchainsEndCbk()
{
  DEF_INIT;
  defContext.callbacks->ScanchainsEndCbk = nullptr;
}

void defrUnsetSiteCbk()
{
  DEF_INIT;
  defContext.callbacks->SiteCbk = nullptr;
}

void defrUnsetSlotCbk()
{
  DEF_INIT;
  defContext.callbacks->SlotCbk = nullptr;
}

void defrUnsetSlotStartCbk()
{
  DEF_INIT;
  defContext.callbacks->SlotStartCbk = nullptr;
}

void defrUnsetSlotEndCbk()
{
  DEF_INIT;
  defContext.callbacks->SlotEndCbk = nullptr;
}

void defrUnsetSNetWireCbk()
{
  DEF_INIT;
  defContext.callbacks->SNetWireCbk = nullptr;
}

void defrUnsetSNetCbk()
{
  DEF_INIT;
  defContext.callbacks->SNetCbk = nullptr;
}

void defrUnsetSNetStartCbk()
{
  DEF_INIT;
  defContext.callbacks->SNetStartCbk = nullptr;
}

void defrUnsetSNetEndCbk()
{
  DEF_INIT;
  defContext.callbacks->SNetEndCbk = nullptr;
}

void defrUnsetSNetPartialPathCbk()
{
  DEF_INIT;
  defContext.callbacks->SNetPartialPathCbk = nullptr;
}

void defrUnsetStartPinsCbk()
{
  DEF_INIT;
  defContext.callbacks->StartPinsCbk = nullptr;
}

void defrUnsetStylesCbk()
{
  DEF_INIT;
  defContext.callbacks->StylesCbk = nullptr;
}

void defrUnsetStylesStartCbk()
{
  DEF_INIT;
  defContext.callbacks->StylesStartCbk = nullptr;
}

void defrUnsetStylesEndCbk()
{
  DEF_INIT;
  defContext.callbacks->StylesEndCbk = nullptr;
}

void defrUnsetTechnologyCbk()
{
  DEF_INIT;
  defContext.callbacks->TechnologyCbk = nullptr;
}

void defrUnsetTimingDisableCbk()
{
  DEF_INIT;
  defContext.callbacks->TimingDisableCbk = nullptr;
}

void defrUnsetTimingDisablesStartCbk()
{
  DEF_INIT;
  defContext.callbacks->TimingDisablesStartCbk = nullptr;
}

void defrUnsetTimingDisablesEndCbk()
{
  DEF_INIT;
  defContext.callbacks->TimingDisablesEndCbk = nullptr;
}

void defrUnsetTrackCbk()
{
  DEF_INIT;
  defContext.callbacks->TrackCbk = nullptr;
}

void defrUnsetUnitsCbk()
{
  DEF_INIT;
  defContext.callbacks->UnitsCbk = nullptr;
}

void defrUnsetVersionCbk()
{
  DEF_INIT;
  defContext.callbacks->VersionCbk = nullptr;
}

void defrUnsetVersionStrCbk()
{
  DEF_INIT;
  defContext.callbacks->VersionStrCbk = nullptr;
}

void defrUnsetViaCbk()
{
  DEF_INIT;
  defContext.callbacks->ViaCbk = nullptr;
}

void defrUnsetViaExtCbk()
{
  DEF_INIT;
  defContext.callbacks->ViaExtCbk = nullptr;
}

void defrUnsetViaStartCbk()
{
  DEF_INIT;
  defContext.callbacks->ViaStartCbk = nullptr;
}

void defrUnsetViaEndCbk()
{
  DEF_INIT;
  defContext.callbacks->ViaEndCbk = nullptr;
}

int* defUnusedCallbackCount()
{
  DEF_INIT;
  return defContext.settings->UnusedCallbacks;
}

const char* defrFName()
{
  DEF_INIT;
  return nullptr;
}

void defrClearSession()
{
  if (defContext.session) {
    delete defContext.session;
    defContext.session = new defrSession();
  }
}

int defrRead(FILE* f, const char* fName, defiUserData uData, int case_sensitive)
{
  int status;

  delete defContext.data;

  defrData* defData = new defrData(
      defContext.callbacks, defContext.settings, defContext.session);

  defContext.data = defData;

  // lex_init
  struct stat statbuf;

  /* 4/11/2003 - Remove file lefrRWarning.log from directory if it exist */
  /* pcr 569729 */
  if (stat("defRWarning.log", &statbuf) != -1) {
    /* file exist, remove it */
    if (!defContext.settings->LogFileAppend) {
      remove("defRWarning.log");
    }
  }

  // Propagate Settings parameter to Data.
  if (defData->settings->reader_case_sensitive_set) {
    defData->names_case_sensitive = defData->session->reader_case_sensitive;
  } else if (defData->VersionNum > 5.5) {
    defData->names_case_sensitive = true;
  }

  defData->session->FileName = (char*) fName;
  defData->File = f;
  defData->session->UserData = uData;
  defData->session->reader_case_sensitive = case_sensitive;

  // Create a path pointer that is all ready to go just in case
  // we need it later.

  defData->NeedPathData
      = (((defData->callbacks->NetCbk || defData->callbacks->SNetCbk)
          && defData->settings->AddPathToNet)
         || defData->callbacks->PathCbk)
            ? 1
            : 0;
  if (defData->NeedPathData) {
    defData->PathObj.Init();
  }

  status = defyyparse(defData);

  return status;
}

void defrSetUserData(defiUserData ud)
{
  DEF_INIT;
  defContext.session->UserData = ud;
}

defiUserData defrGetUserData()
{
  return defContext.session->UserData;
}

void defrSetDesignCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->DesignCbk = f;
}

void defrSetTechnologyCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->TechnologyCbk = f;
}

void defrSetDesignEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->DesignEndCbk = f;
}

void defrSetPropCbk(defrPropCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PropCbk = f;
}

void defrSetPropDefEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PropDefEndCbk = f;
}

void defrSetPropDefStartCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PropDefStartCbk = f;
}

void defrSetArrayNameCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ArrayNameCbk = f;
}

void defrSetFloorPlanNameCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FloorPlanNameCbk = f;
}

void defrSetUnitsCbk(defrDoubleCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->UnitsCbk = f;
}

void defrSetVersionCbk(defrDoubleCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->VersionCbk = f;
}

void defrSetVersionStrCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->VersionStrCbk = f;
}

void defrSetDividerCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->DividerCbk = f;
}

void defrSetBusBitCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->BusBitCbk = f;
}

void defrSetSiteCbk(defrSiteCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SiteCbk = f;
}

void defrSetCanplaceCbk(defrSiteCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->CanplaceCbk = f;
}

void defrSetCannotOccupyCbk(defrSiteCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->CannotOccupyCbk = f;
}

void defrSetComponentStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ComponentStartCbk = f;
}

void defrSetComponentEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ComponentEndCbk = f;
}

void defrSetComponentCbk(defrComponentCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ComponentCbk = f;
}

void defrSetNetStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetStartCbk = f;
}

void defrSetNetEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetEndCbk = f;
}

void defrSetNetCbk(defrNetCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetCbk = f;
}

void defrSetNetNameCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetNameCbk = f;
}

void defrSetNetSubnetNameCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetSubnetNameCbk = f;
}

void defrSetNetNonDefaultRuleCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetNonDefaultRuleCbk = f;
}

void defrSetNetPartialPathCbk(defrNetCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetPartialPathCbk = f;
}

void defrSetSNetStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SNetStartCbk = f;
}

void defrSetSNetEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SNetEndCbk = f;
}

void defrSetSNetCbk(defrNetCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SNetCbk = f;
}

void defrSetSNetPartialPathCbk(defrNetCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SNetPartialPathCbk = f;
}

void defrSetSNetWireCbk(defrNetCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SNetWireCbk = f;
}

void defrSetPathCbk(defrPathCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PathCbk = f;
}

void defrSetAddPathToNet()
{
  DEF_INIT;
  defContext.settings->AddPathToNet = 1;
}

void defrSetAllowComponentNets()
{
  DEF_INIT;
  defContext.settings->AllowComponentNets = 1;
}

int defrGetAllowComponentNets()
{
  DEF_INIT;
  return defContext.settings->AllowComponentNets;
}

void defrSetComponentExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ComponentExtCbk = f;
}

void defrSetPinExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinExtCbk = f;
}

void defrSetViaExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ViaExtCbk = f;
}

void defrSetNetConnectionExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetConnectionExtCbk = f;
}

void defrSetNetExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NetExtCbk = f;
}

void defrSetGroupExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GroupExtCbk = f;
}

void defrSetScanChainExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ScanChainExtCbk = f;
}

void defrSetIoTimingsExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->IoTimingsExtCbk = f;
}

void defrSetPartitionsExtCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PartitionsExtCbk = f;
}

void defrSetHistoryCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->HistoryCbk = f;
}

void defrSetDieAreaCbk(defrBoxCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->DieAreaCbk = f;
}

void defrSetPinCapCbk(defrPinCapCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinCapCbk = f;
}

void defrSetPinEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinEndCbk = f;
}

void defrSetStartPinsCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->StartPinsCbk = f;
}

void defrSetDefaultCapCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->DefaultCapCbk = f;
}

void defrSetPinCbk(defrPinCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinCbk = f;
}

void defrSetRowCbk(defrRowCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->RowCbk = f;
}

void defrSetTrackCbk(defrTrackCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->TrackCbk = f;
}

void defrSetGcellGridCbk(defrGcellGridCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GcellGridCbk = f;
}

void defrSetViaStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ViaStartCbk = f;
}

void defrSetViaEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ViaEndCbk = f;
}

void defrSetViaCbk(defrViaCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ViaCbk = f;
}

void defrSetRegionStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->RegionStartCbk = f;
}

void defrSetRegionEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->RegionEndCbk = f;
}

void defrSetRegionCbk(defrRegionCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->RegionCbk = f;
}

void defrSetGroupsStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GroupsStartCbk = f;
}

void defrSetGroupsEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GroupsEndCbk = f;
}

void defrSetGroupNameCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GroupNameCbk = f;
}

void defrSetGroupMemberCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GroupMemberCbk = f;
}

void defrSetComponentMaskShiftLayerCbk(defrComponentMaskShiftLayerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ComponentMaskShiftLayerCbk = f;
}

void defrSetGroupCbk(defrGroupCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->GroupCbk = f;
}

void defrSetAssertionsStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->AssertionsStartCbk = f;
}

void defrSetAssertionsEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->AssertionsEndCbk = f;
}

void defrSetAssertionCbk(defrAssertionCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->AssertionCbk = f;
}

void defrSetConstraintsStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ConstraintsStartCbk = f;
}

void defrSetConstraintsEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ConstraintsEndCbk = f;
}

void defrSetConstraintCbk(defrAssertionCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ConstraintCbk = f;
}

void defrSetScanchainsStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ScanchainsStartCbk = f;
}

void defrSetScanchainsEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ScanchainsEndCbk = f;
}

void defrSetScanchainCbk(defrScanchainCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ScanchainCbk = f;
}

void defrSetIOTimingsStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->IOTimingsStartCbk = f;
}

void defrSetIOTimingsEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->IOTimingsEndCbk = f;
}

void defrSetIOTimingCbk(defrIOTimingCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->IOTimingCbk = f;
}

void defrSetFPCStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FPCStartCbk = f;
}

void defrSetFPCEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FPCEndCbk = f;
}

void defrSetFPCCbk(defrFPCCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FPCCbk = f;
}

void defrSetTimingDisablesStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->TimingDisablesStartCbk = f;
}

void defrSetTimingDisablesEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->TimingDisablesEndCbk = f;
}

void defrSetTimingDisableCbk(defrTimingDisableCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->TimingDisableCbk = f;
}

void defrSetPartitionsStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PartitionsStartCbk = f;
}

void defrSetPartitionsEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PartitionsEndCbk = f;
}

void defrSetPartitionCbk(defrPartitionCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PartitionCbk = f;
}

void defrSetPinPropStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinPropStartCbk = f;
}

void defrSetPinPropEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinPropEndCbk = f;
}

void defrSetPinPropCbk(defrPinPropCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->PinPropCbk = f;
}

void defrSetCaseSensitiveCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->CaseSensitiveCbk = f;
}

void defrSetBlockageStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->BlockageStartCbk = f;
}

void defrSetBlockageEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->BlockageEndCbk = f;
}

void defrSetBlockageCbk(defrBlockageCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->BlockageCbk = f;
}

void defrSetSlotStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SlotStartCbk = f;
}

void defrSetSlotEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SlotEndCbk = f;
}

void defrSetSlotCbk(defrSlotCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->SlotCbk = f;
}

void defrSetFillStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FillStartCbk = f;
}

void defrSetFillEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FillEndCbk = f;
}

void defrSetFillCbk(defrFillCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->FillCbk = f;
}

void defrSetNonDefaultStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NonDefaultStartCbk = f;
}

void defrSetNonDefaultEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NonDefaultEndCbk = f;
}

void defrSetNonDefaultCbk(defrNonDefaultCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->NonDefaultCbk = f;
}

void defrSetStylesStartCbk(defrIntegerCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->StylesStartCbk = f;
}

void defrSetStylesEndCbk(defrVoidCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->StylesEndCbk = f;
}

void defrSetStylesCbk(defrStylesCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->StylesCbk = f;
}

void defrSetExtensionCbk(defrStringCbkFnType f)
{
  DEF_INIT;
  defContext.callbacks->ExtensionCbk = f;
}

// NEW CALLBACK - Put the set functions for the new callbacks here.

void defrSetAssertionWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->AssertionWarnings = warn;
}

void defrSetBlockageWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->BlockageWarnings = warn;
}

void defrSetCaseSensitiveWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->CaseSensitiveWarnings = warn;
}

void defrSetComponentWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->ComponentWarnings = warn;
}

void defrSetConstraintWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->ConstraintWarnings = warn;
}

void defrSetDefaultCapWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->DefaultCapWarnings = warn;
}

void defrSetGcellGridWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->GcellGridWarnings = warn;
}

void defrSetIOTimingWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->IOTimingWarnings = warn;
}

void defrSetNetWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->NetWarnings = warn;
}

void defrSetNonDefaultWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->NonDefaultWarnings = warn;
}

void defrSetPinExtWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->PinExtWarnings = warn;
}

void defrSetPinWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->PinWarnings = warn;
}

void defrSetRegionWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->RegionWarnings = warn;
}

void defrSetRowWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->RowWarnings = warn;
}

void defrSetScanchainWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->ScanchainWarnings = warn;
}

void defrSetSNetWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->SNetWarnings = warn;
}

void defrSetStylesWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->StylesWarnings = warn;
}

void defrSetTrackWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->TrackWarnings = warn;
}

void defrSetUnitsWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->UnitsWarnings = warn;
}

void defrSetVersionWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->VersionWarnings = warn;
}

void defrSetViaWarnings(int warn)
{
  DEF_INIT;
  defContext.settings->ViaWarnings = warn;
}

void defrDisableParserMsgs(int nMsg, int* msgs)
{
  DEF_INIT;
  int i, j;
  int* tmp;

  if (defContext.settings->nDDMsgs == 0) {
    defContext.settings->nDDMsgs = nMsg;
    defContext.settings->disableDMsgs = (int*) malloc(sizeof(int) * nMsg);
    for (i = 0; i < nMsg; i++) {
      defContext.settings->disableDMsgs[i] = msgs[i];
    }
  } else {  // add the list to the existing list
    // 1st check if the msgId is already on the list before adding it on
    tmp = (int*) malloc(sizeof(int) * (nMsg + defContext.settings->nDDMsgs));
    for (i = 0; i < defContext.settings->nDDMsgs;
         i++) {  // copy the existing to the new list
      tmp[i] = defContext.settings->disableDMsgs[i];
    }
    free((int*) (defContext.settings->disableDMsgs));
    defContext.settings->disableDMsgs
        = tmp;                    // set disableDMsgs to the new list
    for (i = 0; i < nMsg; i++) {  // merge the new list with the existing
      for (j = 0; j < defContext.settings->nDDMsgs; j++) {
        if (defContext.settings->disableDMsgs[j] == msgs[i]) {
          break;  // msgId already on the list
        }
      }
      if (j
          == defContext.settings
                 ->nDDMsgs) {  // msgId not on the list, add it on
        defContext.settings->disableDMsgs[defContext.settings->nDDMsgs++]
            = msgs[i];
      }
    }
  }
  return;
}

void defrEnableParserMsgs(int nMsg, int* msgs)
{
  DEF_INIT;
  int i, j;

  if (defContext.settings->nDDMsgs == 0) {
    return;  // list is empty, nothing to remove
  }

  for (i = 0; i < nMsg; i++) {  // loop through the given list
    for (j = 0; j < defContext.settings->nDDMsgs; j++) {
      if (defContext.settings->disableDMsgs[j] == msgs[i]) {
        defContext.settings->disableDMsgs[j]
            = -1;  // temp assign a -1 on that slot
        break;
      }
    }
  }
  // fill up the empty slot with the next non -1 msgId
  for (i = 0; i < defContext.settings->nDDMsgs; i++) {
    if (defContext.settings->disableDMsgs[i] == -1) {
      j = i + 1;
      while (j < defContext.settings->nDDMsgs) {
        if (defContext.settings->disableDMsgs[j] != -1) {
          defContext.settings->disableDMsgs[i++]
              = defContext.settings->disableDMsgs[j++];
        }
      }
      break;  // break out the for loop, the list should all moved
    }
  }
  // Count how many messageId left and change all -1 to 0
  for (j = i; j < defContext.settings->nDDMsgs; j++) {
    defContext.settings->disableDMsgs[j] = 0;  // set to 0
  }
  defContext.settings->nDDMsgs = i;
  return;
}

void defrEnableAllMsgs()
{
  DEF_INIT;
  defContext.settings->nDDMsgs = 0;
  free((int*) (defContext.settings->disableDMsgs));
}

void defrSetTotalMsgLimit(int totNumMsgs)
{
  DEF_INIT;
  defContext.settings->totalDefMsgLimit = totNumMsgs;
}

void defrSetLimitPerMsg(int msgId, int numMsg)
{
  DEF_INIT;
  char msgStr[10];

  if ((msgId <= 0) || ((msgId - 5000) >= NODEFMSG)) {  // Def starts at 5000
    sprintf(msgStr, "%d", msgId);
    return;
  }
  defContext.settings->MsgLimit[msgId - 5000] = numMsg;
  return;
}

// *****************************************************************
// Utility functions
//
// These are utility functions. Note: this part still contains some
// global variables. Ideally they would be part of the main class.
// *****************************************************************

void defrSetMagicCommentFoundFunction(DEFI_MAGIC_COMMENT_FOUND_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->MagicCommentFoundFunction = f;
}

void defrSetMagicCommentString(char* s)
{
  DEF_INIT;

  free(defContext.data->magic);
  defContext.data->magic = strdup(s);
}

void defrSetLogFunction(DEFI_LOG_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ErrorLogFunction = f;
}

void defrSetWarningLogFunction(DEFI_WARNING_LOG_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->WarningLogFunction = f;
}

void defrSetContextLogFunction(DEFI_CONTEXT_LOG_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ContextErrorLogFunction = f;
}

void defrSetContextWarningLogFunction(DEFI_CONTEXT_WARNING_LOG_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ContextWarningLogFunction = f;
}

void defrSetMallocFunction(DEFI_MALLOC_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->MallocFunction = f;
}

void defrSetReallocFunction(DEFI_REALLOC_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ReallocFunction = f;
}

void defrSetFreeFunction(DEFI_FREE_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->FreeFunction = f;
}

void defrSetLineNumberFunction(DEFI_LINE_NUMBER_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->LineNumberFunction = f;
}

void defrSetLongLineNumberFunction(DEFI_LONG_LINE_NUMBER_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->LongLineNumberFunction = f;
}

void defrSetContextLineNumberFunction(DEFI_CONTEXT_LINE_NUMBER_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ContextLineNumberFunction = f;
}

void defrSetContextLongLineNumberFunction(
    DEFI_CONTEXT_LONG_LINE_NUMBER_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ContextLongLineNumberFunction = f;
}

void defrSetDeltaNumberLines(int numLines)
{
  DEF_INIT;
  defContext.settings->defiDeltaNumberLines = numLines;
}

void defrSetCommentChar(char c)
{
  DEF_INIT;
  defContext.settings->CommentChar = c;
}

void defrSetCaseSensitivity(int caseSense)
{
  DEF_INIT;

  defContext.settings->reader_case_sensitive_set = 1;
  defContext.session->reader_case_sensitive = caseSense;
  if (defContext.data) {
    defContext.data->names_case_sensitive = caseSense;
  }
}

void defrAddAlias(std::string_view key, std::string_view value, int marked)
{
  // Since the alias data is stored in the hash table, the hash table
  // only takes the key and the data, the marked data will be stored
  // at the end of the value data

  if (marked != 0) {
    marked = 1;  // make sure only 1 digit
  }
  std::stringstream concated_value;
  concated_value << marked << value;
  defContext.data->def_alias_set[std::string(key)] = concated_value.str();
}

void defrSetOpenLogFileAppend()
{
  DEF_INIT;
  defContext.settings->LogFileAppend = true;
}

void defrUnsetOpenLogFileAppend()
{
  DEF_INIT;
  defContext.settings->LogFileAppend = false;
}

void defrSetReadFunction(DEFI_READ_FUNCTION f)
{
  DEF_INIT;
  defContext.settings->ReadFunction = f;
}

void defrUnsetReadFunction()
{
  DEF_INIT;
  defContext.settings->ReadFunction = nullptr;
}

void defrDisablePropStrProcess()
{
  DEF_INIT;
  defContext.settings->DisPropStrProcess = 1;
}

void defrSetNLines(long long n)
{
  defrData* defData = defContext.data;

  defData->nlines = n;
}

int defrLineNumber()
{
  // Compatibility feature: in old versions the translators,
  // the function can be called before defData initialization.
  if (defContext.data) {
    return (int) defContext.data->nlines;
  }

  return 0;
}

long long defrLongLineNumber()
{
  // Compatibility feature: in old versions the translators,
  // the function can be called before defData initialization.

  if (defContext.data) {
    return defContext.data->nlines;
  }

  return (long long) 0;
}

END_DEF_PARSER_NAMESPACE
