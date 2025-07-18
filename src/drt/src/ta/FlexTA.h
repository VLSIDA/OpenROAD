// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "db/obj/frVia.h"
#include "db/taObj/taPin.h"
#include "frDesign.h"

namespace drt {
class FlexTAGraphics;
class AbstractTAGraphics;

class FlexTA
{
 public:
  // constructors
  FlexTA(frDesign* in,
         Logger* logger,
         RouterConfiguration* router_cfg,
         bool save_updates_);
  ~FlexTA();
  // getters
  frTechObject* getTech() const { return tech_; }
  frDesign* getDesign() const { return design_; }
  // others
  int main();
  void setDebug(std::unique_ptr<AbstractTAGraphics> ta_graphics);

 private:
  frTechObject* tech_;
  frDesign* design_;
  Logger* logger_;
  RouterConfiguration* router_cfg_;
  bool save_updates_;
  std::unique_ptr<AbstractTAGraphics> graphics_;
  // others
  void main_helper(frLayerNum lNum, int maxOffsetIter, int panelWidth);
  void initTA(int size);
  void searchRepair(int iter, int size, int offset);
  int initTA_helper(int iter, int size, int offset, bool isH, int& numPanels);
};

class FlexTAWorker;
class FlexTAWorkerRegionQuery
{
 public:
  FlexTAWorkerRegionQuery(FlexTAWorker* in);
  ~FlexTAWorkerRegionQuery();
  frDesign* getDesign() const;
  FlexTAWorker* getTAWorker() const;

  void add(taPinFig* fig);
  void remove(taPinFig* fig);
  void query(const Rect& box,
             frLayerNum layerNum,
             frOrderedIdSet<taPin*>& result) const;

  void addCost(const Rect& box,
               frLayerNum layerNum,
               frBlockObject* obj,
               frConstraint* con);
  void removeCost(const Rect& box,
                  frLayerNum layerNum,
                  frBlockObject* obj,
                  frConstraint* con);
  void queryCost(
      const Rect& box,
      frLayerNum layerNum,
      std::vector<rq_box_value_t<std::pair<frBlockObject*, frConstraint*>>>&
          result) const;
  void addViaCost(const Rect& box,
                  frLayerNum layerNum,
                  frBlockObject* obj,
                  frConstraint* con);
  void removeViaCost(const Rect& box,
                     frLayerNum layerNum,
                     frBlockObject* obj,
                     frConstraint* con);
  void queryViaCost(
      const Rect& box,
      frLayerNum layerNum,
      std::vector<rq_box_value_t<std::pair<frBlockObject*, frConstraint*>>>&
          result) const;

  void init();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class FlexTAWorker
{
 public:
  // constructors
  FlexTAWorker(frDesign* designIn,
               Logger* logger,
               RouterConfiguration* router_cfg,
               bool save_updates)
      : design_(designIn),
        logger_(logger),
        router_cfg_(router_cfg),
        save_updates_(save_updates),
        dir_(dbTechLayerDir::NONE),
        taIter_(0),
        rq_(this),
        numAssigned_(0),
        totCost_(0),
        maxRetry_(1),
        hardIroutesMode_(false)
  {
  }
  // setters
  void setRouteBox(const Rect& boxIn) { routeBox_ = boxIn; }
  void setExtBox(const Rect& boxIn) { extBox_ = boxIn; }
  void setDir(const dbTechLayerDir& in) { dir_ = in; }
  void setTAIter(int in) { taIter_ = in; }
  void addIroute(std::unique_ptr<taPin> in, bool isExt = false)
  {
    in->setId(iroutes_.size() + extIroutes_.size());
    if (isExt) {
      extIroutes_.push_back(std::move(in));
    } else {
      iroutes_.push_back(std::move(in));
    }
  }
  void addToReassignIroutes(taPin* in) { reassignIroutes_.insert(in); }
  void removeFromReassignIroutes(taPin* in)
  {
    auto it = reassignIroutes_.find(in);
    if (it != reassignIroutes_.end()) {
      reassignIroutes_.erase(it);
    }
  }
  taPin* popFromReassignIroutes()
  {
    taPin* sol = nullptr;
    if (!reassignIroutes_.empty()) {
      sol = *reassignIroutes_.begin();
      reassignIroutes_.erase(reassignIroutes_.begin());
    }
    return sol;
  }

  // getters
  frTechObject* getTech() const { return design_->getTech(); }
  frDesign* getDesign() const { return design_; }
  const Rect& getRouteBox() const { return routeBox_; }
  const Rect& getExtBox() const { return extBox_; }
  dbTechLayerDir getDir() const { return dir_; }
  int getTAIter() const { return taIter_; }
  bool isInitTA() const { return (taIter_ == 0); }
  frRegionQuery* getRegionQuery() const { return design_->getRegionQuery(); }
  void getTrackIdx(frCoord loc1,
                   frCoord loc2,
                   frLayerNum lNum,
                   int& idx1,
                   int& idx2) const
  {
    idx1 = int(
        std::lower_bound(trackLocs_[lNum].begin(), trackLocs_[lNum].end(), loc1)
        - trackLocs_[lNum].begin());
    idx2 = int(std::upper_bound(
                   trackLocs_[lNum].begin(), trackLocs_[lNum].end(), loc2)
               - trackLocs_[lNum].begin())
           - 1;
  }
  const std::vector<frCoord>& getTrackLocs(frLayerNum in) const
  {
    return trackLocs_[in];
  }
  const FlexTAWorkerRegionQuery& getWorkerRegionQuery() const { return rq_; }
  FlexTAWorkerRegionQuery& getWorkerRegionQuery() { return rq_; }
  int getNumAssigned() const { return numAssigned_; }
  // others
  int main_mt();

 private:
  frDesign* design_;
  Logger* logger_;
  RouterConfiguration* router_cfg_;
  bool save_updates_;
  Rect routeBox_;
  Rect extBox_;
  dbTechLayerDir dir_;
  int taIter_;
  FlexTAWorkerRegionQuery rq_;

  std::vector<std::unique_ptr<taPin>> iroutes_;  // unsorted iroutes
  std::vector<std::unique_ptr<taPin>> extIroutes_;
  std::vector<std::vector<frCoord>> trackLocs_;
  std::set<taPin*, taPinComp>
      reassignIroutes_;  // iroutes to be assigned in sorted order
  int numAssigned_;
  int totCost_;
  int maxRetry_;
  bool hardIroutesMode_;

  //// others
  void init();
  void initFixedObjs();
  frCoord initFixedObjs_calcBloatDist(frBlockObject* obj,
                                      frLayerNum lNum,
                                      const Rect& box);
  frCoord initFixedObjs_calcOBSBloatDistVia(const frViaDef* viaDef,
                                            frLayerNum lNum,
                                            const Rect& box,
                                            bool isOBS = true);
  void initFixedObjs_helper(const Rect& box,
                            frCoord bloatDist,
                            frLayerNum lNum,
                            frNet* net,
                            bool isViaCost = false);
  void initTracks();
  void initIroutes();
  void initIroute(frGuide* guide);
  void initIroute_helper(frGuide* guide,
                         frCoord& maxBegin,
                         frCoord& minEnd,
                         std::set<frCoord>& downViaCoordSet,
                         std::set<frCoord>& upViaCoordSet,
                         int& nextIrouteDir,
                         frCoord& pinCoord);
  void initIroute_helper_generic(frGuide* guide,
                                 frCoord& minBegin,
                                 frCoord& maxEnd,
                                 std::set<frCoord>& downViaCoordSet,
                                 std::set<frCoord>& upViaCoordSet,
                                 int& nextIrouteDir,
                                 frCoord& pinCoord);
  void initIroute_helper_generic_helper(frGuide* guide, frCoord& pinCoord);
  bool initIroute_helper_pin(frGuide* guide,
                             frCoord& maxBegin,
                             frCoord& minEnd,
                             std::set<frCoord>& downViaCoordSet,
                             std::set<frCoord>& upViaCoordSet,
                             int& nextIrouteDir,
                             frCoord& pinCoord);
  void initCosts();
  void sortIroutes();
  bool outOfDieVia(frLayerNum layer_num,
                   const Point& pt,
                   const Rect& die_box) const;

  // quick drc
  frSquaredDistance box2boxDistSquare(const Rect& box1,
                                      const Rect& box2,
                                      frCoord& dx,
                                      frCoord& dy);
  void addCost(taPinFig* fig, frOrderedIdSet<taPin*>* pinS = nullptr);
  void subCost(taPinFig* fig, frOrderedIdSet<taPin*>* pinS = nullptr);
  void modCost(taPinFig* fig,
               bool isAddCost,
               frOrderedIdSet<taPin*>* pinS = nullptr);
  void modMinSpacingCostPlanar(const Rect& box,
                               frLayerNum lNum,
                               taPinFig* fig,
                               bool isAddCost,
                               frOrderedIdSet<taPin*>* pinS = nullptr);
  void modMinSpacingCostVia(const Rect& box,
                            frLayerNum lNum,
                            taPinFig* fig,
                            bool isAddCost,
                            bool isUpperVia,
                            bool isCurrPs,
                            frOrderedIdSet<taPin*>* pinS = nullptr);
  void modCutSpacingCost(const Rect& box,
                         frLayerNum lNum,
                         taPinFig* fig,
                         bool isAddCost,
                         frOrderedIdSet<taPin*>* pinS = nullptr);

  // initTA
  void assign();
  void assignIroute(taPin* iroute);
  void assignIroute_init(taPin* iroute, frOrderedIdSet<taPin*>* pinS);
  void assignIroute_availTracks(taPin* iroute,
                                frLayerNum& lNum,
                                int& idx1,
                                int& idx2);
  int assignIroute_bestTrack(taPin* iroute,
                             frLayerNum lNum,
                             int idx1,
                             int idx2);
  void assignIroute_bestTrack_helper(taPin* iroute,
                                     frLayerNum lNum,
                                     int trackIdx,
                                     frUInt4& bestCost,
                                     frCoord& bestTrackLoc,
                                     int& bestTrackIdx,
                                     frUInt4& drcCost);
  frUInt4 assignIroute_getCost(taPin* iroute,
                               frCoord trackLoc,
                               frUInt4& drcCost);
  frUInt4 assignIroute_getNextIrouteDirCost(taPin* iroute, frCoord trackLoc);
  frUInt4 assignIroute_getPinCost(taPin* iroute, frCoord trackLoc);
  frUInt4 assignIroute_getAlignCost(taPin* iroute, frCoord trackLoc);
  frUInt4 assignIroute_getDRCCost(taPin* iroute, frCoord trackLoc);
  frUInt4 assignIroute_getDRCCost_helper(taPin* iroute,
                                         Rect& box,
                                         frLayerNum lNum);
  void assignIroute_updateIroute(taPin* iroute,
                                 frCoord bestTrackLoc,
                                 frOrderedIdSet<taPin*>* pinS);
  void assignIroute_updateOthers(frOrderedIdSet<taPin*>& pinS);

  // end
  void end();
  void saveToGuides();

  friend class FlexTA;
};

}  // namespace drt
