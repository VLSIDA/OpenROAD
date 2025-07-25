// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include <memory>
#include <utility>
#include <vector>

#include "frRTree.h"
#include "ta/FlexTA.h"

namespace drt {

struct FlexTAWorkerRegionQuery::Impl
{
  FlexTAWorker* taWorker;
  std::vector<RTree<taPinFig*>> shapes;  // resource map
  // fixed objs, owner:: nullptr or net, con = short
  std::vector<RTree<std::pair<frBlockObject*, frConstraint*>>> route_costs;
  std::vector<RTree<std::pair<frBlockObject*, frConstraint*>>> via_costs;
};

FlexTAWorkerRegionQuery::FlexTAWorkerRegionQuery(FlexTAWorker* in)
    : impl_(std::make_unique<Impl>())
{
  impl_->taWorker = in;
}

FlexTAWorkerRegionQuery::~FlexTAWorkerRegionQuery() = default;

FlexTAWorker* FlexTAWorkerRegionQuery::getTAWorker() const
{
  return impl_->taWorker;
}

frDesign* FlexTAWorkerRegionQuery::getDesign() const
{
  return impl_->taWorker->getDesign();
}

void FlexTAWorkerRegionQuery::add(taPinFig* fig)
{
  Rect box;
  if (fig->typeId() == tacPathSeg) {
    auto obj = static_cast<taPathSeg*>(fig);
    auto [bp, ep] = obj->getPoints();
    box = Rect(bp, ep);
    impl_->shapes.at(obj->getLayerNum()).insert(std::make_pair(box, obj));
  } else if (fig->typeId() == tacVia) {
    auto obj = static_cast<taVia*>(fig);
    auto bp = obj->getOrigin();
    box = Rect(bp, bp);
    impl_->shapes.at(obj->getViaDef()->getCutLayerNum())
        .insert(std::make_pair(box, obj));
  } else {
    std::cout << "Error: unsupported region query add" << std::endl;
  }
}

void FlexTAWorkerRegionQuery::remove(taPinFig* fig)
{
  Rect box;
  if (fig->typeId() == tacPathSeg) {
    auto obj = static_cast<taPathSeg*>(fig);
    auto [bp, ep] = obj->getPoints();
    box = Rect(bp, ep);
    impl_->shapes.at(obj->getLayerNum()).remove(std::make_pair(box, obj));
  } else if (fig->typeId() == tacVia) {
    auto obj = static_cast<taVia*>(fig);
    auto bp = obj->getOrigin();
    box = Rect(bp, bp);
    impl_->shapes.at(obj->getViaDef()->getCutLayerNum())
        .remove(std::make_pair(box, obj));
  } else {
    std::cout << "Error: unsupported region query add" << std::endl;
  }
}

void FlexTAWorkerRegionQuery::query(const Rect& box,
                                    const frLayerNum layerNum,
                                    frOrderedIdSet<taPin*>& result) const
{
  std::vector<rq_box_value_t<taPinFig*>> temp;
  auto& tree = impl_->shapes.at(layerNum);
  transform(tree.qbegin(bgi::intersects(box)),
            tree.qend(),
            inserter(result, result.end()),
            [](const auto& box_fig) { return box_fig.second->getPin(); });
}

void FlexTAWorkerRegionQuery::init()
{
  int numLayers = getDesign()->getTech()->getLayers().size();
  impl_->shapes.clear();
  impl_->shapes.resize(numLayers);
  impl_->route_costs.clear();
  impl_->route_costs.resize(numLayers);
  impl_->via_costs.clear();
  impl_->via_costs.resize(numLayers);
}

void FlexTAWorkerRegionQuery::addCost(const Rect& box,
                                      const frLayerNum layerNum,
                                      frBlockObject* obj,
                                      frConstraint* con)
{
  impl_->route_costs.at(layerNum).insert(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::removeCost(const Rect& box,
                                         const frLayerNum layerNum,
                                         frBlockObject* obj,
                                         frConstraint* con)
{
  impl_->route_costs.at(layerNum).remove(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::queryCost(
    const Rect& box,
    const frLayerNum layerNum,
    std::vector<rq_box_value_t<std::pair<frBlockObject*, frConstraint*>>>&
        result) const
{
  impl_->route_costs.at(layerNum).query(bgi::intersects(box),
                                        back_inserter(result));
}

void FlexTAWorkerRegionQuery::addViaCost(const Rect& box,
                                         const frLayerNum layerNum,
                                         frBlockObject* obj,
                                         frConstraint* con)
{
  impl_->via_costs.at(layerNum).insert(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::removeViaCost(const Rect& box,
                                            const frLayerNum layerNum,
                                            frBlockObject* obj,
                                            frConstraint* con)
{
  impl_->via_costs.at(layerNum).remove(
      std::make_pair(box, std::make_pair(obj, con)));
}

void FlexTAWorkerRegionQuery::queryViaCost(
    const Rect& box,
    const frLayerNum layerNum,
    std::vector<rq_box_value_t<std::pair<frBlockObject*, frConstraint*>>>&
        result) const
{
  impl_->via_costs.at(layerNum).query(bgi::intersects(box),
                                      back_inserter(result));
}

}  // namespace drt
