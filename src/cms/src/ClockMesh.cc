// Copyright (c) 2021, The Regents of the University of California
// All rights reserved.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "cms/ClockMesh.hh"
#include "sta/StaMain.hh"

extern "C" {
extern int Cms_Init(Tcl_Interp *interp);
}

namespace cms {

using utl::CMS;
using std::string;
using sta::LibertyLibrary;
using sta::LibertyLibrarySeq;
using sta::LibertyPort;
using odb::dbPlacementStatus;

// Tcl files encoded into strings.
extern const char *cms_tcl_inits[];

ClockMesh::ClockMesh()
{
  this->value_ = 0;
  this->buffer_ptr_ = 0;
  this->buffers_ = nullptr;
  this->dbu_ = db_->getTech()->getDbUnitsPerMicron();
}

ClockMesh::~ClockMesh()
{
  if (this->buffers_ != nullptr) {
    for (int i = 0; i < value_; i++) {
      network_->deleteInstance(buffers_[i]);
    }
    delete[] this->buffers_;
    this->buffers_ = nullptr;
    delete point_;
    this->point_ = nullptr;
  }
  
}

void
ClockMesh::init(Tcl_Interp* tcl_interp,
	   odb::dbDatabase* db,
     sta::dbNetwork* network,
     rsz::Resizer* resizer,
     utl::Logger* logger)
{
  db_ = db;
  logger_ = logger;
  network_ = network;
  resizer_ = resizer;
  // Define swig TCL commands.
  Cms_Init(tcl_interp);
  // Eval encoded cms TCL sources.
  sta::evalTclInit(tcl_interp, cms::cms_tcl_inits);
}

int
ClockMesh::dump_value()
{
  return this->value_;
  logger_->info(CMS, 189, "Dumping ClockMesh Value of {}",value_);
}

int
ClockMesh::set_value(int value)
{
  this->value_ = std::abs(value);
  logger_->info(CMS, 151, "Set ClockMesh Value to {}", value);
  return this->value_;
}

int
ClockMesh::createBufferArray(int amount)
{
  if (amount == 0) {
    logger_->error(CMS, 409, "Need to set CMS Buffer Amount to non zero");
    return 1;
  } else {
    this->buffers_ = new sta::Instance*[amount];
    this->point_ = new Point[amount];
    logger_->info(CMS, 125, "CMS Buffer and Point arrays initialized!");
    return 0;
  }
}

void
ClockMesh::addBuffer()
{
  resizer_->initBlock();
  this->point_[buffer_ptr_].setX(buffer_ptr_);
  this->point_[buffer_ptr_].setY(buffer_ptr_);
  const string buffer_name = makeUniqueInstName("clock_mesh_buffer",true);
  buffers_[buffer_ptr_] = network_->makeInstance(buffer_cells_[0],
                          buffer_name.c_str(),
                          nullptr);
  dbInst* db_inst = resizer_->getDbNetwork()->staToDb(buffers_[buffer_ptr_]);
  //set the location
  setLocation(db_inst, point_[buffer_ptr_]);
  //call legalizer later
  //incremenet area of the design
  resizer_->designAreaIncr(area(db_inst->getMaster()));

  logger_->info(CMS, 95, "CMS added buffer: {} at point X: {} Y: {}",buffer_name, point_[buffer_ptr_].getX(),point_[buffer_ptr_].getY());
  buffer_ptr_++;
}

void
ClockMesh::findBuffers()
{
  if (buffer_cells_.empty()) {
    sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      LibertyLibrary* lib = lib_iter->next();
      for (LibertyCell* buffer : *lib->buffers()) {
        if (!resizer_->dontUse(buffer) && isLinkCell(buffer)) {
          buffer_cells_.emplace_back(buffer);
        }
      }
    }
    delete lib_iter;
    if (buffer_cells_.empty()) {
      logger_->error(CMS, 124, "no buffers found.");
    } else {
      sort(buffer_cells_,
           [this](const LibertyCell* buffer1, const LibertyCell* buffer2) {
             return bufferDriveResistance(buffer1)
                    > bufferDriveResistance(buffer2);
           });
    }
  }
}

std::string
ClockMesh::makeUniqueInstName(const char* base_name, bool underscore)
{
  string inst_name;
  do {
    // sta::stringPrint can lead to string overflow and fatal
    if (underscore) {
      inst_name = fmt::format("{}_{}", base_name, unique_inst_index_++);
    } else {
      inst_name = fmt::format("{}{}", base_name, unique_inst_index_++);
    }
  } while (network_->findInstance(inst_name.c_str()));
  return inst_name;
}

float
ClockMesh::bufferDriveResistance(const LibertyCell* buffer) const
{
  LibertyPort *input, *output;
  buffer->bufferPorts(input, output);
  return output->driveResistance();
}


void
ClockMesh::createGrid()
{
  findBuffers();
  //add one buffer for now
  addBuffer();
}

void 
ClockMesh::makeGrid()
{
  debugPrint(logger_, utl::CMS, "Make", 1, "Build - begin");
  auto* block = db_->getChip()->getBlock();

}

bool
ClockMesh::isLinkCell(LibertyCell* cell) const
{
  return network_->findLibertyCell(cell->name()) == cell;
}

double
ClockMesh::area(dbMaster* master)
{
  if (!master->isCoreAutoPlaceable()) {
    return 0;
  }
  return dbuToMeters(master->getWidth()) * dbuToMeters(master->getHeight());
}

double
ClockMesh::dbuToMeters(int dist) const
{
  return dist / (dbu_ * 1e+6);
}

void
ClockMesh::setLocation(dbInst* db_inst, const Point& pt)
{
  int x = pt.x();
  int y = pt.y();
  //do proper placement later
  db_inst->setPlacementStatus(dbPlacementStatus::PLACED);
  db_inst->setLocation(x, y);
}


void ClockMesh::reset()
{
  core_domain_ = nullptr;
  domains_.clear();
  updateRenderer();
}

void ClockMesh::resetShapes()
{
  for (auto* grid : getGrids()) {
    grid->resetShapes();
  }
  updateRenderer();
}

void ClockMesh::buildGrids(bool trim)
{
  debugPrint(logger_, utl::CMS, "Make", 1, "Build - begin");
  auto* block = db_->getChip()->getBlock();

  resetShapes();

  const std::vector<Grid*> grids = getGrids();

  // connect instances already assigned to grids
  std::set<odb::dbInst*> insts_in_grids;
  for (auto* grid : grids) {
    auto insts_in_grid = grid->getInstances();
    insts_in_grids.insert(insts_in_grid.begin(), insts_in_grid.end());
  }

  ShapeVectorMap block_obs_vec;
  Grid::makeInitialObstructions(block, block_obs_vec, insts_in_grids, logger_);
  for (auto* grid : grids) {
    grid->getGridLevelObstructions(block_obs_vec);
  }
  ShapeVectorMap all_shapes_vec;

  // get special shapes
  Grid::makeInitialShapes(block, all_shapes_vec, logger_);
  for (const auto& [layer, layer_shapes] : all_shapes_vec) {
    auto& layer_obs = block_obs_vec[layer];
    for (const auto& shape : layer_shapes) {
      layer_obs.push_back(shape);
    }
  }

  Shape::ObstructionTreeMap block_obs;
  for (const auto& [layer, shapes] : block_obs_vec) {
    block_obs[layer] = Shape::ObstructionTree(shapes.begin(), shapes.end());
  }
  block_obs_vec.clear();

  Shape::ShapeTreeMap all_shapes;
  for (const auto& [layer, shapes] : all_shapes_vec) {
    all_shapes[layer] = Shape::ShapeTree(shapes.begin(), shapes.end());
  }
  all_shapes_vec.clear();

  for (auto* grid : grids) {
    debugPrint(
        logger_, utl::CMS, "Make", 2, "Build start grid - {}", grid->getName());
    grid->makeShapes(all_shapes, block_obs);
    for (const auto& [layer, shapes] : grid->getShapes()) {
      auto& all_shapes_layer = all_shapes[layer];
      for (auto& shape : shapes) {
        all_shapes_layer.insert(shape);
      }
    }
    grid->getObstructions(block_obs);
    debugPrint(
        logger_, utl::CMS, "Make", 2, "Build end grid - {}", grid->getName());
  }

  updateVias();

  if (trim) {
    trimShapes();

    cleanupVias();
  }

  bool failed = false;
  for (auto* grid : grids) {
    if (grid->hasShapes() || grid->hasVias()) {
      continue;
    }
    logger_->warn(utl::CMS,
                  232,
                  "{} does not contain any shapes or vias.",
                  grid->getLongName());
    failed = true;
  }
  if (failed) {
    logger_->error(utl::CMS, 233, "Failed to generate full power grid.");
  }

  updateRenderer();
  debugPrint(logger_, utl::CMS, "Make", 1, "Build - end");
}

void ClockMesh::cleanupVias()
{
  debugPrint(logger_, utl::CMS, "Make", 2, "Cleanup vias - begin");
  for (auto* grid : getGrids()) {
    grid->removeInvalidVias();
  }
  updateVias();
  debugPrint(logger_, utl::CMS, "Make", 2, "Cleanup vias - end");
}

void ClockMesh::updateVias()
{
  const auto grids = getGrids();

  for (auto* grid : grids) {
    for (const auto& [layer, shapes] : grid->getShapes()) {
      for (const auto& shape : shapes) {
        shape->clearVias();
      }
    }

    std::vector<ViaPtr> all_vias;
    grid->getVias(all_vias);

    for (const auto& via : all_vias) {
      via->getLowerShape()->addVia(via);
      via->getUpperShape()->addVia(via);
    }
  }
}

void ClockMesh::trimShapes()
{
  debugPrint(logger_, utl::CMS, "Make", 2, "Trim shapes - start");
  auto grids = getGrids();

  for (auto* grid : grids) {
    if (grid->type() == Grid::Existing) {
      // fixed shapes, so nothing to do
      continue;
    }
    const auto& pin_layers = grid->getPinLayers();
    for (const auto& [layer, shapes] : grid->getShapes()) {
      for (const auto& shape : shapes) {
        if (!shape->isModifiable()) {
          continue;
        }

        // if pin layer, do not modify the shapes, but allow them to be
        // removed if they are not connected to anything
        const bool is_pin_layer
            = pin_layers.find(shape->getLayer()) != pin_layers.end();

        Shape* new_shape = nullptr;
        const odb::Rect new_rect = shape->getMinimumRect();
        if (new_rect == shape->getRect()) {  // no change to shape
          continue;
        }

        // check if vias and shape form a stack without any other connections
        bool effectively_vias_stack = true;
        for (const auto& via : shape->getVias()) {
          if (via->getArea() != new_rect) {
            effectively_vias_stack = false;
            break;
          }
        }
        if (!effectively_vias_stack) {
          new_shape = shape->copy();
          new_shape->setRect(new_rect);
        }

        auto* component = shape->getGridComponent();
        if (new_shape == nullptr) {
          if (shape->isRemovable(is_pin_layer)) {
            component->removeShape(shape.get());
          }
        } else {
          if (!is_pin_layer) {
            component->replaceShape(shape.get(), {new_shape});
          }
        }
      }
    }
  }
  debugPrint(logger_, utl::CMS, "Make", 2, "Trim shapes - end");
}

VoltageDomain* ClockMesh::getCoreDomain() const
{
  return core_domain_.get();
}

void ClockMesh::ensureCoreDomain()
{
  if (core_domain_ == nullptr) {
    setCoreDomain(nullptr, nullptr, nullptr, {});
  }
}

std::vector<VoltageDomain*> ClockMesh::getDomains() const
{
  std::vector<VoltageDomain*> domains;
  if (core_domain_ != nullptr) {
    domains.push_back(core_domain_.get());
  }

  for (const auto& domain : domains_) {
    domains.push_back(domain.get());
  }
  return domains;
}

VoltageDomain* ClockMesh::findDomain(const std::string& name)
{
  ensureCoreDomain();
  auto domains = getDomains();
  if (name.empty()) {
    return domains.back();
  }

  for (const auto& domain : domains) {
    if (domain->getName() == name) {
      return domain;
    }
  }

  return nullptr;
}

void ClockMesh::setCoreDomain(odb::dbNet* power,
                           odb::dbNet* switched_power,
                           odb::dbNet* ground,
                           const std::vector<odb::dbNet*>& secondary)
{
  auto* block = db_->getChip()->getBlock();
  if (core_domain_ != nullptr) {
    logger_->warn(utl::CMS, 183, "Replacing existing core voltage domain.");
  }
  core_domain_ = std::make_unique<VoltageDomain>(
      this, block, power, ground, secondary, logger_);

  if (importUPF(core_domain_.get())) {
    if (switched_power) {
      logger_->error(
          utl::CMS, 210, "Cannot specify switched power net when using UPF.");
    }
  } else {
    core_domain_->setSwitchedPower(switched_power);
  }
}

void ClockMesh::makeRegionVoltageDomain(
    const std::string& name,
    odb::dbNet* power,
    odb::dbNet* switched_power,
    odb::dbNet* ground,
    const std::vector<odb::dbNet*>& secondary_nets,
    odb::dbRegion* region)
{
  if (region == nullptr) {
    logger_->error(utl::CMS, 229, "Region must be specified.");
  }
  for (const auto& domain : domains_) {
    if (domain->getName() == name) {
      logger_->error(utl::CMS,
                     184,
                     "Cannot have region voltage domain with the same name "
                     "already exists: {}",
                     name);
    }
  }
  auto* block = db_->getChip()->getBlock();
  auto domain = std::make_unique<VoltageDomain>(
      this, name, block, power, ground, secondary_nets, region, logger_);

  if (importUPF(domain.get())) {
    if (switched_power) {
      logger_->error(
          utl::CMS, 199, "Cannot specify switched power net when using UPF.");
    }
  } else {
    domain->setSwitchedPower(switched_power);
  }

  domains_.push_back(std::move(domain));
}

PowerCell* ClockMesh::findSwitchedPowerCell(const std::string& name) const
{
  for (const auto& cell : switched_power_cells_) {
    if (cell->getName() == name) {
      return cell.get();
    }
  }
  return nullptr;
}

void ClockMesh::makeSwitchedPowerCell(odb::dbMaster* master,
                                   odb::dbMTerm* control,
                                   odb::dbMTerm* acknowledge,
                                   odb::dbMTerm* switched_power,
                                   odb::dbMTerm* alwayson_power,
                                   odb::dbMTerm* ground)
{
  auto* check = findSwitchedPowerCell(master->getName());
  if (check != nullptr) {
    logger_->error(utl::CMS, 196, "{} is already defined.", master->getName());
  }

  switched_power_cells_.push_back(std::make_unique<PowerCell>(logger_,
                                                              master,
                                                              control,
                                                              acknowledge,
                                                              switched_power,
                                                              alwayson_power,
                                                              ground));
}

std::vector<Grid*> ClockMesh::getGrids() const
{
  std::vector<Grid*> grids;
  if (core_domain_ != nullptr) {
    for (const auto& grid : core_domain_->getGrids()) {
      grids.push_back(grid.get());
    }
  }
  for (const auto& domain : domains_) {
    for (const auto& grid : domain->getGrids()) {
      grids.push_back(grid.get());
    }
  }

  return grids;
}

std::vector<Grid*> ClockMesh::findGrid(const std::string& name) const
{
  std::vector<Grid*> found_grids;
  auto grids = getGrids();

  if (name.empty()) {
    if (grids.empty()) {
      return {};
    }

    return findGrid(grids.back()->getName());
  }

  for (auto* grid : grids) {
    if (grid->getName() == name || grid->getLongName() == name) {
      found_grids.push_back(grid);
    }
  }

  return found_grids;
}

void ClockMesh::makeCoreGrid(
    VoltageDomain* domain,
    const std::string& name,
    StartsWith starts_with,
    const std::vector<odb::dbTechLayer*>& pin_layers,
    const std::vector<odb::dbTechLayer*>& generate_obstructions,
    PowerCell* powercell,
    odb::dbNet* powercontrol,
    const char* powercontrolnetwork)
{
  auto grid = std::make_unique<CoreGrid>(
      domain, name, starts_with == POWER, generate_obstructions);
  grid->setPinLayers(pin_layers);

  PowerSwitchNetworkType control_network = PowerSwitchNetworkType::DAISY;
  if (strlen(powercontrolnetwork) > 0) {
    control_network
        = GridSwitchedPower::fromString(powercontrolnetwork, logger_);
  }
  if (importUPF(grid.get(), control_network)) {
    if (powercell != nullptr) {
      logger_->error(
          utl::CMS, 201, "Cannot specify power switch when UPF is available.");
    }
    if (powercontrol != nullptr) {
      logger_->error(
          utl::CMS, 202, "Cannot specify power control when UPF is available.");
    }
  } else {
    if (powercell != nullptr) {
      grid->setSwitchedPower(new GridSwitchedPower(
          grid.get(),
          powercell,
          powercontrol,
          GridSwitchedPower::fromString(powercontrolnetwork, logger_)));
    }
  }
  domain->addGrid(std::move(grid));
}

Grid* ClockMesh::instanceGrid(odb::dbInst* inst) const
{
  for (auto* check_grid : getGrids()) {
    auto* other_grid = dynamic_cast<InstanceGrid*>(check_grid);
    if (other_grid != nullptr) {
      if (other_grid->getInstance() == inst) {
        return check_grid;
      }
    }
  }

  return nullptr;
}

void ClockMesh::makeInstanceGrid(
    VoltageDomain* domain,
    const std::string& name,
    StartsWith starts_with,
    odb::dbInst* inst,
    const std::array<int, 4>& halo,
    bool pg_pins_to_boundary,
    bool default_grid,
    const std::vector<odb::dbTechLayer*>& generate_obstructions,
    bool is_bump)
{
  auto* check_grid = instanceGrid(inst);
  if (check_grid != nullptr) {
    if (check_grid->isReplaceable()) {
      // remove the old grid and replace with this one
      debugPrint(logger_,
                 utl::CMS,
                 "Setup",
                 1,
                 "Replacing {} with {} for instance: {}",
                 check_grid->getName(),
                 name,
                 inst->getName());
      auto* check_domain = check_grid->getDomain();
      check_domain->removeGrid(check_grid);
    } else if (default_grid) {
      // this is a default grid so we can ignore this assignment
      return;
    } else {
      logger_->warn(utl::CMS,
                    182,
                    "Instance {} already belongs to another grid \"{}\" and "
                    "therefore cannot belong to \"{}\".",
                    inst->getName(),
                    check_grid->getName(),
                    name);
      return;
    }
  }

  std::unique_ptr<InstanceGrid> grid = nullptr;
  if (is_bump) {
    grid = std::make_unique<BumpGrid>(domain, name, inst);
  } else {
    grid = std::make_unique<InstanceGrid>(
        domain, name, starts_with == POWER, inst, generate_obstructions);
  }
  if (!std::all_of(halo.begin(), halo.end(), [](int v) { return v == 0; })) {
    grid->addHalo(halo);
  }
  grid->setGridToBoundary(pg_pins_to_boundary);

  grid->setReplaceable(default_grid);

  if (!grid->isValid()) {
    return;
  }

  domain->addGrid(std::move(grid));
}

void ClockMesh::makeExistingGrid(
    const std::string& name,
    const std::vector<odb::dbTechLayer*>& generate_obstructions)
{
  auto grid = std::make_unique<ExistingGrid>(
      this, db_->getChip()->getBlock(), logger_, name, generate_obstructions);

  ensureCoreDomain();
  getCoreDomain()->addGrid(std::move(grid));
}

void ClockMesh::makeRing(Grid* grid,
                      odb::dbTechLayer* layer0,
                      int width0,
                      int spacing0,
                      odb::dbTechLayer* layer1,
                      int width1,
                      int spacing1,
                      StartsWith starts_with,
                      const std::array<int, 4>& offset,
                      const std::array<int, 4>& pad_offset,
                      bool extend,
                      const std::vector<odb::dbTechLayer*>& pad_pin_layers,
                      const std::vector<odb::dbNet*>& nets,
                      bool allow_out_of_die)
{
  std::array<Rings::Layer, 2> layers{Rings::Layer{layer0, width0, spacing0},
                                     Rings::Layer{layer1, width1, spacing1}};
  auto ring = std::make_unique<Rings>(grid, layers);
  ring->setOffset(offset);
  if (std::any_of(
          pad_offset.begin(), pad_offset.end(), [](int o) { return o != 0; })) {
    ring->setPadOffset(pad_offset);
  }
  ring->setExtendToBoundary(extend);
  if (starts_with != GRID) {
    ring->setStartWithPower(starts_with == POWER);
  }
  if (allow_out_of_die) {
    ring->setAllowOutsideDieArea();
  }
  ring->setNets(nets);
  grid->addRing(std::move(ring));
  if (!pad_pin_layers.empty() && grid->type() == Grid::Core) {
    auto* core_grid = static_cast<CoreGrid*>(grid);
    core_grid->setupDirectConnect(pad_pin_layers);
  }
}

void ClockMesh::makeFollowpin(Grid* grid,
                           odb::dbTechLayer* layer,
                           int width,
                           ExtensionMode extend)
{
  auto strap = std::make_unique<FollowPins>(grid, layer, width);
  strap->setExtend(extend);

  grid->addStrap(std::move(strap));
}

void ClockMesh::makeStrap(Grid* grid,
                       odb::dbTechLayer* layer,
                       int width,
                       int spacing,
                       int pitch,
                       int offset,
                       int number_of_straps,
                       bool snap,
                       StartsWith starts_with,
                       ExtensionMode extend,
                       const std::vector<odb::dbNet*>& nets)
{
  auto strap = std::make_unique<Straps>(
      grid, layer, width, pitch, spacing, number_of_straps);
  strap->setExtend(extend);
  strap->setOffset(offset);
  strap->setSnapToGrid(snap);
  if (starts_with != GRID) {
    strap->setStartWithPower(starts_with == POWER);
  }
  strap->setNets(nets);
  grid->addStrap(std::move(strap));
}

void ClockMesh::makeConnect(Grid* grid,
                         odb::dbTechLayer* layer0,
                         odb::dbTechLayer* layer1,
                         int cut_pitch_x,
                         int cut_pitch_y,
                         const std::vector<odb::dbTechViaGenerateRule*>& vias,
                         const std::vector<odb::dbTechVia*>& techvias,
                         int max_rows,
                         int max_columns,
                         const std::vector<odb::dbTechLayer*>& ongrid,
                         const std::map<odb::dbTechLayer*, int>& split_cuts,
                         const std::string& dont_use_vias)
{
  auto con = std::make_unique<Connect>(grid, layer0, layer1);
  con->setCutPitch(cut_pitch_x, cut_pitch_y);

  for (auto* via : vias) {
    con->addFixedVia(via);
  }

  for (auto* via : techvias) {
    con->addFixedVia(via);
  }

  con->setMaxRows(max_rows);
  con->setMaxColumns(max_columns);
  con->setOnGrid(ongrid);
  con->setSplitCuts(split_cuts);

  if (!dont_use_vias.empty()) {
    con->filterVias(dont_use_vias);
  }

  grid->addConnect(std::move(con));
}

void ClockMesh::setDebugRenderer(bool on)
{
  if (on && gui::Gui::enabled()) {
    if (debug_renderer_ == nullptr) {
      debug_renderer_ = std::make_unique<CMSRenderer>(this);
      rendererRedraw();
    }
  } else {
    debug_renderer_ = nullptr;
  }
}

void ClockMesh::rendererRedraw()
{
  if (debug_renderer_ != nullptr) {
    try {
      buildGrids(false);
    } catch (const std::runtime_error& /* e */) {
      // do nothing, dont want grid error to prevent debug renderer
      debug_renderer_->update();
    }
  }
}

void ClockMesh::updateRenderer() const
{
  if (debug_renderer_ != nullptr) {
    debug_renderer_->update();
  }
}

void ClockMesh::createSrouteWires(
    const char* net,
    const char* outerNet,
    odb::dbTechLayer* layer0,
    odb::dbTechLayer* layer1,
    int cut_pitch_x,
    int cut_pitch_y,
    const std::vector<odb::dbTechViaGenerateRule*>& vias,
    const std::vector<odb::dbTechVia*>& techvias,
    int max_rows,
    int max_columns,
    const std::vector<odb::dbTechLayer*>& ongrid,
    std::vector<int> metalwidths,
    std::vector<int> metalspaces,
    const std::vector<odb::dbInst*>& insts)
{
  sroute_->createSrouteWires(net,
                             outerNet,
                             layer0,
                             layer1,
                             cut_pitch_x,
                             cut_pitch_y,
                             vias,
                             techvias,
                             max_rows,
                             max_columns,
                             ongrid,
                             metalwidths,
                             metalspaces,
                             insts);
}

void ClockMesh::writeToDb(bool add_pins, const std::string& report_file) const
{
  std::map<odb::dbNet*, odb::dbSWire*> net_map;

  auto domains = getDomains();
  for (auto* domain : domains) {
    for (auto* net : domain->getNets()) {
      net_map[net] = nullptr;
    }
  }

  for (auto& [net, swire] : net_map) {
    net->setSpecial();

    // determine if unique and set WildConnected
    bool appear_in_all_grids = true;
    for (auto* domain : domains) {
      for (const auto& grid : domain->getGrids()) {
        const auto nets = grid->getNets();
        if (std::find(nets.begin(), nets.end(), net) == nets.end()) {
          appear_in_all_grids = false;
        }
      }
    }

    if (appear_in_all_grids) {
      // should this be based on the global connect?
      net->setWildConnected();
    }

    swire = odb::dbSWire::create(net, odb::dbWireType::ROUTED);
  }

  // collect all the SWires from the block
  auto* block = db_->getChip()->getBlock();
  ShapeVectorMap net_shapes_vec;
  for (auto* net : block->getNets()) {
    Shape::populateMapFromDb(net, net_shapes_vec);
  }
  const Shape::ObstructionTreeMap obstructions(net_shapes_vec.begin(),
                                               net_shapes_vec.end());
  net_shapes_vec.clear();

  // Remove existing non-fixed bpins
  for (auto& [net, swire] : net_map) {
    for (auto* bterm : net->getBTerms()) {
      auto bpins = bterm->getBPins();
      std::set<odb::dbBPin*> pins(bpins.begin(), bpins.end());
      for (auto* bpin : pins) {
        if (!bpin->getPlacementStatus().isFixed()) {
          odb::dbBPin::destroy(bpin);
        }
      }
    }
  }

  for (auto* domain : domains) {
    for (const auto& grid : domain->getGrids()) {
      grid->writeToDb(net_map, add_pins, obstructions);
      grid->makeRoutingObstructions(db_->getChip()->getBlock());
    }
  }

  // remove stale results
  odb::dbMarkerCategory* category = block->findMarkerCategory("CMS");
  if (category != nullptr) {
    odb::dbMarkerCategory::destroy(category);
  }

  for (auto* grid : getGrids()) {
    for (const auto& connect : grid->getConnect()) {
      connect->recordFailedVias();
    }
  }

  if (!report_file.empty()) {
    odb::dbMarkerCategory* category = block->findMarkerCategory("CMS");
    if (category != nullptr) {
      category->writeTR(report_file);
    }
  }
}

void ClockMesh::ripUp(odb::dbNet* net)
{
  if (net == nullptr) {
    resetShapes();
    std::set<odb::dbNet*> nets;
    ensureCoreDomain();
    for (auto* domain : getDomains()) {
      for (auto* net : domain->getNets()) {
        nets.insert(net);
      }
    }

    for (auto* grid : getGrids()) {
      grid->ripup();
    }

    for (odb::dbNet* net : nets) {
      ripUp(net);
    }

    return;
  }

  ShapeVectorMap net_shapes_vec;
  Shape::populateMapFromDb(net, net_shapes_vec);
  Shape::ShapeTreeMap net_shapes = Shape::convertVectorToTree(net_shapes_vec);

  // remove bterms that connect to swires
  std::set<odb::dbBTerm*> terms;
  for (auto* bterm : net->getBTerms()) {
    std::set<odb::dbBPin*> pins;
    for (auto* pin : bterm->getBPins()) {
      bool remove = false;
      for (auto* box : pin->getBoxes()) {
        auto* layer = box->getTechLayer();
        if (layer == nullptr) {
          continue;
        }
        if (net_shapes.count(layer) == 0) {
          continue;
        }

        odb::Rect rect = box->getBox();
        const auto& shapes = net_shapes[layer];
        if (shapes.qbegin(bgi::intersects(rect)) != shapes.qend()) {
          remove = true;
          break;
        }
      }
      if (remove) {
        pins.insert(pin);
      }
    }
    for (auto* pin : pins) {
      odb::dbBPin::destroy(pin);
    }
    if (bterm->getBPins().empty()) {
      terms.insert(bterm);
    }
  }
  for (auto* term : terms) {
    odb::dbBTerm::destroy(term);
  }
  auto swires = net->getSWires();
  for (auto iter = swires.begin(); iter != swires.end();) {
    iter = odb::dbSWire::destroy(iter);
  }
}

void ClockMesh::report()
{
  for (const auto& cell : switched_power_cells_) {
    cell->report();
  }
  ensureCoreDomain();
  for (auto* domain : getDomains()) {
    domain->report();
  }
}

void ClockMesh::setAllowRepairChannels(bool allow)
{
  for (auto* grid : getGrids()) {
    grid->setAllowRepairChannels(allow);
  }
}

void ClockMesh::filterVias(const std::string& filter)
{
  for (auto* grid : getGrids()) {
    for (const auto& connect : grid->getConnect()) {
      connect->filterVias(filter);
    }
  }
}

void ClockMesh::checkDesign(odb::dbBlock* block) const
{
  for (auto* inst : block->getInsts()) {
    if (!inst->getPlacementStatus().isFixed()) {
      continue;
    }
    for (auto* term : inst->getITerms()) {
      if (term->getSigType().isSupply() && term->getNet() == nullptr) {
        logger_->warn(utl::CMS,
                      189,
                      "Supply pin {} is not connected to any net.",
                      term->getName());
      }
    }
  }

  bool unplaced_macros = false;
  for (auto* inst : block->getInsts()) {
    if (!inst->isBlock()) {
      continue;
    }
    if (!inst->getPlacementStatus().isFixed()) {
      unplaced_macros = true;
      logger_->warn(
          utl::CMS, 234, "{} has not been placed and fixed.", inst->getName());
    }
  }
  if (unplaced_macros) {
    logger_->error(utl::CMS, 235, "Design has unplaced macros.");
  }
}

void ClockMesh::checkSetup() const
{
  auto* block = db_->getChip()->getBlock();

  checkDesign(block);

  for (auto* domain : getDomains()) {
    domain->checkSetup();
  }
}

void ClockMesh::repairVias(const std::set<odb::dbNet*>& nets)
{
  ViaRepair repair(logger_, nets);
  repair.repair();
  repair.report();
}

bool ClockMesh::importUPF(VoltageDomain* domain)
{
  auto* block = db_->getChip()->getBlock();

  bool has_upf = false;
  odb::dbPowerDomain* power_domain = nullptr;
  for (auto* upf_domain : block->getPowerDomains()) {
    has_upf = true;

    if (domain == core_domain_.get()) {
      if (upf_domain->isTop()) {
        power_domain = upf_domain;
        break;
      }
    } else {
      odb::dbGroup* upf_group = upf_domain->getGroup();

      if (upf_group != nullptr
          && upf_group->getRegion() == domain->getRegion()) {
        power_domain = upf_domain;
        break;
      }
    }
  }

  if (power_domain != nullptr) {
    const auto power_switches = power_domain->getPowerSwitches();
    if (power_switches.size() > 1) {
      logger_->error(
          utl::CMS,
          203,
          "Unable to process power domain with more than 1 power switch");
    }

    for (auto* pswitch : power_switches) {
      auto port_map = pswitch->getPortMap();
      if (port_map.empty()) {
        logger_->error(
            utl::CMS,
            204,
            "Unable to process power switch, {}, without port mapping",
            pswitch->getName());
      }

      odb::dbMaster* master = pswitch->getLibCell();
      odb::dbMTerm* control = nullptr;
      odb::dbMTerm* acknowledge = nullptr;
      odb::dbMTerm* switched_power = nullptr;
      odb::dbMTerm* alwayson_power = nullptr;
      odb::dbMTerm* ground = nullptr;

      const auto control_port = pswitch->getControlPorts();
      const auto input_supply = pswitch->getInputSupplyPorts();
      const auto output_supply = pswitch->getOutputSupplyPort();
      const auto ack_port = pswitch->getAcknowledgePorts();

      for (auto* mterm : master->getMTerms()) {
        if (mterm->getSigType() == odb::dbSigType::GROUND) {
          ground = mterm;
        }
      }

      control = port_map[control_port[0].port_name];
      alwayson_power = port_map[input_supply[0].port_name];
      switched_power = port_map[output_supply.port_name];
      acknowledge = port_map[ack_port[0].port_name];

      if (control == nullptr) {
        logger_->error(utl::CMS,
                       205,
                       "Unable to determine control port for: {}",
                       master->getName());
      }

      if (alwayson_power == nullptr) {
        logger_->error(utl::CMS,
                       206,
                       "Unable to determine always on power port for: {}",
                       master->getName());
      }

      if (switched_power == nullptr) {
        logger_->error(utl::CMS,
                       207,
                       "Unable to determine switched power port for: {}",
                       master->getName());
      }

      if (ground == nullptr) {
        logger_->error(utl::CMS,
                       208,
                       "Unable to determine ground port for: {}",
                       master->getName());
      }

      auto* switched_net
          = block->findNet(output_supply.supply_net_name.c_str());
      if (switched_net == nullptr) {
        logger_->error(utl::CMS, 238, "Unable to determine switched power net");
      }
      domain->setSwitchedPower(switched_net);

      if (findSwitchedPowerCell(master->getName())) {
        logger_->warn(utl::CMS,
                      209,
                      "Power switch for {} already exists",
                      pswitch->getName());
      } else {
        makeSwitchedPowerCell(master,
                              control,
                              acknowledge,
                              switched_power,
                              alwayson_power,
                              ground);
      }
    }
  }

  return has_upf;
}

bool ClockMesh::importUPF(Grid* grid, PowerSwitchNetworkType type) const
{
  auto* block = db_->getChip()->getBlock();

  auto* domain = grid->getDomain();

  bool has_upf = false;
  odb::dbPowerDomain* power_domain = nullptr;
  for (auto* upf_domain : block->getPowerDomains()) {
    has_upf = true;

    if (domain == core_domain_.get()) {
      if (upf_domain->isTop()) {
        power_domain = upf_domain;
        break;
      }
    } else {
      odb::dbGroup* upf_group = upf_domain->getGroup();

      if (upf_group != nullptr
          && upf_group->getRegion() == domain->getRegion()) {
        power_domain = upf_domain;
        break;
      }
    }
  }

  if (power_domain != nullptr) {
    auto power_switches = power_domain->getPowerSwitches();
    if (!power_switches.empty()) {
      auto pswitch = power_switches[0];

      auto* cms_switch
          = findSwitchedPowerCell(pswitch->getLibCell()->getName());

      const auto& control_ports = pswitch->getControlPorts();
      const auto& control_net = control_ports[0].net_name;
      if (control_net.empty()) {
        logger_->error(
            utl::CMS,
            236,
            "Cannot handle undefined control net for power switch: {}",
            pswitch->getName());
      }
      auto control = block->findNet(control_net.c_str());
      if (control == nullptr) {
        logger_->error(
            utl::CMS, 237, "Unable to find control net: {}", control_net);
      }

      grid->setSwitchedPower(
          new GridSwitchedPower(grid, cms_switch, control, type));
    }
  }

  return has_upf;
}
} // namespace cms