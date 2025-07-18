#define BOOST_TEST_MODULE TestCallbacks
#include <boost/test/included/unit_test.hpp>
#include <iostream>

#include "CallBack.h"
#include "helper.h"
#include "odb/db.h"

namespace odb {
namespace {

BOOST_AUTO_TEST_SUITE(test_suite)
dbDatabase* db;
dbBlock* block;
CallBack* cb;
void setup()
{
  cb = new CallBack();
  cb->clearEvents();
}
void tearDown()
{
  dbDatabase::destroy(db);
  delete cb;
}
BOOST_AUTO_TEST_CASE(test_inst_and_iterm)
{
  setup();
  db = createSimpleDB();
  block = db->getChip()->getBlock();
  dbNet* n1 = dbNet::create(block, "n1");
  cb->addOwner(block);
  dbInst* i1 = dbInst::create(block, db->findMaster("and2"), "i1");
  BOOST_TEST(cb->events.size() == 4);
  BOOST_TEST(cb->events[0] == "Create inst i1");
  BOOST_TEST(cb->events[1] == "Create iterm a of inst i1");
  BOOST_TEST(cb->events[2] == "Create iterm b of inst i1");
  BOOST_TEST(cb->events[3] == "Create iterm o of inst i1");
  cb->clearEvents();
  cb->pause();
  dbRegion* parent_region = dbRegion::create(block, "parent");
  cb->unpause();
  dbInst::create(block, db->findMaster("and2"), "i2", parent_region);
  BOOST_TEST(cb->events.size() == 4);
  BOOST_TEST(cb->events[0] == "Create inst i2 in region parent");
  BOOST_TEST(cb->events[1] == "Create iterm a of inst i2");
  BOOST_TEST(cb->events[2] == "Create iterm b of inst i2");
  BOOST_TEST(cb->events[3] == "Create iterm o of inst i2");
  cb->clearEvents();
  i1->swapMaster(db->findMaster("or2"));
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreSwap inst i1 from master and2 to master or2");
  BOOST_TEST(cb->events[1] == "PostSwap inst i1 to master or2");
  cb->clearEvents();
  i1->setOrigin(100, 100);
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreMove inst i1");
  BOOST_TEST(cb->events[1] == "PostMove inst i1");
  cb->clearEvents();
  i1->setOrigin(100, 100);
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreMove inst i1");
  BOOST_TEST(cb->events[1] == "PostMove inst i1");
  cb->clearEvents();
  i1->findITerm("a")->connect(n1);
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreConnect iterm to net n1");
  BOOST_TEST(cb->events[1] == "PostConnect iterm to net n1");
  cb->clearEvents();
  i1->findITerm("a")->connect(n1);
  BOOST_TEST(cb->events.size() == 0);
  i1->findITerm("a")->disconnect();
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreDisconnect iterm from net n1");
  BOOST_TEST(cb->events[1] == "PostDisconnect iterm from net n1");
  cb->clearEvents();
  i1->findITerm("a")->disconnect();
  BOOST_TEST(cb->events.size() == 0);

  dbInst::destroy(i1);

  BOOST_TEST(cb->events.size() == 4);
  BOOST_TEST(cb->events[0] == "Destroy iterm o of inst i1");
  BOOST_TEST(cb->events[1] == "Destroy iterm b of inst i1");
  BOOST_TEST(cb->events[2] == "Destroy iterm a of inst i1");
  BOOST_TEST(cb->events[3] == "Destroy inst i1");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_net)
{
  setup();
  db = createSimpleDB();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbNet* n1 = dbNet::create(block, "n1");
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create net n1");
  cb->clearEvents();
  dbNet::destroy(n1);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy net n1");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_bterm)
{
  setup();
  db = create2LevetDbNoBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  auto n1 = block->findNet("n1");
  dbBTerm* IN1 = dbBTerm::create(n1, "IN1");
  IN1->setIoType(dbIoType::INPUT);
  BOOST_TEST(cb->events.size() == 3);
  BOOST_TEST(cb->events[0] == "Create IN1");
  BOOST_TEST(cb->events[1] == "Preconnect IN1 to n1");
  BOOST_TEST(cb->events[2] == "Postconnect IN1");
  cb->clearEvents();
  IN1->disconnect();
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "Predisconnect IN1");
  BOOST_TEST(cb->events[1] == "Postdisconnect IN1 from n1");
  cb->clearEvents();
  dbBTerm::destroy(IN1);

  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy IN1");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_bpin)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbBPin* pin = dbBPin::create(block->findBTerm("IN1"));
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create BPin for IN1");
  cb->clearEvents();
  dbBPin::destroy(pin);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy BPin");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_blockage)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbBlockage* blk = dbBlockage::create(block, 0, 0, 100, 100);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create blockage (0,0) (100,100)");
  cb->clearEvents();
  dbBlockage::destroy(blk);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy blockage");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_obstruction)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbObstruction* obs = dbObstruction::create(
      block, db->getTech()->findLayer("L1"), 0, 0, 100, 100);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create obstruction (0,0) (100,100)");
  cb->clearEvents();
  dbObstruction::destroy(obs);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy obstruction");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_region)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbRegion* parent_region = dbRegion::create(block, "parent");
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create region parent");
  cb->clearEvents();
  dbBox::create(parent_region, 0, 0, 1, 1);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Add box (0, 0) (1, 1) to region parent");
  cb->clearEvents();
  dbRegion::destroy(parent_region);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy region parent");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_row)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbSite* site = dbSite::create(db->findLib("lib1"), "site1");
  dbRow* row = dbRow::create(
      block, "row1", site, 0, 0, dbOrientType::MX, dbRowDir::HORIZONTAL, 1, 20);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create row row1");
  cb->clearEvents();
  dbRow::destroy(row);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy row row1");
  tearDown();
}
BOOST_AUTO_TEST_CASE(test_wire)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbWire* wire1 = dbWire::create(block->findNet("n1"));
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create wire");
  cb->clearEvents();
  wire1->attach(block->findNet("n1"));
  BOOST_TEST(cb->events.size() == 0);
  wire1->detach();
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreDetach wire");
  BOOST_TEST(cb->events[1] == "PostDetach wire from n1");
  cb->clearEvents();
  wire1->attach(block->findNet("n1"));
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreAttach wire to n1");
  BOOST_TEST(cb->events[1] == "PostAttach wire");
  cb->clearEvents();

  cb->pause();
  dbWire* wire2 = dbWire::create(block);
  cb->unpause();
  wire2->append(wire1);
  BOOST_TEST(cb->events.size() == 2);
  BOOST_TEST(cb->events[0] == "PreAppend wire");
  BOOST_TEST(cb->events[1] == "PostAppend wire");
  cb->clearEvents();

  dbWire::destroy(wire2);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Destroy wire");
}
BOOST_AUTO_TEST_CASE(test_swire)
{
  setup();
  db = create2LevetDbWithBTerms();
  block = db->getChip()->getBlock();
  cb->addOwner(block);
  dbSWire* wire = dbSWire::create(block->findNet("n1"), dbWireType::NOSHIELD);
  BOOST_TEST(cb->events.size() == 1);
  BOOST_TEST(cb->events[0] == "Create swire");
  cb->clearEvents();
  dbSBox::create(wire,
                 db->getTech()->findLayer("L1"),
                 0,
                 100,
                 100,
                 100,
                 dbWireShapeType::IOWIRE,
                 dbSBox::Direction::HORIZONTAL);

  dbSWire::destroy(wire);
  BOOST_TEST(cb->events.size() == 3);
  BOOST_TEST(cb->events[0] == "PreDestroySBoxes");
  BOOST_TEST(cb->events[1] == "PostDestroySBoxes");
  BOOST_TEST(cb->events[2] == "Destroy swire");
}
BOOST_AUTO_TEST_CASE(test_findInst_in_callback)
{
  class FindCallback : public dbBlockCallBackObj
  {
   public:
    void inDbITermDestroy(dbITerm* iterm) override
    {
      dbInst* inst = iterm->getInst();
      if (auto it = inst->findITerm("a")) {
        BOOST_TEST(it->getId() != 0);
      }
      if (auto it = inst->findITerm("b")) {
        BOOST_TEST(it->getId() != 0);
      }
      if (auto it = inst->findITerm("o")) {
        BOOST_TEST(it->getId() != 0);
      }
    }
  };

  db = createSimpleDB();
  block = db->getChip()->getBlock();
  FindCallback cb;
  cb.addOwner(block);
  dbLib* lib = db->findLib("lib1");
  auto and2 = lib->findMaster("and2");
  dbInst* i1 = dbInst::create(block, and2, "i1");
  dbInst::destroy(i1);
}
BOOST_AUTO_TEST_SUITE_END()

}  // namespace
}  // namespace odb
