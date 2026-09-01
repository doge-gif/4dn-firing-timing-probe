#include "core/fsm.hpp"
#include "doctest/doctest.h"

using namespace core;

TEST_CASE("fsm: boot -> config_load -> ready on valid config") {
  CHECK(step(State::Boot, Event::Init).next == State::ConfigLoad);
  CHECK(step(State::Boot, Event::Init).action == Action::LoadConfig);
  CHECK(step(State::ConfigLoad, Event::ConfigValid).next == State::Ready);
}

TEST_CASE("fsm: invalid config -> CONFIG_INVALID with 10Hz LED") {
  Transition t = step(State::ConfigLoad, Event::ConfigInvalid);
  CHECK(t.next == State::ConfigInvalid);
  CHECK(led_for(State::ConfigInvalid) == LedPattern::Blink10Hz);
}

TEST_CASE("fsm: start accepted only in READY") {
  Transition r = step(State::Ready, Event::StartPressed);
  CHECK(r.next == State::Running);
  CHECK(r.action == Action::BeginRun);
  // ignored elsewhere (stays put, no action)
  CHECK(step(State::ConfigInvalid, Event::StartPressed).next == State::ConfigInvalid);
  CHECK(step(State::ConfigInvalid, Event::StartPressed).action == Action::None);
  CHECK(step(State::Running, Event::StartPressed).next == State::Running);
  CHECK(step(State::Running, Event::StartPressed).action == Action::None);
}

TEST_CASE("fsm: run lifecycle -> flush -> done -> ready") {
  Transition rc = step(State::Running, Event::RunComplete);
  CHECK(rc.next == State::Flush);
  CHECK(rc.action == Action::WriteResults);
  CHECK(step(State::Flush, Event::FlushDone).next == State::Done);
  Transition dn = step(State::Done, Event::Tick);
  CHECK(dn.next == State::Ready);
  CHECK(dn.action == Action::ReturnReady);
}

TEST_CASE("fsm: lost sync is recoverable to ready") {
  CHECK(step(State::Running, Event::LostSync).next == State::Error);
  Transition er = step(State::Error, Event::Tick);
  CHECK(er.next == State::Ready);
  CHECK(er.action == Action::ReturnReady);
}

TEST_CASE("fsm: LED patterns per state") {
  CHECK(led_for(State::Ready) == LedPattern::Blink1Hz);
  CHECK(led_for(State::Running) == LedPattern::Heartbeat);
  CHECK(led_for(State::Boot) == LedPattern::Off);
}
