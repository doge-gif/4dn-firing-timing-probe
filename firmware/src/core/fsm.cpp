#include "core/fsm.hpp"

namespace core {

Transition step(State s, Event e) {
  switch (s) {
  case State::Boot:
    if (e == Event::Init)
      return {State::ConfigLoad, Action::LoadConfig};
    break;
  case State::ConfigLoad:
    if (e == Event::ConfigValid)
      return {State::Ready, Action::None};
    if (e == Event::ConfigInvalid)
      return {State::ConfigInvalid, Action::None};
    break;
  case State::ConfigInvalid:
    break; // ignore inputs; recover via power-cycle / FS eject re-validate
  case State::Ready:
    if (e == Event::StartPressed)
      return {State::Running, Action::BeginRun};
    break;
  case State::Running:
    if (e == Event::RunComplete)
      return {State::Flush, Action::WriteResults};
    if (e == Event::LostSync)
      return {State::Error, Action::None};
    break;
  case State::Flush:
    if (e == Event::FlushDone)
      return {State::Done, Action::None};
    break;
  // Done and Error are both transient; a Tick returns to Ready either way.
  case State::Done:
  case State::Error:
    if (e == Event::Tick)
      return {State::Ready, Action::ReturnReady};
    break;
  }
  return {s, Action::None}; // default: stay, no action
}

LedPattern led_for(State s) {
  switch (s) {
  case State::Boot:
  case State::ConfigLoad:
    return LedPattern::Off;
  case State::ConfigInvalid:
    return LedPattern::Blink10Hz;
  case State::Ready:
    return LedPattern::Blink1Hz;
  case State::Running:
    return LedPattern::Heartbeat;
  case State::Flush:
    return LedPattern::DoubleBlink;
  case State::Done:
    return LedPattern::SolidBriefly;
  case State::Error:
    return LedPattern::ErrorTriple;
  }
  return LedPattern::Off;
}

} // namespace core
