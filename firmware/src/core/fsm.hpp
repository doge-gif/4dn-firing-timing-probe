#pragma once
#include <cstdint>

namespace core {

enum class State : std::uint8_t {
  Boot,
  ConfigLoad,
  ConfigInvalid,
  Ready,
  Running,
  Flush,
  Done,
  Error
};
enum class Event : std::uint8_t {
  Init,
  ConfigValid,
  ConfigInvalid,
  StartPressed,
  RunComplete,
  FlushDone,
  LostSync,
  Tick
};
enum class Action : std::uint8_t { None, LoadConfig, BeginRun, WriteResults, ReturnReady };
enum class LedPattern : std::uint8_t {
  Off,
  Blink1Hz,
  Blink10Hz,
  Heartbeat,
  DoubleBlink,
  SolidBriefly,
  ErrorTriple
};

struct Transition {
  State next;
  Action action;
};

Transition step(State s, Event e);
LedPattern led_for(State s);

} // namespace core
