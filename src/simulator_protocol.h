#ifndef SIMULATOR_PROTOCOL_H
#define SIMULATOR_PROTOCOL_H

#include <string>

#include "planner_types.h"

enum class SimulatorMessageKind {
  kIgnore,
  kManual,
  kTelemetry,
};

struct SimulatorMessage {
  SimulatorMessageKind kind = SimulatorMessageKind::kIgnore;
  PlannerInput input;
  std::string error;
};

SimulatorMessage ParseSimulatorMessage(const std::string &data);
std::string MakeControlMessage(const PlannerOutput &output);
std::string MakeManualMessage();

#endif // SIMULATOR_PROTOCOL_H
