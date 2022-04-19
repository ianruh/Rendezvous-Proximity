#ifndef SRC_SIMULATION_RUNS_H
#define SRC_SIMULATION_RUNS_H
#include "Simulator.h"

// Linearized stablizing LQR
void below200InfiniteLQR(const std::string& fileName);
void above200InfiniteLQR(const std::string& fileName);
void above20InfiniteLQR(const std::string& fileName);

void boxInfiniteLQRLinearTracking(const std::string& fileName);
void boxInfiniteLQRNonLinearTracking(const std::string& fileName);

#endif // SRC_SIMULATION_RUNS_H
