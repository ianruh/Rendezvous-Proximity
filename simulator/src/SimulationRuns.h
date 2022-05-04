#ifndef SRC_SIMULATION_RUNS_H
#define SRC_SIMULATION_RUNS_H
#include "Simulator.h"

// Linearized stablizing LQR
void below200InfiniteLQR(const std::string& fileName);
void leading2000InfiniteLQR(const std::string& fileName);
void above20InfiniteLQR(const std::string& fileName);

// Trajectory Tracking
void leading2000InfiniteLQRLinearTracking(const std::string& fileName);
void leading2000InfiniteLQRNonLinearTracking(const std::string& fileName);

// Different box SMAs
void boxInfiniteLQRLinearTracking(
        const std::string& fileName,
        double sma,
        double boxDuration);
void boxVaryingInfiniteLQRLinearTracking(
        const std::string& fileName,
        double sma,
        double boxDuration);

void boxGeoInfiniteLQRLinearTracking(const std::string& fileName);
void box30000InfiniteLQRLinearTracking(const std::string& fileName);
void box20000InfiniteLQRLinearTracking(const std::string& fileName);
void box10000InfiniteLQRLinearTracking(const std::string& fileName);

void boxGeoVaryingInfiniteLQRLinearTracking(const std::string& fileName);
void box30000VaryingInfiniteLQRLinearTracking(const std::string& fileName);
void box20000VaryingInfiniteLQRLinearTracking(const std::string& fileName);
void box10000VaryingInfiniteLQRLinearTracking(const std::string& fileName);

// MPC
void mpcStabilize(const std::string& fileName);

#endif // SRC_SIMULATION_RUNS_H
