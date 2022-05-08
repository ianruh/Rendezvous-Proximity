#ifndef SRC_VISUALIZATIONS_H_
#define SRC_VISUALIZATIONS_H_
#include "Simulator.h"
#include <string>
#include <matplot/matplot.h>

void allInOneVisual(const Simulator::Record& record,
        const std::string& fileName);

void trajectoryStateControl(const Simulator::Record& record,
        const std::string& fileName);

void statistics(const Simulator::Record& recod,
        const std::string& fileName);

#endif // SRC_VISUALIZATIONS_H_
