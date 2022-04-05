#include "Simulator.h"
#include <memory>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
    
    auto target =  std::make_shared<Simulator::Vehicle>(1000.0);
    auto chaser = std::make_shared<Simulator::Vehicle>(100.0);
    Simulator::Simulator sim(target, chaser);

    sim.simulate(10000);

    return 0;
}
