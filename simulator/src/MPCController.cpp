#include "Simulator.h"
#include "OrbitalState.h"
#include "FiniteLQR.h"
#include <matplot/matplot.h>
#include <memory>
#include <Eigen/Dense>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
    
    Simulator::COE targetCOE;
    targetCOE << 0.0, 8000e3, 0.0, 0.0, 0.0, 0.0;
    Simulator::COE chaserCOE;
    chaserCOE << 0.0, 7999.8e3, 0.0, 0.0, 0.0, 0.0;

    Simulator::PV target0 = Simulator::pvFromCoe(targetCOE);
    Simulator::PV chaser0 = Simulator::pvFromCoe(chaserCOE);
    auto target =  std::make_shared<Simulator::Vehicle>(1000.0, target0);
    auto chaser = std::make_shared<Controllers::FiniteLQRVehicle>(
            100.0,         // Mass
            chaser0,       // Initial state
            targetCOE[1],  // Target SMA
            1);         // Time horizon
    Simulator::Simulator sim(target, chaser);

    sim.simulate(10000);

    auto fig = matplot::figure(true);
    fig->size(1920*2,1080*2);
    matplot::subplot(2,2,0);
    sim.record.plotChaserRTN(fig->current_axes());
    matplot::subplot(2,2,1);
    sim.record.plotECI(fig->current_axes());
    matplot::subplot(2,2,2);
    sim.record.plotChaserControlOverTime(fig->current_axes());
    matplot::subplot(2,2,3);
    sim.record.plotDistanceOverTime(fig->current_axes());
    fig->save("../outputs/chaser_rtn.jpg");

    return 0;
}
