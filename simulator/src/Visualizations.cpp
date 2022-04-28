#include "Visualizations.h"
#include <string>
#include <matplot/matplot.h>

void allInOneVisual(const Simulator::Record& record,
        const std::string& fileName) {

    auto fig = matplot::figure(true);
    fig->size(1920*2,1080*2);
    // Trajectory in RTN
    matplot::subplot(6,3,{0, 3, 6});
    record.plotChaserRTN(fig->current_axes());
    // Trajectory in ECI
    matplot::subplot(6,3,{1, 4, 7});
    record.plotECI(fig->current_axes());
    // Control over time
    matplot::subplot(6,3,{9, 12, 15});
    record.plotChaserControlOverTime(fig->current_axes());
    // Distance over time
    matplot::subplot(6,3,{10,13,16});
    record.plotDistanceOverTime(fig->current_axes());
    
    //==== Individual states over time ==== 
    matplot::subplot(6,3,2);
    record.plotChaserRTNState2D(fig->current_axes(), 0);
    matplot::subplot(6,3,5);
    record.plotChaserRTNState2D(fig->current_axes(), 1);
    matplot::subplot(6,3,8);
    record.plotChaserRTNState2D(fig->current_axes(), 2);
    matplot::subplot(6,3,11);
    record.plotChaserRTNState2D(fig->current_axes(), 3);
    matplot::subplot(6,3,14);
    record.plotChaserRTNState2D(fig->current_axes(), 4);
    matplot::subplot(6,3,17);
    record.plotChaserRTNState2D(fig->current_axes(), 5);

    fig->save(fileName);
}

void trajectoryStateControl(const Simulator::Record& record,
        const std::string& fileName) {

    auto fig = matplot::figure(true);
    fig->size(1920,1080);
    // Trajectory in RTN
    matplot::subplot(3,2,{0, 2});
    record.plotChaserRTN(fig->current_axes());
    // Control over time
    matplot::subplot(3,2,4);
    record.plotChaserControlOverTime(fig->current_axes());
    
    //==== Individual states over time ==== 
    matplot::subplot(3,2,1);
    record.plotChaserRTNState2D(fig->current_axes(), 0);
    matplot::subplot(3,2,3);
    record.plotChaserRTNState2D(fig->current_axes(), 1);
    matplot::subplot(3,2,5);
    record.plotChaserRTNState2D(fig->current_axes(), 2);

    fig->save(fileName);
}
