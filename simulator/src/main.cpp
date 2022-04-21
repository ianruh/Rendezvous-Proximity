#include <memory>
#include <argparse/argparse.hpp>
#include <stdexcept>
#include "SimulationRuns.h"
#include "Simulator.h"
#include <map>
#include <functional>
#include <matplot/matplot.h>

using namespace std::string_literals;

std::map<std::string, std::function<void(const std::string&)>> simulationRuns {
    {"below200InfiniteLQR", below200InfiniteLQR},
    {"above200InfiniteLQR", above200InfiniteLQR},
    {"above20InfiniteLQR", above20InfiniteLQR},
    {"leading2000InfiniteLQRLinearTracking", leading2000InfiniteLQRLinearTracking},
    {"leading2000InfiniteLQRNonLinearTracking", leading2000InfiniteLQRNonLinearTracking},
    {"boxGeoInfiniteLQRLinearTracking", boxGeoInfiniteLQRLinearTracking},
    {"box30000InfiniteLQRLinearTracking", box30000InfiniteLQRLinearTracking},
    {"box20000InfiniteLQRLinearTracking", box20000InfiniteLQRLinearTracking},
    {"box10000InfiniteLQRLinearTracking", box10000InfiniteLQRLinearTracking}
};

void simulate(const std::string& simulatioName, const std::string& outputName) {
    if (simulationRuns.count(simulatioName) != 1) {
        std::cerr << "Simulation name must be one of these simulations:\n";
        for(auto const& [name, func] : simulationRuns) {
            std::cerr << "    " << name << "\n";
        }
        exit(1);
    }

    simulationRuns[simulatioName](outputName);
}

void visualize(const std::string& recordName, const std::string& outputName) {

    Simulator::Record record = Simulator::Record::load(recordName);

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

    fig->save(outputName + ".jpg");
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

    argparse::ArgumentParser parser("rendevousMain", "0.0.1");

    // Verb (simulate or visualize)
    parser.add_argument("action")
        .help("Either 'simulate' or 'visualize'.\n"s +
                "    Example: rendevousMain simulate below200InfiniteLQR below200output\n"s +
                "    Example: rendevousMain visualize below200output.csv below200video\n"s)
        .action([](const std::string& value) {
            static const std::vector<std::string> choices = { "simulate", "visualize"};
            if (std::find(choices.begin(), choices.end(), value) != choices.end()) {
              return value;
            } else {
                throw std::runtime_error("Action must be either 'simulate' or 'visualize'");
            }
        });

    parser.add_argument("input")
        .help("Either a simulation name or a record csv.");

    parser.add_argument("output_name")
        .help("The output name (no final extension).");

    try {
        parser.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << parser;
        std::exit(1);
    }

    const std::string& action = parser.get("action");
    const std::string& input = parser.get("input"); 
    const std::string& outputName = parser.get("output_name");

    if(action == "simulate") {
        simulate(input, outputName + ".csv");
    } else if(action == "visualize") {
        visualize(input, outputName);
    }

    return 0;
}
