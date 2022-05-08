#include <memory>
#include <argparse/argparse.hpp>
#include <stdexcept>
#include "SimulationRuns.h"
#include "Simulator.h"
#include "Visualizations.h"
#include <map>
#include <functional>
#include <matplot/matplot.h>

using namespace std::string_literals;

std::map<std::string, std::function<void(const std::string&)>> simulationRuns {
    {"below200InfiniteLQR", below200InfiniteLQR},
    {"leading2000InfiniteLQR", leading2000InfiniteLQR},
    {"above20InfiniteLQR", above20InfiniteLQR},
    {"leading2000InfiniteLQRLinearTracking", leading2000InfiniteLQRLinearTracking},
    {"leading2000InfiniteLQRNonLinearTracking", leading2000InfiniteLQRNonLinearTracking},
    {"boxGeoInfiniteLQRLinearTracking", boxGeoInfiniteLQRLinearTracking},
    {"box30000InfiniteLQRLinearTracking", box30000InfiniteLQRLinearTracking},
    {"box20000InfiniteLQRLinearTracking", box20000InfiniteLQRLinearTracking},
    {"box10000InfiniteLQRLinearTracking", box10000InfiniteLQRLinearTracking},
    {"boxGeoVaryingInfiniteLQRLinearTracking", boxGeoVaryingInfiniteLQRLinearTracking},
    {"box30000VaryingInfiniteLQRLinearTracking", box30000VaryingInfiniteLQRLinearTracking},
    {"box20000VaryingInfiniteLQRLinearTracking", box20000VaryingInfiniteLQRLinearTracking},
    {"box10000VaryingInfiniteLQRLinearTracking", box10000VaryingInfiniteLQRLinearTracking},
    {"mpcStabilize", mpcStabilize}
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

    allInOneVisual(record, outputName + ".jpg");
    trajectoryStateControl(record, outputName + "-trajStateControl.jpg");
    trajectoryStateControl(record, outputName + "-trajStateControl.svg");
    statistics(record, outputName  + "-stats.md");
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
