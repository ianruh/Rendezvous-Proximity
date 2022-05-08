# ARCHIVE BRANCH

# Rendezvous & Proximity Operations

This respository contains the code for the restricted two-body simulator and controllers that were implemented for the final project of my Spring 2022 ME 601 - Autonomous Feedback course.

The final report can be found [here](https://github.com/ianruh/Rendezvous-Proximity/blob/master/report/latexbuild/Report.pdf).

**Example Trajectory Tracking Simulation**

![Example Simulation](./simulator/outputs/box30000InfiniteLQRLinearTracking-trajStateControl.jpg)

*This simulation tracked a box trajectory around the target in a circular orbit at an SMA of approx 30,000 km. using the time varying LQR trajectory tracking controller.*

## Structure

The `proposal/` and `report/` directories contain the documents that were actually turned in. The `simulator/` directory contains the source code.

### Simulator Architecture

There is a `Simulator` class that takes in `Vehicle` objects, and propogates the dynamics of the system for a given duration. Each `Vehicle` can define its own control law.

Each controller has it's own source file, and each one inherits from the toplevel `Vehicle` class. The four controllers are can be found here:

- [Infinite LQR static state feedback controller](./simulator/src/InfiniteLQR.cpp)
- [Time invariant trajectory tracking LQR controller](./simulator/src/InfiniteLQRLinearTracking.cpp)
- [Time varying trajectory tracking LQR controller](./simulator/src/InfiniteLQRNonLinearTracking.cpp)
- [Time invariant MPC controller](./simulator/src/MPCNonLinearTracking.cpp)

There is one primary executable (`rendezvousMain`), that can be used to run and visualize a set of different simulations, each of which are defined as a function in [`SimulationRuns.cpp`](./simulator/src/SimulationRuns.cpp). 

## Building

`Eigen3` is the only dependency that won't be downloaded at configure time. To build, simply clone and use the provided utility script:

```
$ git clone git@github.com:ianruh/Rendezvous-Proximity.git
$ cd Rendezvous-Proximity/simulator
$ ./utils build
```

or use CMake directly:

```
$ git clone git@github.com:ianruh/Rendezvous-Proximity.git
$ cd Rendezvous-Proximity/simulator
$ mkdir build/ && cd build/
$ cmake ..
$ make
```

## Running Simulations

Once a simulation run has been defined in the `SimulationRuns.cpp` file a registered in the map of simulations at the top of `main.cpp`, then it can be run using the `rendevousMain` binary directly:

```
Usage: rendevousMain [options] action input output_name

Positional arguments:
action          Either 'simulate' or 'visualize'.
    Example: rendevousMain simulate below200InfiniteLQR below200output
    Example: rendevousMain visualize below200output.csv below200video

input           Either a simulation name or a record csv.
output_name     The output name (no final extension).

Optional arguments:
-h --help       shows help message and exits [default: false]
-v --version    prints version information and exits [default: false]
```

Or you can use the `utils` helper script (this is recommended, as I put minimal effort into the usability of the binary's CLI):

```
$ ./utils
Usage: utils <subcommand> [options]\n
Subcommands:
    build                 Build in release
    build-debug           Build in debug
    clean                 Clean build artifeacts
    run [name]            Run simulation and generate outputs
    runo [name]           Run simulation, generate and open outputs

For help with each subcommand run:
utils <subcommand> -h|--help

$ ./utils run below200InfiniteLQR
Simulating and visualizing below200InfiniteLQR finished.
```

## Dependencies

- [cppmpc](https://github.com/ianruh/cppmpc) for the MPC controller.
- [SymEngine](https://github.com/symengine/symengine) for the symbolic mathematics.
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) for linear algebra and basic vector/matrix types.
- [Matplot++](https://github.com/alandefreitas/matplotplusplus) for plotting and visualization.
- [rapidcsv](https://github.com/d99kris/rapidcsv) for CSV reading.
