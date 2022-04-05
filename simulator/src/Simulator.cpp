#include "Simulator.h"
#include "Constants.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>

namespace Simulator {

void Vehicle::integrate(double duration, Eigen::Vector3d control) {
    double dt = 0.01;
    double time = 0.0;

    while(time < duration) {
        if(time + dt > duration) {
            dt = duration - time;
        }

        Eigen::Vector3d controlAccel = control / this->mass;
        Eigen::Vector3d gravityAccel = -1 * MU_EARTH * this->mass / 
            std::pow(this->state.head(3).norm(), 3) * this->state.head(3);
        Eigen::Vector3d accel = controlAccel + gravityAccel;
        Eigen::Vector3d vel = this->state.tail(3) + dt * accel;
        Eigen::Vector3d pos = this->state.head(3) + dt * vel;
        
        this->state.head(3) = pos;
        this->state.tail(3) = vel;

        time += dt;
    }
}

Simulator::Simulator(std::shared_ptr<Vehicle> target,
        std::shared_ptr<Vehicle> chaser,
        double controlFrequency,
        double recordTimeStep) {
    this->target = target;
    this->chaser = chaser;
    this->targetControl = {0.0, 0.0, 0.0};
    this->chaserControl = {0.0, 0.0, 0.0};

    this->controlFrequency = controlFrequency;
    this->controlTimestep = 1 / controlFrequency;

    this->recordTimeStep = recordTimeStep;

    assert(this->controlTimestep < this->recordTimeStep);

    this->time = 0.0;
}

void  Simulator::integrate(double duration) {
    double dt = controlTimestep;

    double finalTime = this->time + duration;

    while(this->time < finalTime) {
        if(this->time + dt > finalTime) {
            dt = finalTime - this->time;
        }

        this->targetControl = {0.0, 0.0, 0.0};
        this->chaserControl = this->chaser->getControl(this->time,
                *this->target);

        this->target->integrate(dt, targetControl);
        this->chaser->integrate(dt, chaserControl);

        this->time += dt;
    }
}

void Simulator::simulate(double duration,
        bool quiet) {
    double dt = recordTimeStep;

    double finalTime = this->time + duration;

    while(this->time < finalTime) {
        if(this->time + dt > finalTime) {
            dt = finalTime - this->time;
        }
        if(!quiet) {
            double minutes = this->time / 60.0;
            double hours = minutes / 60.0;
            std::cout << "Sim Time: " << this->time << " s (" << minutes <<
                " min)(" << hours << " hrs)                            /" << 
                duration << " s\n";
        }

        // Record Data
        this->record.times.push_back(this->time);
        this->record.targetState.push_back(this->target->getPv());
        this->record.chaserState.push_back(this->chaser->getPv());
        this->record.targetControl.push_back(this->targetControl);
        this->record.chaserControl.push_back(this->chaserControl);
        
        // Do the next timestep
        this->integrate(dt);

        this->time += dt;
    }
}

} // namespace Simulator
