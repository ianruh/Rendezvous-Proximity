#include "Simulator.h"
#include "Constants.h"
#include "OrbitalState.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <matplot/matplot.h>
#include <string>
#include <boost/numeric/odeint.hpp>
#include "external/rapidcsv.h"

namespace Simulator {

void Vehicle::integrate(double duration, Eigen::Vector3d control) {
    double dt = 0.01;
    double time = 0.0;

    while(time < duration) {
        if(time + dt > duration) {
            dt = duration - time;
        }

        Eigen::Vector3d controlAccel = control / this->mass;
        Eigen::Vector3d gravityAccel = -1 * MU_EARTH / 
            std::pow(this->state.head(3).norm(), 3) * this->state.head(3);
        Eigen::Vector3d accel = controlAccel + gravityAccel;
        Eigen::Vector3d vel = this->state.tail(3) + dt * accel;
        Eigen::Vector3d pos = this->state.head(3) + dt * vel;

        this->state.head(3) = pos;
        this->state.tail(3) = vel;

        time += dt;
    }
}

void Record::write(const std::string& fileName) const {
    std::ofstream file;
    file.open(fileName);
    // Header
    file << "time,";
    file << "target_x,target_y,target_z,target_vx,target_vy,target_vz,";
    file << "chaser_x,chaser_y,chaser_z,chaser_vx,chaser_vy,chaser_vz,";
    file << "target_cx,target_cy,target_cz,chaser_cx,chaser_cy,chaser_cz\n";
    
    for(size_t i = 0; i < this->times.size(); i++) {
        file <<
            times[i] << "," <<
            std::to_string(targetState[i](0)) << "," <<
            std::to_string(targetState[i](1)) << "," <<
            std::to_string(targetState[i](2)) << "," <<
            std::to_string(targetState[i](3)) << "," <<
            std::to_string(targetState[i](4)) << "," <<
            std::to_string(targetState[i](5)) << "," <<
            std::to_string(chaserState[i](0)) << "," <<
            std::to_string(chaserState[i](1)) << "," <<
            std::to_string(chaserState[i](2)) << "," <<
            std::to_string(chaserState[i](3)) << "," <<
            std::to_string(chaserState[i](4)) << "," <<
            std::to_string(chaserState[i](5)) << "," <<
            std::to_string(targetControl[i](0)) << "," <<
            std::to_string(targetControl[i](1)) << "," <<
            std::to_string(targetControl[i](2)) << "," <<
            std::to_string(chaserControl[i](0)) << "," <<
            std::to_string(chaserControl[i](1)) << "," <<
            std::to_string(chaserControl[i](2)) << "\n";
    }

    file.close();
}

Record Record::load(const std::string& fileName) {
    Record record;
    rapidcsv::Document doc(fileName);

    std::vector<double> times = doc.GetColumn<double>("time");
    std::vector<double> target_x = doc.GetColumn<double>("target_x");
    std::vector<double> target_y = doc.GetColumn<double>("target_y");
    std::vector<double> target_z = doc.GetColumn<double>("target_z");
    std::vector<double> target_vx = doc.GetColumn<double>("target_vx");
    std::vector<double> target_vy = doc.GetColumn<double>("target_vy");
    std::vector<double> target_vz = doc.GetColumn<double>("target_vz");
    std::vector<double> chaser_x = doc.GetColumn<double>("chaser_x");
    std::vector<double> chaser_y = doc.GetColumn<double>("chaser_y");
    std::vector<double> chaser_z = doc.GetColumn<double>("chaser_z");
    std::vector<double> chaser_vx = doc.GetColumn<double>("chaser_vx");
    std::vector<double> chaser_vy = doc.GetColumn<double>("chaser_vy");
    std::vector<double> chaser_vz = doc.GetColumn<double>("chaser_vz");
    std::vector<double> target_cx = doc.GetColumn<double>("target_cx");
    std::vector<double> target_cy = doc.GetColumn<double>("target_cy");
    std::vector<double> target_cz = doc.GetColumn<double>("target_cz");
    std::vector<double> chaser_cx = doc.GetColumn<double>("chaser_cx");
    std::vector<double> chaser_cy = doc.GetColumn<double>("chaser_cy");
    std::vector<double> chaser_cz = doc.GetColumn<double>("chaser_cz");

    record.times = times;
    for(size_t i = 0; i < times.size(); i++) {
        PV targetState;
        targetState <<
            target_x[i],
            target_y[i],
            target_z[i],
            target_vx[i],
            target_vy[i],
            target_vz[i];

        PV chaserState;
        chaserState <<
            chaser_x[i],
            chaser_y[i],
            chaser_z[i],
            chaser_vx[i],
            chaser_vy[i],
            chaser_vz[i];

        Eigen::Vector3d targetControl;
        targetControl <<
            target_cx[i],
            target_cy[i],
            target_cz[i];

        Eigen::Vector3d chaserControl;
        chaserControl <<
            chaser_cx[i],
            chaser_cy[i],
            chaser_cz[i];


        record.targetState.push_back(targetState);
        record.chaserState.push_back(chaserState);
        record.targetControl.push_back(targetControl);
        record.chaserControl.push_back(chaserControl);
    }

    return record;
}

void Record::plotChaserRTN(matplot::axes_handle ax) const {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(size_t i = 0; i < this->chaserState.size(); i++) {
        RTN rtn = pvToRtn(this->chaserState.at(i), this->targetState.at(i));
        x.push_back(rtn(0));
        y.push_back(rtn(1));
        z.push_back(rtn(2));
    }

    std::vector<double> targetPos {0.0};

    auto cp = ax->plot3(x, y, z);
    ax->title("Chaser in RTN Frame");
    ax->xlabel("R (m.)");
    ax->ylabel("T (m.)");
    ax->zlabel("N (m.)");
    ax->grid(matplot::off);
    ax->hold(matplot::on);
    auto tp = ax->scatter3(targetPos, targetPos, targetPos);
    tp->marker_face(true);
    tp->marker_size(20);
    ax->hold(matplot::off);
    ax->axis(matplot::equal);
    ax->legend({"Chaser", "Target"});
}

void Record::plotECI(matplot::axes_handle ax) const {
    std::vector<double> tx;
    std::vector<double> ty;
    std::vector<double> tz;
    std::vector<double> cx;
    std::vector<double> cy;
    std::vector<double> cz;

    for(size_t i = 0; i < this->chaserState.size(); i++) {
        tx.push_back(this->targetState[i](0));
        ty.push_back(this->targetState[i](1));
        tz.push_back(this->targetState[i](2));
        cx.push_back(this->chaserState[i](0));
        cy.push_back(this->chaserState[i](1));
        cz.push_back(this->chaserState[i](2));
    }

    ax->title("Target and Chaser in ECI Frame");
    auto tp = ax->plot3(tx, ty, tz);
    ax->hold(matplot::on);
    auto cp = ax->plot3(cx, cy, cz);
    ax->xlabel("X (m.)");
    ax->ylabel("Y (m.)");
    ax->zlabel("Z (m.)");
    ax->grid(matplot::off);
    ax->hold(matplot::off);
    ax->axis(matplot::equal);
    ax->legend({"Target", "Chaser"});
}

void Record::plotDistanceOverTime(matplot::axes_handle ax) const {
    std::vector<double> distances;
    for(size_t i = 0; i < this->times.size(); i++) {
        Eigen::Vector3d chaserPos = this->chaserState[i].head(3);
        Eigen::Vector3d targetPos = this->targetState[i].head(3);
        distances.push_back((chaserPos - targetPos).norm());
    }

    ax->title("Distance from Chaser to Target");
    auto cp = ax->plot(this->times, distances);
    ax->xlabel("Time (s.)");
    ax->ylabel("Distance (m.)");
}

void Record::plotChaserControlOverTime(matplot::axes_handle ax) const {
    std::vector<double> controlX;
    std::vector<double> controlY;
    std::vector<double> controlZ;
    for(size_t i = 0; i < this->times.size(); i++) {
        controlX.push_back(this->chaserControl[i](0));
        controlY.push_back(this->chaserControl[i](1));
        controlZ.push_back(this->chaserControl[i](2));
    }

    ax->title("Chaser Control");
    ax->hold(matplot::on);
    auto xp = ax->plot(this->times, controlX);
    auto yp = ax->plot(this->times, controlY);
    auto zp = ax->plot(this->times, controlZ);
    ax->xlabel("Time (s.)");
    ax->ylabel("Acceleration (m/s^2.)");
    ax->legend({"X", "Y", "Z"});
    ax->hold(matplot::off);
}

Simulator::Simulator(std::shared_ptr<Vehicle> target,
        std::shared_ptr<Vehicle> chaser,
        double controlFrequency,
        double recordTimeStep,
        double controlSaturation) {
    this->target = target;
    this->chaser = chaser;
    this->targetControl = {0.0, 0.0, 0.0};
    this->chaserControl = {0.0, 0.0, 0.0};

    this->controlFrequency = controlFrequency;
    this->controlTimestep = 1 / controlFrequency;
    this->controlSaturation = controlSaturation;

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

        if(targetControl.norm() > this->controlSaturation) {
            targetControl = targetControl / targetControl.norm() * controlSaturation;
        }

        if(chaserControl.norm() > this->controlSaturation) {
            chaserControl = chaserControl / chaserControl.norm() * controlSaturation;
        }

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
