//
// Dive segment validation runner.
//

#include <cmath>
#include <iomanip>
#include <iostream>

#include "TGC/Engine.h"
#include "TGC/TGCMissile.h"
#include "Util/CoordinateHelper.h"

namespace {
    double calcPsiToTarget(const Eigen::Vector3d &missileLLA, const Eigen::Vector3d &targetLLA) {
        const auto targetEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
            targetEcf,
            missileLLA.x(),
            missileLLA.y());
        return ModelDevelop::Utils::CoordinateHelper::getPsi(relNue) * 180.0 / std::acos(-1.0);
    }

    void printState(const char *prefix, const ModelDevelop::TGC::Missile &missile) {
        const auto lla = missile.lla();
        std::cout << prefix
                  << " t=" << missile.flyTime()
                  << "s, phase=" << missile.phaseName()
                  << ", dis=" << missile.targetDis()
                  << "m, V=" << missile.V()
                  << "m/s, alt=" << lla.z()
                  << "m, theta=" << missile.velocityTheta()
                  << "deg, psi=" << missile.velocityPsi()
                  << "deg, mass=" << missile.mass()
                  << "kg, P=" << missile.P()
                  << "N" << std::endl;
    }
}

int main() {
    using ModelDevelop::TGC::Missile;

    constexpr double step = 0.01;
    constexpr double maxDiveTime = 200.0;
    constexpr double initialSpeed = 8000.0;
    constexpr double initialTheta = -12.0;

    const Eigen::Vector3d missileLLA = {159.5, 5.5, 20000.0};
    const Eigen::Vector3d targetLLA = {160.0, 5.0, 0.0};
    const double initialPsi = calcPsiToTarget(missileLLA, targetLLA);

    Missile missile;
    missile.initDiveTest(step, missileLLA, initialSpeed, initialTheta, initialPsi);
    missile.setTargetLLA(targetLLA, Eigen::Vector3d::Zero(), true);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "TGC dive segment validation started" << std::endl;
    std::cout << "initial lla: [" << missileLLA.x() << ", " << missileLLA.y() << ", " << missileLLA.z() << "]" << std::endl;
    std::cout << "target lla: [" << targetLLA.x() << ", " << targetLLA.y() << ", " << targetLLA.z() << "]" << std::endl;
    std::cout << "initial theta: " << initialTheta << " deg, initial psi: " << initialPsi << " deg" << std::endl;
    printState("[initial]", missile);

    std::string lastPhase = missile.phaseName();
    double closestMissDistance = missile.targetDis();
    while (missile.flyTime() < ModelDevelop::TGC::Engine::boostTotalTime() + maxDiveTime) {
        const double terminalDistance = missile.update();
        const double currentDistance = missile.targetDis();
        if (currentDistance >= 0.0 && currentDistance < closestMissDistance) {
            closestMissDistance = currentDistance;
        }
        const std::string phase = missile.phaseName();
        if (phase != lastPhase) {
            printState("[phase change]", missile);
            lastPhase = phase;
        }

        if (terminalDistance >= 0.0) {
            if (terminalDistance < closestMissDistance) {
                closestMissDistance = terminalDistance;
            }
            std::cout << "[hit/closest] terminal distance=" << terminalDistance << "m" << std::endl;
            break;
        }
        if (missile.lla().z() < -50.0) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
            break;
        }
    }

    printState("[final]", missile);
    std::cout << "[miss] final distance=" << missile.targetDis()
              << "m, closest miss distance=" << closestMissDistance
              << "m" << std::endl;
    std::cout << "trajectory saved to Results/TGC/result.dat" << std::endl;
    return 0;
}
