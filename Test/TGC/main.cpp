//
// Created by 17298 on 2026/4/22.
//

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

#include "TGC/Engine.h"
#include "TGC/TGCMissile.h"
#include "Util/CoordinateHelper.h"

namespace {
    constexpr double kStep = 0.005;
    constexpr double kMaxSimTime = 1000.0;
    const Eigen::Vector3d kMissileLLA = {120.0, 40.0, 10.0};
    const Eigen::Vector3d kTargetLLA = {140.0, 5.0, 0.0};

    constexpr double kStatusPrintInterval = 200.0;
    constexpr double kGroundStopAltitude = -50.0;
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

    void printMissileState(const std::string &tag, const ModelDevelop::TGC::Missile &missile) {
        const auto lla = missile.lla();
        const auto attitude = missile.attitudeEuler();

        std::cout << tag
                  << "phase=" << missile.phaseName()
                  << ", t=" << missile.flyTime()
                  << "s, V=" << missile.V()
                  << "m/s, alt=" << lla.z()
                  << "m, dis=" << missile.targetDis()
                  << "m, theta=" << missile.velocityTheta()
                  << "deg, psi=" << missile.velocityPsi()
                  << "deg, pitch=" << attitude.y()
                  << "deg, mass=" << missile.mass()
                  << "kg, P=" << missile.P()
                  << "N" << std::endl;
    }

    void printBoostState(const int stage, const ModelDevelop::TGC::Missile &missile) {
        printMissileState("[stage " + std::to_string(stage) + " burnout] ", missile);
    }

    double calculateLaunchPsi(const Eigen::Vector3d &missileLLA, const Eigen::Vector3d &targetLLA) {
        const auto targetEcef = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
            targetEcef, missileLLA.x(), missileLLA.y());
        return ModelDevelop::Utils::CoordinateHelper::getPsi(relNue) * kRadToDeg;
    }

    ModelDevelop::TGC::Missile createMissile(const double step, const double launchPsi) {
        ModelDevelop::TGC::Missile missile;
        missile.init(step, kMissileLLA);
        missile.setTargetLLA(kTargetLLA, Eigen::Vector3d::Zero(), true);
        missile.launch(89.0, launchPsi);
        return missile;
    }

    void printSimulationHeader(const double launchPsi) {
        const auto burnOutTimes = ModelDevelop::TGC::Engine::boostBurnOutTimes();

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "TGC full trajectory test started" << std::endl;
        std::cout << "launch lla: [" << kMissileLLA.x() << ", " << kMissileLLA.y() << ", " << kMissileLLA.z() << "]" << std::endl;
        std::cout << "target lla: [" << kTargetLLA.x() << ", " << kTargetLLA.y() << ", " << kTargetLLA.z() << "]" << std::endl;
        std::cout << "launchTheta: " << 89 << " deg" << std::endl;
        std::cout << "launchPsi: " << launchPsi << " deg" << std::endl;
        std::cout << "boost burnout schedule(s): [" << burnOutTimes[0] << ", " << burnOutTimes[1] << ", " << burnOutTimes[2] << "]" << std::endl;
    }

    void printPhaseChange(const ModelDevelop::TGC::Missile &missile) {
        printMissileState("[phase change] ", missile);
    }

    void printFinalSummary(
        const ModelDevelop::TGC::Missile &missile,
        const double finalMissDistance,
        const double closestMissDistance) {
        printMissileState("[final] ", missile);
        std::cout << "[miss] final distance=" << finalMissDistance
                  << "m, closest distance=" << closestMissDistance
                  << "m" << std::endl;
    }
}

int main() {
    using ModelDevelop::TGC::Engine;

    const double launchPsi = calculateLaunchPsi(kMissileLLA, kTargetLLA);
    auto missile = createMissile(kStep, launchPsi);
    const auto burnOutTimes = Engine::boostBurnOutTimes();

    printSimulationHeader(launchPsi);
    printPhaseChange(missile);

    double nextStatusPrintTime = kStatusPrintInterval;
    size_t nextBurnOut = 0;
    std::string lastPhase = missile.phaseName();
    double closestMissDistance = missile.targetDis();
    double finalMissDistance = closestMissDistance;

    while (missile.flyTime() < kMaxSimTime - kStep * 0.5) {
        const double terminalMissDistance = missile.update();
        const double currentDistance = missile.targetDis();
        closestMissDistance = std::min(closestMissDistance, currentDistance);
        finalMissDistance = terminalMissDistance >= 0.0 ? terminalMissDistance : currentDistance;

        const std::string phase = missile.phaseName();
        if (phase != lastPhase) {
            printPhaseChange(missile);
            lastPhase = phase;
        }

        while (missile.flyTime() + kStep * 0.5 >= nextStatusPrintTime) {
            printMissileState("[status] ", missile);
            nextStatusPrintTime += kStatusPrintInterval;
        }

        while (nextBurnOut < burnOutTimes.size() && missile.flyTime() + kStep * 0.5 >= burnOutTimes[nextBurnOut]) {
            printBoostState(static_cast<int>(nextBurnOut + 1), missile);
            ++nextBurnOut;
        }

        if (missile.lla().z() < kGroundStopAltitude) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
            break;
        }
        if (terminalMissDistance >= 0.0) {
            std::cout << "simulation stopped after closest approach." << std::endl;
            break;
        }
    }

    printFinalSummary(missile, finalMissDistance, closestMissDistance);
    return 0;
}
