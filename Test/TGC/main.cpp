//
// Created by 17298 on 2026/4/22.
//

#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "TGC/Engine.h"
#include "TGC/TGCMissile.h"
#include "Util/CoordinateHelper.h"

namespace {
    void printBoostState(const int stage, const ModelDevelop::TGC::Missile &missile) {
        const auto lla = missile.lla();
        const auto attitude = missile.attitudeEuler();
        const double theta = missile.velocityTheta();

        std::cout << "[stage " << stage << " burnout] "
                  << "t=" << missile.flyTime()
                  << "s, V=" << missile.V()
                  << "m/s, alt=" << lla.z()
                  << "m, theta=" << theta
                  << "deg, vertical_offset=" << std::abs(90.0 - theta)
                  << "deg, pitch=" << attitude.y()
                  << "deg, mass=" << missile.mass()
                  << "kg, P=" << missile.P()
                  << "N" << std::endl;
    }
}

int main() {
    using ModelDevelop::TGC::Engine;
    using ModelDevelop::TGC::Missile;

    constexpr double step = 0.01;
    const Eigen::Vector3d missileLLA = {120.0, 40.0, 10.0};
    const Eigen::Vector3d targetLLA = {130.0, 35.0, 0.0};
    const auto burnOutTimes = Engine::boostBurnOutTimes();
    const double maxSimTime = 1000;

    // 计算发射偏角：发射点 → 目标点

    const auto targetEcef = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
        targetEcef, missileLLA.x(), missileLLA.y());
    const double launchPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(relNue) * 180.0 / std::acos(-1.0);

    Missile missile;
    missile.init(step, missileLLA);
    missile.setTargetLLA(targetLLA, Eigen::Vector3d::Zero(), true);
    missile.launch(89.0, launchPsi);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "TGC vertical boost test started" << std::endl;
    std::cout << "launch lla: [" << missileLLA.x() << ", " << missileLLA.y() << ", " << missileLLA.z() << "]" << std::endl;
    std::cout << "target lla: [" << targetLLA.x() << ", " << targetLLA.y() << ", " << targetLLA.z() << "]" << std::endl;
    std::cout << "launchPsi: " << launchPsi << " deg" << std::endl;
    std::cout << "boost burnout schedule(s): [" << burnOutTimes[0] << ", " << burnOutTimes[1] << ", " << burnOutTimes[2] << "]" << std::endl;

    size_t nextBurnOut = 0;
    std::string lastPhase = missile.phaseName();
    double closestMissDistance = missile.targetDis();
    double finalMissDistance = closestMissDistance;
    std::cout << "[phase] t=" << missile.flyTime()
              << "s, phase=" << lastPhase
              << ", dis=" << missile.targetDis()
              << "m, alt=" << missile.lla().z()
              << "m, V=" << missile.V()
              << "m/s" << std::endl;

    while (missile.flyTime() < maxSimTime - step * 0.5) {
        const double terminalMissDistance = missile.update();
        const double currentDistance = missile.targetDis();
        closestMissDistance = std::min(closestMissDistance, currentDistance);
        finalMissDistance = terminalMissDistance >= 0.0 ? terminalMissDistance : currentDistance;

        const std::string phase = missile.phaseName();
        if (phase != lastPhase) {
            std::cout << "[phase] t=" << missile.flyTime()
                      << "s, phase=" << phase
                      << ", dis=" << currentDistance
                      << "m, alt=" << missile.lla().z()
                      << "m, V=" << missile.V()
                      << "m/s" << std::endl;
            lastPhase = phase;
        }

        while (nextBurnOut < burnOutTimes.size() && missile.flyTime() + step * 0.5 >= burnOutTimes[nextBurnOut]) {
            printBoostState(static_cast<int>(nextBurnOut + 1), missile);
            ++nextBurnOut;
        }

        if (missile.lla().z() < -50.0) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
            break;
        }
        if (terminalMissDistance >= 0.0) {
            std::cout << "simulation stopped after closest approach." << std::endl;
            break;
        }
    }

    std::cout << "[final] t=" << missile.flyTime()
              << "s, V=" << missile.V()
              << "m/s, alt=" << missile.lla().z()
              << "m, theta=" << missile.velocityTheta()
              << "deg, mass=" << missile.mass()
              << "kg, P=" << missile.P()
              << "N" << std::endl;
    std::cout << "[miss] final distance=" << finalMissDistance
              << "m, closest distance=" << closestMissDistance
              << "m" << std::endl;
    return 0;
}
