//
// Created by 17298 on 2026/4/22.
//

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "TGC/BoostConfig.h"
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
    const Eigen::Vector3d targetLLA = {160.0, 5.0, 0.0};
    const auto burnOutTimes = Engine::boostBurnOutTimes();
    const double maxSimTime = Engine::boostTotalTime();

    // 计算发射偏角：发射点 → 目标点
    const auto targetEcef = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
        targetEcef, missileLLA.x(), missileLLA.y());
    const double launchPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(relNue) * 180.0 / std::acos(-1.0);

    Missile missile;
    missile.init(step, missileLLA);
    missile.setTargetLLA(targetLLA, Eigen::Vector3d::Zero(), true);
    missile.launch(ModelDevelop::TGC::BoostConfig::LAUNCH_THETA_F, launchPsi);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "TGC vertical boost test started" << std::endl;
    std::cout << "launch lla: [" << missileLLA.x() << ", " << missileLLA.y() << ", " << missileLLA.z() << "]" << std::endl;
    std::cout << "target lla: [" << targetLLA.x() << ", " << targetLLA.y() << ", " << targetLLA.z() << "]" << std::endl;
    std::cout << "launchPsi: " << launchPsi << " deg" << std::endl;
    std::cout << "boost burnout schedule(s): [" << burnOutTimes[0] << ", " << burnOutTimes[1] << ", " << burnOutTimes[2] << "]" << std::endl;

    size_t nextBurnOut = 0;
    while (missile.flyTime() < maxSimTime - step * 0.5) {
        missile.update();

        while (nextBurnOut < burnOutTimes.size() && missile.flyTime() + step * 0.5 >= burnOutTimes[nextBurnOut]) {
            printBoostState(static_cast<int>(nextBurnOut + 1), missile);
            ++nextBurnOut;
        }

        if (missile.lla().z() < -50.0) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
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
    return 0;
}
