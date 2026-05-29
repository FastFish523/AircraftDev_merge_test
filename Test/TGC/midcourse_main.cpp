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
    constexpr double initialSpeed = 7755.04;
    constexpr double initialTheta = 7.3;

    const Eigen::Vector3d missileLLA = {125, 37.0, 60000.0};
    const Eigen::Vector3d targetLLA = {159.5, 5.5, 20000.0};
    const double initialPsi = calcPsiToTarget(missileLLA, targetLLA);

    Missile missile;



}
