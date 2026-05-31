//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
#include <algorithm>
#include <cmath>
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/Engine.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::TGC {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    EigenInfo Engine::getEigenInfo(const double simStep, const double flyTime, const State &state, const double dx, const double dy, const double dz) {
        (void) simStep;
        (void) state;

        double mountedMass = 0.0;
        double thrust = 0.0;
        double stageStartTime = 0.0;

        for (size_t i = 0; i < boostStages.size(); ++i) {
            const auto &stage = boostStages[i];
            const double stageEndTime = stageStartTime + stage.burnTime;
            const double dryMass = stage.totalMass - stage.propellantMass;

            if (flyTime < stageStartTime) {
                mountedMass += stage.totalMass;
            } else if (flyTime < stageEndTime) {
                const double burnedTime = std::max(0.0, flyTime - stageStartTime);
                const double propellantFlow = stage.propellantMass / stage.burnTime;
                const double remainingPropellant = std::max(0.0, stage.propellantMass - propellantFlow * burnedTime);
                mountedMass += dryMass + remainingPropellant;
                thrust = stage.thrust;
            }

            stageStartTime = stageEndTime;
        }

        const Eigen::Vector3d P_Body = {
            thrust / 2.0 * std::cos(dx) * (std::cos(dy) + std::cos(dz)),
            thrust / 2.0 * std::cos(dx) * std::sin(dy),
            thrust / 2.0 * std::cos(dx) * std::sin(dz),
        };

        EigenInfo eigen_info;
        eigen_info.mass = mountedMass;
        eigen_info.P_body = P_Body;
        eigen_info.inertia << jx, 0, 0, 0, jy, 0, 0, 0, jz;

        return eigen_info;
    }

    void Engine::reset() {
    }

    double Engine::boostTotalTime() {
        double totalTime = 0.0;
        for (const auto &stage: boostStages) {
            totalTime += stage.burnTime;
        }
        return totalTime;
    }

    std::array<double, 3> Engine::boostBurnOutTimes() {
        std::array<double, 3> burnOutTimes{};
        double totalTime = 0.0;
        for (size_t i = 0; i < boostStages.size(); ++i) {
            totalTime += boostStages[i].burnTime;
            burnOutTimes[i] = totalTime;
        }
        return burnOutTimes;
    }

// endregion

// region Get/Set
// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME
