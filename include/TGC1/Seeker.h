//
// Created by 17298 on 2026/4/22.
//


#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "State.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    class Seeker {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Seeker() = default;

        ~Seeker() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 模拟导引头工作
         * @param targetPosEcf
         * @param targetVelEcf
         * @param state
         * @return
         */
        LosInfo getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
// endregion

// region Private Methods
    private:
// endregion
    };
}
#undef PRETTY_FILE_NAME