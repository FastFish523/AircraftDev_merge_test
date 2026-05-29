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
#include <Eigen/Core>
#include <Eigen/Dense>
#include "CoordinateHelper.h"
#include "State.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    class IMU {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        IMU() = default;

        ~IMU() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 获取imu信息
         * @param state 自身状态
         * @param total_acc_ecf ecf系下和力 后续减去重力作为加表输出
         * @return
         */
        ImuInfo getImuInfoBody(const State &state, const Eigen::Vector3d &total_acc_ecf);

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