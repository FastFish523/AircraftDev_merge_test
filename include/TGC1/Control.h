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
#include "State.h"
#include <utility>
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    class Control {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Control() = default;

        ~Control() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 六自由度控制
         * @param step
         * @param acc_cmd_v
         * @param state
         * @param totalMass
         * @param p_body
         * @param imu_info
         * @param rel_dis
         * @param rel_dis_dot
         * @param s 参考面积
         * @return rudder moment_body
         */
        std::pair<Eigen::Vector3d, Eigen::Vector3d> P6dof_Control(double step, const Eigen::Vector3d &acc_cmd_v, const State &state, double totalMass,
                                                                  const Eigen::Vector3d &p_body, const ImuInfo &imu_info, double rel_dis, double rel_dis_dot, double s);

        /*!
         * @brief 最简单的一阶惯性环节
         * @param input
         * @param prev_output
         * @return
         */
        static double firstOrderFilter(double input, double prev_output);

        /*!
         * @brief 限幅方法
         * @param x
         * @param lower
         * @param upper
         * @return
         */
        static double limit(double x, double lower, double upper);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        double alpha_cmd = 0;
        double beta_cmd  = 0;
        double pre_ex    = 0;
        double pre_ey    = 0;
        double pre_ez    = 0;
        double ex        = 0;
        double ey        = 0;
        double ez        = 0;
        double iex       = 0;
        double iey       = 0;
        double iez       = 0;

        double last_dx = 0;
        double last_dy = 0;
        double last_dz = 0;

// endregion

// region Private Methods
    private:
// endregion
    };
}
#undef PRETTY_FILE_NAME