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
#include <deque>
#include <filesystem>
#include <optional>
#include <string_view>
#include "Control.h"
#include "Engine.h"
#include "FileSaver.h"
#include "Guidance.h"
#include "IMU.h"
#include "Eigen/Core"
#include "CoordinateHelper.h"
#include "Kinematics.h"
// endregion
// endregion
// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
#if defined(_WIN32) && !defined(StaticTGC_Build)
#ifdef SharedTGC_Build
#define DLL_EXPORT_IMPORT __declspec(dllexport)
#else
#define DLL_EXPORT_IMPORT __declspec(dllimport)
#endif
#else
#define DLL_EXPORT_IMPORT
#endif
// endregion
// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    class DLL_EXPORT_IMPORT Missile {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Missile();

        ~Missile();

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 模型初始化
         * @param step  仿真步长 秒
         * @param lla 经纬高 度 米
        */
        void init(double step, const Eigen::Vector3d &lla);

        void initDiveTest(double step, const Eigen::Vector3d &lla, double speed, double theta_d, double psi_d);

        /*!
         * @brief 发射
         * @param theta_f_d 发射倾角 度
         * @param psi_f_d 发射偏角 度
         */
        void launch(double theta_f_d, double psi_f_d);

        /*!
         * @brief 设置目标
         * @param targetPosEcf 目标位置
         * @param targetVelEcf 目标速度
         */
        void setTargetEcf(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const bool clearQueue);

        /*!
         * @brief 设置目标
         * @param targetPosLLa 目标位置
         * @param targetVelEcf 目标速度
         */
        void setTargetLLA(const Eigen::Vector3d &targetPosLLa, const Eigen::Vector3d &targetVelEcf, const bool clearQueue);

        /*!
         * @brief 仿真更新
         */
        double update();

// endregion

// region Get/Set选择器
    public:
        /*!
         * 获取飞行时间 秒
         * @return
         */
        [[nodiscard]]
        double flyTime() const {
            return _flyTime;
        }

        /*!
         * @brief 获取自身发射系nue位置
         * @return
         */
        Eigen::Vector3d positionLaunchNUE() const {
            Eigen::Vector3d ret = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(_state.posEcf, _launchLLA.x(), _launchLLA.y());
            return ret;
        }

        /*!
         * @brief 获取标量速度
         * @return
         */
        [[nodiscard]]
        double V() const {
            return _state.velEcf.norm();
        }

        /*!
         * @brief 获取标量速度ma
         * @return
         */
        [[nodiscard]]
        double Ma() const {
            return _state.velEcf.norm() / 340.0;
        }

        /*!
         * @brief 获取自身欧拉角 度
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d attitudeEuler() const {
            return ModelDevelop::Utils::CoordinateHelper::quaternionToEuler231(_state.qbn);
        }

        /*!
         * @brief 获取速度倾角 度
         * @return
         */
        [[nodiscard]]
        double velocityTheta() const {
            Eigen::Vector3d lla           = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            const Eigen::Vector3d vel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(_state.velEcf, lla.x(), lla.y());
            return ModelDevelop::Utils::CoordinateHelper::getTheta(vel_nue) * 57.3;
        }

        /*!
         * @brief 获取速度倾角指令 度
         * @return
         */
        [[nodiscard]]
        double thetaCmd() const {
            return _theta_cmd * 57.3;
        }

        /*!
         * @brief 获取速度偏角 度
         * @return
         */
        [[nodiscard]]
        double velocityPsi() const {
            Eigen::Vector3d lla           = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            const Eigen::Vector3d vel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(_state.velEcf, lla.x(), lla.y());
            return ModelDevelop::Utils::CoordinateHelper::getPsi(vel_nue) * 57.3;
        }

        /*!
         * @brief 获取攻角 度
         * @return
         */
        [[nodiscard]]
        double alpha() const {
            double alpha                  = 0;
            double beta                   = 0;
            Eigen::Vector3d lla           = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            const Eigen::Vector3d vel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(_state.velEcf, lla.x(), lla.y());
            ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(vel_nue, _state.qbn, alpha, beta);
            return alpha * 57.3;
        }

        /*!
         * @brief 获取侧滑角 度
         * @return
         */
        [[nodiscard]]
        double beta() const {
            double alpha                  = 0;
            double beta                   = 0;
            Eigen::Vector3d lla           = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            const Eigen::Vector3d vel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(_state.velEcf, lla.x(), lla.y());
            ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(vel_nue, _state.qbn, alpha, beta);
            return beta * 57.3;
        }

        [[nodiscard]]
        double alphaCmd() const {
            return _control.alphaCmd();
        }

        [[nodiscard]]
        double betaCmd() const {
            return _control.betaCmd();
        }

        /*!
         * @brief 获取体系加速度 m/s2
         * @return
         */
        [[nodiscard]] Eigen::Vector3d accelerationBody() const {
            return _imu_info.imu_acc_body;
        }

        /*
         ** @brief 获取体系角速度 弧度/s
         ** @return
         **/
        [[nodiscard]] Eigen::Vector3d w_xyz() const {
            return _imu_info.imu_w_xyz_body;
        }

        /*
         * @brief 获取质量 千克
         * @return
         */
        [[nodiscard]]
        double mass() const {
            return _totalMass;
        }

        /*
         * @brief 获取推力 牛
         * @return
         */
        [[nodiscard]]
        double P() const {
            return _p_body.norm();
        }

        /*
         * @brief 获取目标相对发射系 nue位置
         * @return
         */
        Eigen::Vector3d targetPositionLaunchNUE() const {
            if (_targetPosEcf.has_value()) {
                return ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(_targetPosEcf.value(), _launchLLA.x(), _launchLLA.y());
            }
            return ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(_state.posEcf, _launchLLA.x(), _launchLLA.y());
        }

        /*
         * @brief 获取舵偏 度
         * @return
         */
        [[nodiscard]] Eigen::Vector3d rudder() const {
            return _rudder * 57.3;
        }

        [[nodiscard]] Eigen::Vector3d tvcCommand() const {
            return _tvcCommand * 57.3;
        }

        /*
         * @brief 获取nue 速度 米/s
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d velocityNUE() const {
            Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            return ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(_state.velEcf, lla.x(), lla.y());
        }

        /*
         * @brief 获取经纬高
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d lla() const {
            Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            return lla;
        }

        /*
         * @brief 获取弹目距离 米
         * @return
         */
        [[nodiscard]]
        double targetDis() const {
            if (_targetPosEcf.has_value())
                return (_state.posEcf - _targetPosEcf.value()).norm();
            return -1;
        }

        /*
         * @brief 获取体法向指令加速度
         * @return
         */
        [[nodiscard]] auto acc_cmd_b_y() const -> double {
            return _acc_cmd_b_y;
        }

        /*
        * @brief 获取体侧向指令加速度
        * @return
        */
        [[nodiscard]] auto acc_cmd_b_z() const -> double {
            return _acc_cmd_b_z;
        }

        /*!
         * @brief 获取视线倾角,类别速度倾角
         * @return
         */
        [[nodiscard]] auto sigmaElv() const -> double {
            return _sigma_elv * 57.3;
        }

        /*!
         * @brief 获取视线倾角变化率
         * @return
         */
        [[nodiscard]] auto sigmaElvDot() const -> double {
            return _sigma_elv_dot * 57.3;
        }

        /*!
        * @brief 获取视线偏角，类别速度偏角
        * @return
        */
        [[nodiscard]] auto sigmaAz() const -> double {
            return _sigma_az * 57.3;
        }

        /*!
        * @brief 获取视线偏角变化率
        * @return
        */
        [[nodiscard]] auto sigmaAzDot() const -> double {
            return _sigma_az_dot * 57.3;
        }


        [[nodiscard]] GuidancePhase phase() const {
            return _phase;
        }

        [[nodiscard]] int phaseId() const;

        [[nodiscard]] const char *phaseName() const;

        Derivative derivative() const;

// endregion

// region Private Attributes
    private:

        /*!
         * @brief 仿真步长
         */
        double _step = 0.005;
        /*!
         * @brief 飞行时间
         */
        double _flyTime = 0.0;
        /*!
         * @brief 发射标识
         */
        bool _launchFlag = false;

        /*!
         * @brief 自身状态
         */
        State _state{};
        /*!
         * @brief 自身质量（不包含发动机）
         */
        double _mass = 0;
        /*!
         * @brief 参考面积
         */
        double _s = 0;
        /*!
         * @brief 参考长度
         */
        double _l = 0;
        /*!
         * @brief 参考翼展
         */
        double _b = 0;

        /*!
         * @brief 总质量（包含发动机）
         */
        double _totalMass = 0;
        /*!
         * @brief 最大过载
         */
        double _maxLoad = 0;
        /*!
         * @brief 转动惯量矩阵
         */
        Eigen::Matrix3d _inertia{};

        /*!
         * @brief 发射点经纬高
         */
        Eigen::Vector3d _launchLLA{};
        /*!
         * @brief 目标位置
         */
        std::optional<Eigen::Vector3d> _targetPosEcf = std::nullopt;
        /*!
         * @brief 目标速度
         */
        Eigen::Vector3d _targetVelEcf{};
        /*!
         * @brief 体法向加速度指令
         */
        double _acc_cmd_b_y = 0;
        /*!
         * @brief 体侧向加速度指令
         */
        double _acc_cmd_b_z = 0;
        double _theta_cmd = std::numeric_limits<double>::quiet_NaN();

        /*! 
         * @brief 视线倾角
         */
        double _sigma_elv = 0;
        /*!
         * @brief 视线倾角变化率
         */
        double _sigma_elv_dot = 0;
        /*!
            * @brief 视线偏角
            */
        double _sigma_az = 0;
        /*!
         * @brief 视线偏角变化率
         */
        double _sigma_az_dot = 0;
        GuidancePhase _phase = GuidancePhase::Boost;
        /*!
         * @brief 舵偏
         */
        Eigen::Vector3d _rudder{0, 0, 0};
        Eigen::Vector3d _tvcCommand{0, 0, 0};
        /*!
         * @brief 弹体系推力向量
         */
        Eigen::Vector3d _p_body{0, 0, 0};
        /*!
         * @brief 弹体系直接力矩矢量
         */
        Eigen::Vector3d _m_body{0, 0, 0};
        /*!
         * @brief 惯组输出
         */
        ImuInfo _imu_info{};

        /*!
         * @brief 运动学模块
         */
        ModelDevelop::Utils::Kinematics _kinematics{};

        /*!
         * @brief 发动机
         */
        Engine _engine{};
        /*!
         * @brief 制导系统
         */
        Guidance _guidance{};
        /*!
         * @brief 控制系统
         */
        Control _control{};
        /*!
         * @brief 惯组
         */
        IMU _imu{};

        /*!
         * @brief 接收单目距离的队列
         */
        std::deque<double> distance_deque{};

        /*!
         * @brief 文件保存器
         */
        std::shared_ptr<FileSaver> _fileSaver = nullptr;

// endregion

// region Private Methods
    private:
        /*!
         * @brief 四阶龙格库塔
         * @param _rudder
         * @param P_body
         * @param M_body
         * @return
         */
        Eigen::Vector3d rk4(const Eigen::Vector3d &_rudder, const Eigen::Vector3d &P_body, const Eigen::Vector3d &M_body);

// endregion
    };
}
#undef PRETTY_FILE_NAME
