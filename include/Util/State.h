//
// Created by MikuSoft on 2026/1/23.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once 
#include"Eigen/Core"
#include"Eigen/Dense"

struct State{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d posEcf = Eigen::Vector3d::Zero();
    Eigen::Vector3d velEcf = Eigen::Vector3d::Zero();
    Eigen::Quaterniond qbn = Eigen::Quaterniond::Identity();
    Eigen::Vector3d wnb_b= Eigen::Vector3d::Zero();

};

struct D_State{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d d_posEcf = Eigen::Vector3d::Zero();
    Eigen::Vector3d d_velEcf = Eigen::Vector3d::Zero();
    Eigen::Quaterniond d_qbn = Eigen::Quaterniond::Identity();
    Eigen::Vector3d d_wnb_b= Eigen::Vector3d::Zero();
};
