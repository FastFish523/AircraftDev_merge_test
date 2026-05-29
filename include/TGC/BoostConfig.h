//
// Created by 17298 on 2026/5/16.
//

#pragma once

namespace ModelDevelop::TGC::BoostConfig {

constexpr double LAUNCH_THETA_F = 45.0;

constexpr double STAGE1_START_BLEND_THETA = 70.0;
constexpr double STAGE1_END_TARGET_THETA = 30.0;

constexpr double STAGE2_START_BLEND_THETA = 30.0;
constexpr double STAGE2_END_TARGET_THETA = 0.0;

constexpr double STAGE3_SPLIT_START_THETA = 0.0;
constexpr double STAGE3_SPLIT_END_THETA = -20.0;

constexpr double STAGE3_FINAL_START_THETA = -20.0;
constexpr double STAGE3_FINAL_END_THETA = -5.0;

constexpr double TVC_LIMIT_STAGE1 = 60.0;
constexpr double TVC_LIMIT_STAGE2 = 60.0;
constexpr double TVC_LIMIT_STAGE3_SPLIT = 60.0;
constexpr double TVC_LIMIT_STAGE3_FINAL = 60.0;

constexpr double THIRD_STAGE_SPLIT_TIME = 120.0;

constexpr double TARGET_BURNOUT_ALTITUDE = 130000.0;
constexpr double TARGET_BURNOUT_VELOCITY = 6000.0;

constexpr double TERMINAL_ALTITUDE_GAIN = 2.0;
constexpr double TERMINAL_VELOCITY_GAIN = 1.0e-5;
constexpr double TERMINAL_CORRECTION_LIMIT = 2.0;

constexpr double BOOST_SPEED_THETA_GAIN = 1.6;
constexpr double BOOST_DAMPING_GAIN = 0.35;
constexpr double BOOST_PSI_GAIN = 1.2;

}  // namespace ModelDevelop::TGC::BoostConfig
