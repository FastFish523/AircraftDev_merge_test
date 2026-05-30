//
// Created by 17298 on 2026/4/22.
//


#pragma once

// region Include
// region STL
#include <array>
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include <cstdio>
#include <string>
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    class Missile;

    class FileSaver {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        explicit FileSaver(const std::string &path);

        ~FileSaver();

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        void save_traj(const Missile *missile);

        void save_aero(const Missile *missile);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        /*!
         * @brief 文件存储
         */
        std::string _directory = {};
        /*!
         * @brief 弹道文件的指针
         */
        FILE *fp_traj = nullptr;
        /*!
         * @brief 气动导数文件的指针
         */
        FILE *fp_aero = nullptr;
        FILE *fp_boost = nullptr;
        std::array<bool, 4> _boostEventsSaved{false, false, false, false};
// endregion

// region Private Methods
    private:
        auto result_fp_aero() -> FILE *;

        auto result_fp_traj() -> FILE *;

// endregion
    };
}
#undef PRETTY_FILE_NAME
