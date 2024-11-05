/**
*    This file is part of OV²SLAM.
*    
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/
#pragma once


#include <Eigen/Core>

class PointXYZParametersBlock {
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointXYZParametersBlock() {}

    PointXYZParametersBlock(const int id, const Eigen::Vector3d &X) {
        id_ = id;
        Eigen::Map<Eigen::Vector3d> (values_, 3, 1) = X;
    }

    PointXYZParametersBlock(const PointXYZParametersBlock &block) {
        id_ = block.id_;
        for( size_t i = 0 ; i < ndim_ ; i++ ) {
            values_[i] = block.values_[i];
        }
    }

    PointXYZParametersBlock& operator = (const PointXYZParametersBlock &block) 
    { 
        id_ = block.id_;
        for( size_t i = 0 ; i < ndim_ ; i++ ) {
            values_[i] = block.values_[i];
        }
        return *this; 
    }

    Eigen::Vector3d getPoint() {
        Eigen::Map<Eigen::Vector3d> X(values_);
        return X;
    }

    inline double* values() {  
        return values_; 
    }

    static const size_t ndim_ = 3;
    double values_[ndim_] = {0.,0.,0.};
    int id_= -1;
};