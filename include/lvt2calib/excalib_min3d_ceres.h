#ifndef excalib_min3d_ceres_H
#define excalib_min3d_ceres_H

#include <ceres/ceres.h>
#include <vector>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Dense>


class excalib_min3d_ceres
{
private:
    pcl::PointXYZ target_p, source_p;
public:
    excalib_min3d_ceres(pcl::PointXYZ target_p_, pcl::PointXYZ source_p_);
    excalib_min3d_ceres(){};
    
    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals)const{
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> src_p(T(source_p.x), T(source_p.y), T(source_p.z));
        Eigen::Matrix<T, 3, 1> src_p_tgt = q_incre.toRotationMatrix() * src_p + t_incre;

        residuals[0] = src_p_tgt[0] - T(target_p.x);
        residuals[1] = src_p_tgt[1] - T(target_p.y);
        residuals[2] = src_p_tgt[2] - T(target_p.z);

        return true;
    }

    static ceres::CostFunction *Create(pcl::PointXYZ target_p_, pcl::PointXYZ source_p_)
    {
        return (new ceres::AutoDiffCostFunction<excalib_min3d_ceres, 3, 4, 3>(new excalib_min3d_ceres(target_p_, source_p_)));
    }
};

excalib_min3d_ceres::excalib_min3d_ceres(pcl::PointXYZ target_p_, pcl::PointXYZ source_p_)
{
    target_p = target_p_;
    source_p = source_p_;
}

#endif