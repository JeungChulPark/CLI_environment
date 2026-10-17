/**
 * GYRO AID (objpose, 2026-09-18) — rotation-only constraints from an external gyro for the NON-inertial
 * sensor modes (RGB-D / stereo / mono). This is not ORB-SLAM3's visual-inertial mode: no accelerometer,
 * no velocity / bias / gravity states, no IMU initialisation. The caller integrates the gyro (bias removed,
 * rotated into the camera frame) and hands each frame its gyro-only orientation (Frame::mRgc); only the
 * relative rotation between two stamps is ever used, so gyro drift enters through the time gap alone.
 *
 *   EdgeGyroRotPrior     unary, PoseOptimization:      R_cw  ~  R_(c <- last, gyro) * R_(last,w)
 *   EdgeGyroRelRot       binary, LocalBundleAdjustment: R_ciw * R_cjw^T  ~  R_(ci <- cj, gyro)
 *
 * Both use g2o's numeric Jacobians (one unary edge per frame, a few tens of binary edges per local BA).
 */
#ifndef ORB_SLAM3_GYROEDGES_H
#define ORB_SLAM3_GYROEDGES_H

#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM3
{

inline Eigen::Vector3d GyroLogSO3(const Eigen::Matrix3d &R)
{
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

// error = Log( R_prior^T * R_cw )
class EdgeGyroRotPrior : public g2o::BaseUnaryEdge<3, Eigen::Matrix3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGyroRotPrior() {}
    virtual bool read(std::istream&) { return false; }
    virtual bool write(std::ostream&) const { return false; }
    void computeError()
    {
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        _error = GyroLogSO3(_measurement.transpose() * v->estimate().rotation().toRotationMatrix());
    }
};

// measurement = R_(ci <- cj) from the gyro; error = Log( meas^T * R_ciw * R_cjw^T )
class EdgeGyroRelRot : public g2o::BaseBinaryEdge<3, Eigen::Matrix3d, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeGyroRelRot() {}
    virtual bool read(std::istream&) { return false; }
    virtual bool write(std::ostream&) const { return false; }
    void computeError()
    {
        const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const Eigen::Matrix3d Rij = vi->estimate().rotation().toRotationMatrix() *
                                    vj->estimate().rotation().toRotationMatrix().transpose();
        _error = GyroLogSO3(_measurement.transpose() * Rij);
    }
};

} // namespace ORB_SLAM3

#endif
