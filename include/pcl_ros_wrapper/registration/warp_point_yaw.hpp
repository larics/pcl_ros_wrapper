#ifndef WARP_POINT_YAW
#define WARP_POINT_YAW

#pragma once

#include <pcl/registration/warp_point_rigid.h>
#include <boost/shared_ptr.hpp>

namespace pcl {
namespace registration {
  /** \brief @b WarpPointYaw enables 1D rotation around yaw
   * transformations for points.
   *
   * \note The class is templated on the source and target point types as well as on the
   * output scalar of the transformation matrix (i.e., float or double). Default: float.
   * \author Radu B. Rusu
   * \ingroup registration
   */
  template<typename PointSourceT, typename PointTargetT, typename Scalar = float>
  class WarpPointYaw : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
  {
  public:
    using Matrix4 = typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4;
    using VectorX = typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX;

    using Ptr = boost::shared_ptr<WarpPointYaw<PointSourceT, PointTargetT, Scalar>>;
    using ConstPtr =
      boost::shared_ptr<const WarpPointYaw<PointSourceT, PointTargetT, Scalar>>;

    /** \brief Constructor. */
    WarpPointYaw() : WarpPointRigid<PointSourceT, PointTargetT, Scalar>(1) {}

    /** \brief Empty destructor */
    ~WarpPointYaw() {}

    /** \brief Set warp parameters.
     * \param[in] p warp parameters (tx ty rz)
     */
    void setParam(const VectorX &p) override
    {
      assert(p.rows() == this->getDimension());
      Matrix4 &trans = this->transform_matrix_;

      trans = Matrix4::Identity();

      // Copy the rotation and translation components
      trans.block(0, 3, 4, 1) = Eigen::Matrix<Scalar, 4, 1>(0, 0, 0, 1.0);

      // Add only yaw rotation
      Eigen::AngleAxis<Scalar> yawAngle(p[0], Eigen::Matrix<Scalar, 3, 1>(0, 0, 1));
      Eigen::Matrix3f m;
      m = yawAngle;
      trans.topLeftCorner(3, 3) = m;
    }
  };
}// namespace registration
}// namespace pcl

#endif /* WARP_POINT_YAW */