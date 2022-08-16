#ifndef RML_MATH_HPP
#define RML_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace RML {
    /**
     * @brief Computes the inverse of a 4x4 homogeneous transformation matrix
     * @param T The 4x4 homogeneous transformation matrix
     * @return The 4x4 inverse homogeneous transformation matrix
     */
    template <typename Scalar>
    Eigen::Transform<Scalar, 3, Eigen::Affine> inv(const Eigen::Transform<Scalar, 3, Eigen::Affine>& T) {
        Eigen::Transform<Scalar, 3, Eigen::Affine> Tinv = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
        Tinv.linear()                                   = T.linear().transpose();
        Tinv.translation()                              = -T.linear().transpose() * T.translation();
        return Tinv;
    }

    /**
     * @brief Computes the skew of a 3x1 vector
     * @param V The 3x1 vector
     * @return The skew of V
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& V) {
        Eigen::Matrix<Scalar, 3, 3> S;
        S << 0, -V(2), V(1), V(2), 0, -V(0), -V(1), V(0), 0;
        return S;
    }


}  // namespace RML

#endif