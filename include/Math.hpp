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
    Eigen::Transform<Scalar, 3, Eigen::Isometry> inv(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Tinv = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        Tinv.linear()                                     = T.linear().transpose();
        Tinv.translation()                                = -T.linear().transpose() * T.translation();
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

    /**
     * @brief Computes an orthonormal basis for the null space of a matrix.
     * @param A The matrix
     * @return The orthonormal basis for the null space of A
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> null(
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
        Eigen::FullPivLU<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> lu(A);
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> A_null_space = lu.kernel();
        return A_null_space;
    }

    /**
     * @brief Computes the error between two quaternions
     * @param q1 The first quaternion
     * @param q2 The second quaternion
     * @return The error between the two quaternions
     */
    template <typename Scalar>
    Scalar quaternion_error(const Eigen::Quaternion<Scalar>& q1, const Eigen::Quaternion<Scalar>& q2) {
        return (q1.inverse() * q2).vec().norm();
    }

}  // namespace RML

#endif