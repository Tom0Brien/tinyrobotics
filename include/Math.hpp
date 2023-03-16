#ifndef TR_MATH_HPP
#define TR_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

/** \file Math.hpp
 * @brief Contains various math related functions.
 */
namespace tr {
    /**
     * @brief Computes the skew of a 3x3 vector.
     * @param input The 3x3 vector.
     * @tparam Scalar Scalar type.
     * @return The skew of input.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 3>& input) {
        return 0.5 * (input - input.transpose());
    }

    /**
     * @brief Computes the skew matrix for a 3x1 vector.
     * @param input The 3x1 vector.
     * @tparam Scalar Scalar type.
     * @return The skew of input.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& input) {
        Eigen::Matrix<Scalar, 3, 3> out;
        out << Scalar(0), -input(2), input(1), input(2), Scalar(0), -input(0), -input(1), input(0), Scalar(0);
        return out;
    }

    /**
     * @brief Computes an orthonormal basis for the null space of a matrix.
     * @param A Input matrix.
     * @tparam Scalar Scalar type.
     * @return The orthonormal basis for the null space of A.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> null(
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
        Eigen::FullPivLU<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> lu(A);
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> A_null_space = lu.kernel();
        return A_null_space;
    }

    /**
     * @brief Computes the 6x6 cross-product matrix for a 6D spatial vector.
     * @param v The 6D spatial vector.
     * @tparam Scalar Scalar type.
     * @return 6x6 cross-product matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> cross_spatial(const Eigen::Matrix<Scalar, 6, 1>& v) {
        Eigen::Matrix<Scalar, 6, 6> vcross;
        vcross << 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0, 0, 0, 0, 0, -v(5), v(4), 0, -v(2),
            v(1), v(5), 0, -v(3), v(2), 0, -v(0), -v(4), v(3), 0, -v(1), v(0), 0;
        return vcross;
    }

    /**
     * @brief Computes the 6x6 cross-product matrix for a motion vector and a force vector.
     * @param v The motion and force vector.
     * @tparam Scalar Scalar type.
     * @return 6x6 cross-product matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> cross_motion(const Eigen::Matrix<Scalar, 6, 1>& v) {
        return -cross_spatial(v).transpose();
    }

    /**
     * @brief Spatial coordinate transform from a xyz translation
     * @param v The spatial vector
     * @return Spatial coordinate transform matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> translation_to_spatial(const Eigen::Matrix<Scalar, 3, 1>& xyz) {
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Identity();
        X.block(3, 0, 3, 3)           = -skew(xyz);
        return X;
    }


    /**
     * @brief Spatial coordinate transform from roll-pitch-yaw angles.
     * @param rpy The roll-pitch-yaw angles.
     * @tparam Scalar Scalar type.
     * @return Spatial coordinate transform matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> rpy_to_spatial(const Eigen::Matrix<Scalar, 3, 1>& rpy) {
        Eigen::Matrix<Scalar, 3, 3> E = rx(rpy(0)) * ry(rpy(1)) * rz(rpy(2));
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Identity();
        X.block(0, 0, 3, 3)           = E;
        X.block(3, 3, 3, 3)           = E;
        return X;
    }


    /**
     * @brief Spatial coordinate transform from homogeneous transformation matrix.
     * @param H The homogeneous transformation matrix.
     * @tparam Scalar Scalar type.
     * @return Spatial coordinate transform matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> homogeneous_to_spatial(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& H) {
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Zero();
        X.block(0, 0, 3, 3)           = H.rotation();
        X.block(3, 3, 3, 3)           = H.rotation();
        X.block(3, 0, 3, 3)           = skew(Eigen::Matrix<Scalar, 3, 1>(H.translation())) * H.rotation();
        return X;
    }


    /**
     * @brief Spatial inertia matrix from inertia parameters.
     * @param m Mass of link.
     * @param C Vector from the origin of the link to the center of mass.
     * @param I Inertia of link.
     * @tparam Scalar Scalar type.
     * @return Spatial inertia matrix.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> inertia_to_spatial(const Scalar m,
                                                   const Eigen::Matrix<Scalar, 3, 1>& c,
                                                   const Eigen::Matrix<Scalar, 3, 3>& I) {
        Eigen::Matrix<Scalar, 6, 6> Ic;
        Eigen::Matrix<Scalar, 3, 3> C = skew(c);
        Ic.setZero();
        Ic.block(0, 0, 3, 3) = I + m * C * C.transpose();
        Ic.block(0, 3, 3, 3) = m * C;
        Ic.block(3, 0, 3, 3) = m * C.transpose();
        Ic.block(3, 3, 3, 3) = m * Eigen::Matrix<Scalar, 3, 3>::Identity();
        return Ic;
    }

}  // namespace tr

#endif