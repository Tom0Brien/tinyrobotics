#ifndef TR_MATH_HPP
#define TR_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

/** \file math.hpp
 * @brief Contains various math related functions.
 */
namespace tinyrobotics {

    /**
     * @brief Manually convert the given rotation matrix to [Roll, Pitch, Yaw] ZYX intrinsic euler angle (Better range
     * than Eigen conversion).
     * @param mat Rotation matrix
     * @return [Roll, Pitch, Yaw] ZYX intrinsic euler angle
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 3))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 1> rotation_to_rpy(const T& mat) {
        using std::atan2;
        using std::sqrt;
        // Eigen euler angles and with better range
        return Eigen::Matrix<Scalar, 3, 1>(
            // Roll
            atan2(mat(2, 1), mat(2, 2)),
            // Pitch
            atan2(-mat(2, 0), sqrt(mat(2, 1) * mat(2, 1) + mat(2, 2) * mat(2, 2))),
            // Yaw
            atan2(mat(1, 0), mat(0, 0)));
    }

    /**
     * @brief Convert given Euler angles in [Roll, Pitch, Yaw] ZYX intrinsic format to rotation matrix
     * @param angles [Roll, Pitch, Yaw] ZYX intrinsic euler angles
     * @return Rotation matrix corresponding to given euler angles
     */
    template <typename T,
              typename Scalar                                                                 = typename T::Scalar,
              std::enable_if_t<((T::RowsAtCompileTime == 3) && (T::ColsAtCompileTime == 1))>* = nullptr>
    inline Eigen::Matrix<Scalar, 3, 3> rpy_to_rotation(const T& angles) {
        Eigen::AngleAxis<Scalar> yawRot(angles.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        Eigen::AngleAxis<Scalar> pitchRot(angles.y(), Eigen::Matrix<Scalar, 3, 1>::UnitY());
        Eigen::AngleAxis<Scalar> rollRot(angles.x(), Eigen::Matrix<Scalar, 3, 1>::UnitX());
        Eigen::Quaternion<Scalar> quat = yawRot * pitchRot * rollRot;
        return quat.matrix();
    }

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

    /**
     * @brief Computes the error between two homogeneous transformation matrices.
     * @param H1 The first homogeneous transformation matrix.
     * @param H2 The second homogeneous transformation matrix.
     * @tparam Scalar Scalar type.
     * @return 6x1 error vector.
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 1> homogeneous_error(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& H1,
                                                  const Eigen::Transform<Scalar, 3, Eigen::Isometry>& H2) {

        Eigen::Matrix<Scalar, 3, 3> R1, R2, Re;
        Eigen::Matrix<Scalar, 3, 1> d1, d2, eps;
        Eigen::Matrix<Scalar, 6, 1> e;
        Scalar eps_norm, t;

        // Break out values
        R1 = H1.rotation();
        R2 = H2.rotation();
        d1 = H1.translation();
        d2 = H2.translation();

        // Orientation error
        Re = R1 * R2.transpose();

        // Assign linear error
        e.head(3) = d1 - d2;

        // Extract diagonal and trace
        t = Re.trace();

        // Build l variable, and calculate norm
        eps << Re(2, 1) - Re(1, 2), Re(0, 2) - Re(2, 0), Re(1, 0) - Re(0, 1);
        eps_norm = eps.norm();

        if (t > -.99 || eps_norm > 1e-10) {
            if (eps_norm < 1e-3) {
                e.tail(3) = (0.75 - t / 12) * eps;
            }
            else {
                e.tail(3) = (atan2(eps_norm, t - 1) / eps_norm) * eps;
            }
        }
        else {
            e.tail(3) = 1.570796326794897 * (Re.diagonal().array() + 1);
        }
        return e;
    }
}  // namespace tinyrobotics

#endif