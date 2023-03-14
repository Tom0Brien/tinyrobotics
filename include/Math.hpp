#ifndef TR_MATH_HPP
#define TR_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tr {
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
     * @brief Computes the skew of a 3x1 vector or 3x3 matrix
     * @param in The 3x1 vector or 3x3 matrix
     * @return The skew of in
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 3>& in) {
        return 0.5 * (in - in.transpose());
    }

    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> skew(const Eigen::Matrix<Scalar, 3, 1>& in) {
        Eigen::Matrix<Scalar, 3, 3> out;
        out << Scalar(0), -in(2), in(1), in(2), Scalar(0), -in(0), -in(1), in(0), Scalar(0);
        return out;
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

    /**
     * @brief Computes the 3x3 rotation matrix about the z-axis
     * @param theta The rotation angle in radians
     * @return The 3x3 rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> rz(const Scalar& theta) {
        const Scalar c = std::cos(theta);
        const Scalar s = std::sin(theta);
        Eigen::Matrix<Scalar, 3, 3> E;
        E << c, s, Scalar(0), -s, c, Scalar(0), Scalar(0), Scalar(0), Scalar(1);
        return E;
    }

    /**
     * @brief Computes the 3x3 rotation matrix about the y-axis
     * @param theta The rotation angle in radians
     * @return The 3x3 rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> ry(const Scalar& theta) {
        const Scalar c = std::cos(theta);
        const Scalar s = std::sin(theta);
        Eigen::Matrix<Scalar, 3, 3> E;
        E << c, Scalar(0), -s, Scalar(0), Scalar(1), Scalar(0), s, Scalar(0), c;
        return E;
    }

    /**
     * @brief Computes the 3x3 rotation matrix about the x-axis
     * @param theta The rotation angle in radians
     * @return The 3x3 rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> rx(const Scalar& theta) {
        const Scalar c = std::cos(theta);
        const Scalar s = std::sin(theta);
        Eigen::Matrix<Scalar, 3, 3> E;
        E << Scalar(1), Scalar(0), Scalar(0), Scalar(0), c, s, Scalar(0), -s, c;
        return E;
    }

    /**
     * @brief Computes the 6x6 spatial coordinate transform matrix for a rotation about the X-axis
     * @param theta The rotation angle in radians
     * @return The 6x6 coordinate transform matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> rotx(const Scalar theta) {
        const Scalar c = std::cos(theta);
        const Scalar s = std::sin(theta);
        Eigen::Matrix<Scalar, 6, 6> X;
        X << 1, 0, 0, 0, 0, 0, 0, c, s, 0, 0, 0, 0, -s, c, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, c, s, 0, 0, 0, 0, -s,
            c;
        return X;
    }

    /**
     * @brief Computes the 6x6 spatial coordinate transform matrix for a rotation about the Y-axis
     * @param theta The rotation angle in radians
     * @return The 6x6 coordinate transform matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> roty(const Scalar theta) {
        const Scalar c = std::cos(theta);
        const Scalar s = std::sin(theta);
        Eigen::Matrix<Scalar, 6, 6> Y;
        Y << c, 0, -s, 0, 0, 0, 0, 1, 0, 0, 0, 0, s, 0, c, 0, 0, 0, 0, 0, 0, c, 0, -s, 0, 0, 0, 0, 1, 0, 0, 0, 0, s, 0,
            c;
        return Y;
    }

    /**
     * @brief Computes the 6x6 spatial coordinate transform matrix for a rotation about the Z-axis
     * @param theta The rotation angle in radians
     * @return The 6x6 coordinate transform matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> rotz(const Scalar theta) {
        const Scalar c = std::cos(theta);
        const Scalar s = std::sin(theta);
        Eigen::Matrix<Scalar, 6, 6> Z;
        Z << c, s, 0, 0, 0, 0, -s, c, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, c, s, 0, 0, 0, 0, -s, c, 0, 0, 0, 0, 0, 0,
            1;
        return Z;
    }

    /**
     * @brief Computes the 6x6 cross-product matrix for a 6D spatial vector
     * @param v The 6D spatial vector
     * @return The 6x6 cross-product matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> crm(const Eigen::Matrix<Scalar, 6, 1>& v) {
        Eigen::Matrix<Scalar, 6, 6> vcross;
        vcross << 0, -v(2), v(1), 0, 0, 0, v(2), 0, -v(0), 0, 0, 0, -v(1), v(0), 0, 0, 0, 0, 0, -v(5), v(4), 0, -v(2),
            v(1), v(5), 0, -v(3), v(2), 0, -v(0), -v(4), v(3), 0, -v(1), v(0), 0;
        return vcross;
    }

    /**
     * @brief Computes the 3x3 cross-product matrix for a 3D vector
     * @param v The 3D vector
     * @return The 3x3 cross-product matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> crm(const Eigen::Matrix<Scalar, 3, 1>& v) {
        Eigen::Matrix<Scalar, 3, 3> vcross;
        vcross << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
        return vcross;
    }

    /**
     * @brief Computes the 6x6 (or 3x3) cross-product matrix for a motion vector and a force vector
     * @param v The motion vector
     * @return The 6x6 (or 3x3) cross-product matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> crf(const Eigen::Matrix<Scalar, 6, 1>& v) {
        return -crm(v).transpose();
    }

    /**
     * @brief Computes the 6x6 (or 3x3) cross-product matrix for a motion vector and a force vector
     * @param v The force vector
     * @return The 6x6 (or 3x3) cross-product matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> crf(const Eigen::Matrix<Scalar, 3, 1>& v) {
        return -crm(v).transpose();
    }

    /**
     * @brief Spatial coordinate transform (translation of origin).
     * @param v The spatial vector
     * @return The spatial coordinate transform matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> xlt(const Eigen::Matrix<Scalar, 3, 1>& r) {
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Identity();
        X.block(3, 0, 3, 3)           = -skew(r);
        return X;
    }

    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> R(const Eigen::Matrix<Scalar, 3, 1>& rpy) {
        Eigen::Matrix<Scalar, 3, 3> E = rx(rpy(0)) * ry(rpy(1)) * rz(rpy(2));
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Identity();
        X.block(0, 0, 3, 3)           = E;
        X.block(3, 3, 3, 3)           = E;
        return X;
    }


    /**
     * @brief Spatial coordinate transform from homogeneous transformation matrix.
     * @param H The homogeneous transformation matrix
     * @return The spatial coordinate transform matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> xlt(const Eigen::Matrix<Scalar, 4, 4>& H) {
        Eigen::Matrix<Scalar, 6, 6> X = Eigen::Matrix<Scalar, 6, 6>::Zero();
        Eigen::Matrix<Scalar, 3, 3> R = H.block(0, 0, 3, 3);
        Eigen::Matrix<Scalar, 3, 1> p = H.block(0, 3, 3, 1);

        X.block(0, 0, 3, 3) = R;
        X.block(3, 3, 3, 3) = R;
        X.block(3, 0, 3, 3) = skew(p) * R;

        return X;
    }


    /**
     * @brief Spatial inertia matrix from planar inertia matrix.
     * @param m mass of the link
     * @param C vector from the origin of the link to the center of mass
     * @param I inertia of the link
     * @return The spatial inertia matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 6, 6> spatial_inertia(const Scalar m,
                                                const Eigen::Matrix<Scalar, 3, 1>& c,
                                                const Eigen::Matrix<Scalar, 3, 3>& I) {
        Eigen::Matrix<Scalar, 6, 6> Ic;
        Eigen::Matrix<Scalar, 3, 3> C = tr::skew(c);
        Ic.setZero();
        Ic.block(0, 0, 3, 3) = I + m * C * C.transpose();
        Ic.block(0, 3, 3, 3) = m * C;
        Ic.block(3, 0, 3, 3) = m * C.transpose();
        Ic.block(3, 3, 3, 3) = m * Eigen::Matrix<Scalar, 3, 3>::Identity();
        return Ic;
    }

}  // namespace tr

#endif