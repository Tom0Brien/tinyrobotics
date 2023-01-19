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
        // Faster method but gives different results to MATLAB's null function
        // Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> cod;
        // cod.compute(A);
        // // Find URV^T
        // Eigen::MatrixXd V          = cod.matrixZ().transpose();
        // Eigen::MatrixXd null_space = V.block(0, cod.rank(), V.rows(), V.cols() - cod.rank());
        // Eigen::MatrixXd P          = cod.colsPermutation();
        // null_space                 = P * null_space;

        // Results in same results as MATLAB's null function
        Eigen::FullPivLU<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> lu(A);
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> A_null_space = lu.kernel();
        return A_null_space;
    }

    /**
     * @brief Computes the rotation matrix associated with rotation around the X-axis by theta
     * @param theta The angle of rotation in radians
     * @return The rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> rotx(const Scalar& theta) {
        Eigen::Matrix<Scalar, 3, 3> Rx;
        Rx << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
        return Rx;
    }

    /**
     * @brief Computes the rotation matrix associated with rotation around the Y-axis by theta
     * @param theta The angle of rotation in radians
     * @return The rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> roty(const Scalar& theta) {
        Eigen::Matrix<Scalar, 3, 3> Ry;
        Ry << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
        return Ry;
    }

    /**
     * @brief Computes the rotation matrix associated with rotation around the Z-axis by theta
     * @param theta The angle of rotation in radians
     * @return The rotation matrix
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> rotz(const Scalar& theta) {
        Eigen::Matrix<Scalar, 3, 3> Rz;
        Rz << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
        return Rz;
    }

}  // namespace RML

#endif