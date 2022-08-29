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

    /**
     * @brief Computes an orthonormal basis for the null space of a matrix.
     * @param A The matrix
     * @return The orthonormal basis for the null space of A
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> null(
        const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& A) {
        Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> cod;
        cod.compute(A);
        // Find URV^T
        Eigen::MatrixXd V          = cod.matrixZ().transpose();
        Eigen::MatrixXd null_space = V.block(0, cod.rank(), V.rows(), V.cols() - cod.rank());
        Eigen::MatrixXd P          = cod.colsPermutation();
        null_space                 = P * null_space;
        return null_space;
    }

}  // namespace RML

#endif