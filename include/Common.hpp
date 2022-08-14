#ifndef RML_COMMON_HPP
#define RML_COMMON_HPP

#include <tinyxml2.h>

namespace RML {

    /**
     * @brief Computes the inverse of a 4x4 homogeneous transformation matrix
     * Much faster than actually inverting it since the computations are easy
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
     * @brief Cast an Eigen vector to std::vector.
     * @param x the Eigen vector to be plotted
     * @return the std::vector containing the elements of x
     */
    template <typename Scalar>
    std::vector<Scalar> eigen_to_vec(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x) {
        // Cast the Eigen vector to a std::vector
        std::vector<Scalar> x_vec(x.data(), x.data() + x.rows() * x.cols());
        return x_vec;
    }

}  // namespace RML

#endif
