#ifndef TR_FMIN_HPP
#define TR_FMIN_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nlopt.hpp>

namespace tinyrobotics {

    /**
     * @brief Converts an Eigen vector to an std::vector.
     * @param eigen_vec The Eigen vector to convert.
     * @param nlopt_vec The resulting std::vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void eigen_to_nlopt(const Eigen::Matrix<Scalar, n, 1>& eigen_vec, std::vector<Scalar>& nlopt_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        Eigen::Map<Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the Eigen vector to the std::vector view
        nlopt_eigen_vec = eigen_vec;
    }


    /**
     * @brief Converts an std::vector to an Eigen vector.
     * @param nlopt_vec The std::vector to convert.
     * @param eigen_vec The resulting Eigen vector.
     * @tparam Scalar The scalar type of the Eigen vector.
     * @tparam n The size of the Eigen vector.
     */
    template <typename Scalar, int n>
    inline void nlopt_to_eigen(const std::vector<Scalar>& nlopt_vec, Eigen::Matrix<Scalar, n, 1>& eigen_vec) {
        // Use Eigen::Map to create a view of the std::vector data
        const Eigen::Map<const Eigen::Matrix<Scalar, n, 1>> nlopt_eigen_vec(nlopt_vec.data(), n);

        // Copy the data from the std::vector view to the Eigen vector
        eigen_vec = nlopt_eigen_vec;
    }

    /**
     * @brief Type definition for an objective function that takes an Eigen vector as input and returns a scalar value.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     */
    template <typename Scalar, int nv>
    using ObjectiveFunction =
        std::function<Scalar(const Eigen::Matrix<Scalar, nv, 1>&, Eigen::Matrix<Scalar, nv, 1>&, void*)>;

    /**
     * @brief Wrapper function that converts input and output between NLopt and Eigen formats for an objective function.
     * @param n The size of the input vector.
     * @param x The input vector in NLopt format.
     * @param grad The gradient vector in NLopt format.
     * @param data Pointer to additional data that is passed to the objective function.
     * @tparam Scalar The scalar type of the Eigen vector and the scalar return value.
     * @tparam nv The size of the Eigen vector.
     * @return The scalar value of the objective function.
     */
    template <typename Scalar, int nv>
    inline Scalar eigen_objective_wrapper(unsigned n, const Scalar* x, Scalar* grad, void* data) {
        ObjectiveFunction<Scalar, nv>& obj_fun = *static_cast<ObjectiveFunction<Scalar, nv>*>(data);

        // Convert input from NLopt format to Eigen format
        Eigen::Map<const Eigen::Matrix<Scalar, nv, 1>> eigen_x(x, n);
        Eigen::Matrix<Scalar, nv, 1> eigen_grad;

        if (grad) {
            eigen_grad.resize(n);
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        // Call the actual objective function implemented with Eigen
        Scalar result = obj_fun(eigen_x, eigen_grad, data);

        if (grad) {
            // Copy the gradient back to NLopt format
            Eigen::Map<Eigen::Matrix<Scalar, nv, 1>>(grad, n) = eigen_grad;
        }

        return result;
    }


}  // namespace tinyrobotics

#endif  // TR_FMIN_HPP