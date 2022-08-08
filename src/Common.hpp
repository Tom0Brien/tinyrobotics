#ifndef RML_COMMON_HPP
#define RML_COMMON_HPP

#include "txml.h"

namespace RML {

    /**
     * @brief
     *
     */
    template <typename Scalar>
    Eigen::Transform<Scalar, 3, Eigen::Affine> inv(const Eigen::Transform<Scalar, 3, Eigen::Affine>& T) {
        Eigen::Transform<Scalar, 3, Eigen::Affine> Tinv = Eigen::Transform<Scalar, 3, Eigen::Affine>::Identity();
        Tinv.linear()                                   = T.linear().transpose();
        Tinv.translation()                              = -T.linear().transpose() * T.translation();
        return Tinv;
    }

    /**
     * @brief Get Eigen3 vector from the XML element.
     * @param xml The XML element
     * @return The vector
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 1> vec_from_string(const std::string& vector_str) {
        Eigen::Matrix<Scalar, 3, 1> vec;
        std::vector<Scalar> values;
        std::istringstream ss(vector_str);
        std::string vector_element;
        while (ss >> vector_element) {
            try {
                values.push_back(std::stod(vector_element));
            }
            catch (std::invalid_argument& e) {
                throw std::runtime_error("Error not able to parse component (" + vector_element
                                         + ") to a Scalar (while parsing a vector value)");
            }
        }

        if (values.size() != 3) {
            std::ostringstream error_msg;
            error_msg << "Parser found " << values.size() << " elements but 3 expected while parsing vector ["
                      << vector_str << "]";
            throw std::runtime_error(error_msg.str());
        }

        vec.x() = values[0];
        vec.y() = values[1];
        vec.z() = values[2];

        return vec;
    }

    /**
     * @brief Get Eigen3 rotation from the XML element.
     * @param xml The XML element
     * @return The rotation
     */
    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, 3> rot_from_string(const std::string& rotation_str) {
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rpy = vec_from_string<Scalar>(rotation_str);
        Eigen::Matrix<Scalar, 3, 3> R;
        R = Eigen::AngleAxis<Scalar>(rpy.x(), Eigen::Matrix<Scalar, 3, 1>::UnitX())
            * Eigen::AngleAxis<Scalar>(rpy.y(), Eigen::Matrix<Scalar, 3, 1>::UnitY())
            * Eigen::AngleAxis<Scalar>(rpy.z(), Eigen::Matrix<Scalar, 3, 1>::UnitZ());
        return R;
    }

    /**
     * @brief Get the Eigen3 transform from the XML element.
     * @param xml The XML element
     * @return The transform
     */
    template <typename Scalar>
    Eigen::Transform<Scalar, 3, Eigen::Affine> transform_from_xml(TiXmlElement* xml) {
        Eigen::Transform<Scalar, 3, Eigen::Affine> t;
        t.setIdentity();
        if (xml) {
            const char* xyz_str = xml->Attribute("xyz");
            if (xyz_str != NULL) {
                t.translation() = vec_from_string<Scalar>(xyz_str);
            }

            const char* rpy_str = xml->Attribute("rpy");
            if (rpy_str != NULL) {
                t.linear() = rot_from_string<Scalar>(rpy_str);
            }
        }
        return t;
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
