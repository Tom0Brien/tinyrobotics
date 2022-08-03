#ifndef RML_COMMON_HPP
#define RML_COMMON_HPP

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

namespace RML {

		using std::cos;
		using std::sin;
		using std::sqrt;
		using std::pow;
		using std::atan2;

		/**
		 * @brief Convert rotation matrix to euler angles Thetaxy = [Roll, Pitch, Yaw]
		 */
		template <typename Scalar>
		void rot2rpy(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& Rxy,
						Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Thetaxy) {
			assert(Rxy.rows() == 3);
			assert(Rxy.cols() == 3);
			Thetaxy.resize(3, 1);
			Thetaxy << atan2(Rxy(2, 1), Rxy(2, 2)), atan2(-Rxy(2, 0), sqrt(pow(Rxy(2, 1), 2) + pow(Rxy(2, 2), 2))),
				atan2(Rxy(1, 0), Rxy(0, 0));
		}

		/**
		 * @brief Convert rotation matrix to euler angles Thetaxy = [Roll, Pitch, Yaw]
		 */
		template <typename Scalar>
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rot2rpy(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& Rxy) {
			assert(Rxy.rows() == 3);
			assert(Rxy.cols() == 3);
			Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Thetaxy;
			Thetaxy.resize(3, 1);
			Thetaxy << std::atan2(Rxy(2, 1), Rxy(2, 2)),
				std::atan2(-Rxy(2, 0), std::sqrt(pow(Rxy(2, 1), 2) + std::pow(Rxy(2, 2), 2))),
				std::atan2(Rxy(1, 0), Rxy(0, 0));
			return Thetaxy;
		}

		/**
		 * @brief Rotation matrix around x-axis
		 */
		template <typename Scalar>
		void rotx(const Scalar& x, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& R) {
			R.resize(3, 3);
			R << 1, 0, 0, 0, cos(x), -sin(x), 0, sin(x), cos(x);
		}

		/**
		 * @brief Rotation matrix around y-axis
		 */
		template <typename Scalar>
		void roty(const Scalar& x, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& R) {
			R.resize(3, 3);
			R << cos(x), 0, sin(x), 0, 1, 0, -sin(x), 0, cos(x);
		}

		/**
		 * @brief Rotation matrix around z-axis
		 */
		template <typename Scalar>
		void rotz(const Scalar& x, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& R) {
			R.resize(3, 3);
			R << cos(x), -sin(x), 0, sin(x), cos(x), 0, 0, 0, 1;
		}

		/**
		 * @brief Convert euler angles Thetaxy = [Roll, Pitch, Yaw] to rotation matrix
		 */
		template <typename Scalar>
		void rpy2rot(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Thetaxy,
						Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& Rxy) {
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Rx;
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Ry;
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Rz;
			rotz(Thetaxy(2), Rz);
			roty(Thetaxy(1), Ry);
			rotx(Thetaxy(0), Rx);
			Rxy.resize(3, 3);
			Rxy = Rz * Ry * Rx;
		}
		/**
		 * @brief Convert euler angles Thetaxy = [Roll, Pitch, Yaw] to rotation matrix
		 */
		template <typename Scalar>
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> rpy2rot(
			const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& Thetaxy) {
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Rx;
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Ry;
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Rz;
			rotz(Thetaxy(2), Rz);
			roty(Thetaxy(1), Ry);
			rotx(Thetaxy(0), Rx);
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Rxy = Rz * Ry * Rx;
			return Rxy;
		}

        /**
         * @brief Get Eigen3 vector from the XML element.
         * @param xml The XML element
         * @return The vector
         */
		template <typename Scalar>
        Eigen::Matrix<Scalar, 3, 1> vec_from_string(const std::string& vector_str) {
            Eigen::Matrix<Scalar, 3, 1> vec;
            std::vector<std::string> pieces;
            std::vector<Scalar> values;
            boost::split( pieces, vector_str, boost::is_any_of(" "));
            for (unsigned int i = 0; i < pieces.size(); ++i){
                if (pieces[i] != ""){
                    try {
                        values.push_back(boost::lexical_cast<Scalar>(pieces[i].c_str()));
                    } catch (boost::bad_lexical_cast &e) {
                        throw std::runtime_error("Error not able to parse component (" + pieces[i] + ") to a Scalar (while parsing a vector value)");
                    }
                }
            }

            if (values.size() != 3) {
                std::ostringstream error_msg;
                error_msg << "Parser found " << values.size()
                        << " elements but 3 expected while parsing vector ["
                        << vector_str <<  "]";
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
		Eigen::Matrix<Scalar, 3, 3> rot_from_string(const string &rotation_str) {
			Eigen::Matrix<Scalar, Eigen::Dynamic, 1> rpy = vec_from_string<Scalar>(rotation_str);
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> R;
			R = rpy2rot(rpy);
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

}

#endif
