//
// Created by Harshavardhan Karnati on 21/05/2024.
//

#ifndef CONCEPTUAL_EULER_ANGLES_H
#define CONCEPTUAL_EULER_ANGLES_H

#include <vector>
#include <array>

#include "../eigen3/Eigen/Core"
#include "../eigen3/Eigen/Dense"

#include "../unsupported/meta_checks.h"


namespace concpt::aux {




    template<typename type>
    requires (meta_checks::is_number_v<type>)
    inline
    meta_checks::remove_all_qual<type>
    rads_to_degrees (type &&IN_RADS) noexcept;

    template<typename type>
    requires (meta_checks::is_number_v<type>)
    inline
    meta_checks::remove_all_qual<type>
    degrees_to_rads (type &&IN_DEGREES) noexcept;




    namespace {
        template<typename type>
        requires (meta_checks::is_number_v<type>)
        class euler_angle {
        public:
            euler_angle () {
                this->rotation_matrix_psi.setIdentity();
                this->rotation_matrix_theta.setIdentity();
                this->rotation_matrix_phi.setIdentity();
            }
            void set_psi (const type &IN_PSI);
            void set_theta (const type &IN_THETA);
            void set_phi (const type &IN_PHI);
            Eigen::Vector<type, 3> calc (const Eigen::Vector<type, 3>& IN_VECTOR) const;
        private:
            Eigen::Matrix<type, 3, 3> rotation_matrix_psi;
            Eigen::Matrix<type, 3, 3> rotation_matrix_theta;
            Eigen::Matrix<type, 3, 3> rotation_matrix_phi;
        };
    }

    template<typename type>
    class euler_angle_operator {
    public:
        explicit euler_angle_operator (Eigen::Vector<type, 3>& IN_VECTOR);
        explicit euler_angle_operator (std::vector<type> &IN_VECTOR);
        explicit euler_angle_operator (std::array<type, 3> &IN_VECTOR);

        euler_angle_operator<type>& set_psi (const type &IN_PSI);
        euler_angle_operator<type>& set_theta (const type &IN_THETA);
        euler_angle_operator<type>& set_phi (const type &IN_PHI);

        template<class returnType = std::vector<type>>
        returnType calc () const;
    private:
        concpt::aux::euler_angle<type> euler_angle = concpt::aux::euler_angle<type>();
        Eigen::Map<Eigen::Vector<type, 3>> vector;

        std::vector<type> cal_using_vector () const;
        Eigen::Vector<type, 3> cal_using_eigen () const;
        std::array<type, 3> cal_using_array () const;
    };
}



//template<typename type>
//requires (meta_checks::is_number_v<type>)
//inline
//meta_checks::remove_all_qual<type>
//concpt::aux::rads_to_degrees (type &&IN_RADS) noexcept {
//    return std::forward<type>(IN_RADS) * 180.0f / std::numbers::pi_v<meta_checks::remove_all_qual<type>>;
//}


//template<typename type>
//requires (meta_checks::is_number_v<type>)
//inline
//meta_checks::remove_all_qual<type>
//concpt::aux::degrees_to_rads (type &&IN_DEGREES) noexcept {
//    return std::forward<type>(IN_DEGREES) * std::numbers::pi_v<meta_checks::remove_all_qual<type>> / 180.0f;
//}


template<typename type>
requires (meta_checks::is_number_v<type>)
void concpt::aux::euler_angle<type>::set_psi (const type &IN_PSI) {
    this->rotation_matrix_psi << std::cos(IN_PSI),         std::sin(IN_PSI), 0,
                                 -1.0f * std::sin(IN_PSI), std::cos(IN_PSI), 0,
                                 0,                                       0, 1;
}

template<typename type>
requires (meta_checks::is_number_v<type>)
void concpt::aux::euler_angle<type>::set_theta (const type &IN_THETA) {
    this->rotation_matrix_theta << std::cos(IN_THETA), 0.0f, -1.0f * std::sin(IN_THETA),
                                   0.0f,               1.0f,                       0.0f,
                                   std::sin(IN_THETA), 0.0f,         std::cos(IN_THETA);
}

template<typename type>
requires (meta_checks::is_number_v<type>)
void concpt::aux::euler_angle<type>::set_phi (const type &IN_PHI) {
    this->rotation_matrix_phi << 1.0f,                     0.0f,             0.0f,
                                 0.0f,         std::cos(IN_PHI), std::sin(IN_PHI),
                                 0.0f, -1.0f * std::sin(IN_PHI), std::cos(IN_PHI);
}

template<typename type>
requires (meta_checks::is_number_v<type>)
Eigen::Vector<type, 3> concpt::aux::euler_angle<type>::calc (const Eigen::Vector<type, 3> &IN_VECTOR) const {
    return this->rotation_matrix_phi * this->rotation_matrix_theta * this->rotation_matrix_psi * IN_VECTOR;
}








template<typename type>
concpt::aux::euler_angle_operator<type>::euler_angle_operator (Eigen::Vector<type, 3> &IN_VECTOR) :
        vector(IN_VECTOR.data(), IN_VECTOR.size()) {}

template<typename type>
concpt::aux::euler_angle_operator<type>::euler_angle_operator (std::vector<type> &IN_VECTOR) :
        vector(IN_VECTOR.data(), IN_VECTOR.size()) {}

template<typename type>
concpt::aux::euler_angle_operator<type>::euler_angle_operator (std::array<type, 3> &IN_ARRAY) :
        vector(IN_ARRAY.data(), IN_ARRAY.size()) {}

template<typename type>
concpt::aux::euler_angle_operator<type>&
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::set_psi (const type &IN_PSI) {
    this->euler_angle.set_psi((IN_PSI) * 3.14f / 180.0f);
    return *this;
}

template<typename type>
concpt::aux::euler_angle_operator<type>&
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::set_theta (const type &IN_THETA) {
    this->euler_angle.set_theta((IN_THETA) * 3.14f / 180.0f);
    return *this;
}

template<typename type>
concpt::aux::euler_angle_operator<type>&
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::set_phi (const type &IN_PHI) {
    this->euler_angle.set_phi((IN_PHI) * 3.14f / 180.0f);
    return *this;
}

template<typename type>
template<class returnType>
returnType
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::calc () const {
    if constexpr (std::is_same_v<returnType, std::vector<type>>)
        return this->cal_using_vector();
    else if constexpr (std::is_same_v<returnType, std::array<type, 3>>)
        return this->cal_using_array();
    else if constexpr (std::is_same_v<returnType, Eigen::Vector<type, 3>>)
        return this->cal_using_eigen();
    else {
        throw std::runtime_error("'returnType' is not supported...");
    }
}

template<typename type>
std::vector<type>
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::cal_using_vector () const {
    Eigen::Vector<type, 3> vec_ = this->euler_angle.calc(this->vector);
    return std::vector<type>(vec_.begin(), vec_.end());
}

template<typename type>
Eigen::Vector<type, 3>
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::cal_using_eigen () const {
    return this->euler_angle.calc(this->vector);
}

template<typename type>
std::array<type, 3>
concpt::aux::euler_angle_operator<type>::euler_angle_operator<type>::cal_using_array () const {
    Eigen::Vector<type, 3> vec_ = this->euler_angle.calc(this->vector);
    std::array<type, 3> arr_;
    arr_[0] = vec_(0); arr_[1] = vec_(1); arr_[2] = vec_(3);
    return arr_;
}

#endif //CONCEPTUAL_EULER_ANGLES_H
