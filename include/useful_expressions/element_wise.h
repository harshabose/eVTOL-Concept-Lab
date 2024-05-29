//
// Created by Harshavardhan Karnati on 22/05/2024.
//

#ifndef CONCEPTUAL_ELEMENT_WISE_H
#define CONCEPTUAL_ELEMENT_WISE_H

#include <vector>
#include <array>

#include "../eigen3/Eigen/Dense"
#include "../eigen3/Eigen/Core"

namespace concpt::aux {

    namespace {
        template<typename dataType, std::size_t size, typename container = std::vector<dataType>>
        class element_wise_impl {
        public:
            explicit element_wise_impl (container& IN_LEFT) : left(IN_LEFT) {}

            container operator+ (const container& IN_RIGHT) {
                return this->apply_operator(IN_RIGHT, std::plus<dataType>());
            }

            container operator- (const container& IN_RIGHT) {
                return this->apply_operator(IN_RIGHT, std::minus<dataType>());
            }

            container operator* (const container& IN_RIGHT) {
                return this->apply_operator(IN_RIGHT, std::multiplies<dataType>());
            }

            container operator* (const dataType& IN_RIGHT) {
                return this->apply_operator(IN_RIGHT, std::multiplies<dataType>());
            }

            container operator/ (const container& IN_RIGHT) {
                return this->apply_operator(IN_RIGHT, std::divides<dataType>());
            }

            container operator/ (const dataType& IN_RIGHT) {
                return this->apply_operator(IN_RIGHT, std::divides<dataType>());
            }

            element_wise_impl<dataType, size, container>& operator+= (const container& IN_RIGHT) {
                this->left = this->apply_operator(IN_RIGHT, std::plus<dataType>());
                return *this;
            }

            element_wise_impl<dataType, size, container>& operator-= (const container& IN_RIGHT) {
                this->left = this->apply_operator(IN_RIGHT, std::minus<dataType>());
                return *this;
            }

            element_wise_impl<dataType, size, container>& operator*= (const container& IN_RIGHT) {
                this->left = this->apply_operator(IN_RIGHT, std::multiplies<dataType>());
                return *this;
            }

            element_wise_impl<dataType, size, container>& operator/= (const container& IN_RIGHT) {
                this->left = this->apply_operator(IN_RIGHT, std::divides<dataType>());
                return *this;
            }

            template<std::size_t N = size>
            requires(N == 3)
            container cross_product (const container& IN_RIGHT) {
                return container{left[1] * IN_RIGHT[2] - left[2] * IN_RIGHT[1],
                                 left[2] * IN_RIGHT[0] - left[0] * IN_RIGHT[2],
                                 left[0] * IN_RIGHT[1] - left[1] * IN_RIGHT[0]};
            }
        private:
            using indices = std::make_index_sequence<size>;
            container& left;

            template <typename operatorType>
            container apply_operator(const container& IN_RIGHT, operatorType OPERATOR) {
                return [&] <std::size_t... i> (std::index_sequence<i...>) {
                    return container{OPERATOR(this->left[i], IN_RIGHT[i])...};
                }(indices{});
            }

            template <typename operatorType>
            container apply_operator(const dataType& IN_RIGHT, operatorType OPERATOR) {
                return [&] <std::size_t... i> (std::index_sequence<i...>) {
                    return container{OPERATOR(this->left[i], IN_RIGHT)...};
                }(indices{});
            }
        };
    }

    template<std::size_t size, typename dataType>
    [[nodiscard]]
    inline
    concpt::aux::element_wise_impl<dataType, size>
    element_wise (const std::vector<dataType> &IN_LEFT) {
        return concpt::aux::element_wise_impl<dataType, size>(IN_LEFT);
    }

    template<typename dataType, std::size_t size>
    [[nodiscard]]
    inline
    concpt::aux::element_wise_impl<dataType, size, std::array<dataType, size>>
    element_wise (const std::array<dataType, size> &IN_LEFT) {
        return concpt::aux::element_wise_impl<dataType, size, std::array<dataType, size>>(IN_LEFT);
    }
}

#endif //CONCEPTUAL_ELEMENT_WISE_H
