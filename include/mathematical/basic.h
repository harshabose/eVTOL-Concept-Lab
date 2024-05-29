//
// Created by Harshavardhan Karnati on 25/05/2024.
//

#ifndef CONCEPTUAL_BASIC_H
#define CONCEPTUAL_BASIC_H

#include <iostream>
#include <array>
#include <vector>

namespace concpt::aux {
    namespace {
        template<typename dataType, std::size_t size, typename container = std::array<dataType, size>>
        class math_operators_on_simple_containers {
            explicit math_operators_on_simple_containers (const container& IN_CONTAINER) : container_(IN_CONTAINER) {}
            dataType sum () {
                return [&] <std::size_t... i> (std::index_sequence<i...>) {
                    return (static_cast<dataType>(0) + ... + container_[i]);
                }(indices{});
            }

            dataType average () {
                return [&] <std::size_t... i> (std::index_sequence<i...>) {
                    return (static_cast<dataType>(0) + ... + container_[i]) / static_cast<dataType>(size);
                }(indices{});
            }
        private:
            using indices = std::make_index_sequence<size>;
            const container& container_;
        };
    }


    template<typename dataType, std::size_t size>
    dataType sum (const std::array<dataType, size> &IN_ARRAY) {
        return concpt::aux::math_operators_on_simple_containers<dataType, size>(IN_ARRAY).sum();
    }

    template<typename dataType, std::size_t size>
    dataType average (const std::array<dataType, size> &IN_ARRAY) {
        return concpt::aux::math_operators_on_simple_containers<dataType, size>(IN_ARRAY).average();
    }

    template<typename dataType, std::size_t size>
    dataType sum (const std::vector<dataType> &IN_VECTOR) {
        return concpt::aux::math_operators_on_simple_containers<dataType, size,  std::vector<dataType>>(IN_VECTOR).sum();
    }

    template<typename dataType, std::size_t size>
    dataType average (const std::vector<dataType> &IN_VECTOR) {
        return concpt::aux::math_operators_on_simple_containers<dataType, size, std::vector<dataType>>(IN_VECTOR).average();
    }
}

#endif //CONCEPTUAL_BASIC_H
