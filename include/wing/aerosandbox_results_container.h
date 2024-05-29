//
// Created by Harshavardhan Karnati on 23/05/2024.
//

#ifndef CONCEPTUAL_AEROSANDBOX_RESULTS_CONTAINER_H
#define CONCEPTUAL_AEROSANDBOX_RESULTS_CONTAINER_H

#include "../useful_datatypes/enum_indexed_container.h"
#include "aerosandbox_result_types.h"

namespace concpt::detail {
    namespace {
        using enum_type = concpt::result_name;
        using float_container_type = concpt::declare::enum_indexed_container_type<enum_type, float, enum_type::WING_AERO_LAST - 6>;
        using array_container_type = concpt::declare::enum_indexed_container_type<enum_type, std::array<float, 3>, 6>;

        using f_value = float_container_type::value_type;
        using f_value_ref = float_container_type::value_ref;
        using f_value_cref = float_container_type::value_cref;
        static constexpr float_container_type::size_type f_size = float_container_type::size;

        using arr_value = array_container_type::value_type;
        using arr_value_ref = array_container_type::value_ref;
        using arr_value_cref = array_container_type::value_cref;
        static constexpr array_container_type::size_type arr_size = array_container_type::size;

        template<typename type> static constexpr bool check_input_f = std::is_same_v<type, f_value>;
        template<typename type> static constexpr bool check_input_arr = std::is_same_v<type, arr_value>;
    }


    class aerosandbox_result_container {
    public:
        aerosandbox_result_container () : float_container(), array_container() {}

        template<typename inputType> requires(check_input_f<inputType> || check_input_arr<inputType>)
        void input_data (enum_type KEY, const inputType &IN_DATA) noexcept {
            if (KEY < 6)
                this->array_container[KEY] = IN_DATA;
            else
                this->float_container[static_cast<float_container_type::size_type>(KEY) - 6] = IN_DATA;
        }

        template<typename inputType> requires(check_input_f<inputType> || check_input_arr<inputType>)
        void input_data (const std::size_t &IN_KEY, const inputType &IN_DATA) noexcept {
            if constexpr (check_input_arr<inputType>)
                this->array_container[IN_KEY] = IN_DATA;
            else
                this->float_container[IN_KEY - 6] = IN_DATA;
        }




        template<typename returnType> requires(std::is_same_v<returnType, arr_value>)
        arr_value_cref operator[] (enum_type KEY) const {
            return this->array_container[KEY];
        }

        template<typename returnType> requires(std::is_same_v<returnType, f_value>)
        f_value_cref operator[] (enum_type KEY) const {
            return this->float_container[static_cast<float_container_type::size_type>(KEY) - 6];
        }

        template<typename returnType> requires(std::is_same_v<returnType, arr_value>)
        arr_value_cref operator[] (const array_container_type::size_type &KEY ) const {
            return this->array_container[KEY];
        }

        template<typename returnType> requires(std::is_same_v<returnType, f_value>)
        f_value_cref operator[] (const float_container_type::size_type &KEY) const {
            return this->float_container[KEY - 6];
        }




        template<typename returnType> requires(std::is_same_v<returnType, arr_value>)
        arr_value_ref operator[] (enum_type KEY) {
            return this->array_container[KEY];
        }

        template<typename returnType> requires(std::is_same_v<returnType, f_value>)
        f_value_ref operator[] (enum_type KEY) {
            return this->float_container[static_cast<float_container_type::size_type>(KEY) - 6];
        }

        template<typename returnType> requires(std::is_same_v<returnType, arr_value>)
        arr_value_ref operator[] (const array_container_type::size_type &KEY ) {
            return this->array_container[KEY];
        }

        template<typename returnType> requires(std::is_same_v<returnType, f_value>)
        f_value_ref operator[] (const float_container_type::size_type &KEY) {
            return this->float_container[KEY - 6];
        }

        void clear () noexcept {
            this->float_container.clear();
            this->array_container.clear();
        }
    private:
        float_container_type float_container;
        array_container_type array_container;
    };
}

#endif //CONCEPTUAL_AEROSANDBOX_RESULTS_CONTAINER_H
