//
// Created by Harshavardhan Karnati on 23/05/2024.
//

#ifndef CONCEPTUAL_ENUM_INDEXED_CONTAINER_H
#define CONCEPTUAL_ENUM_INDEXED_CONTAINER_H

#include <iostream>
#include <concepts>
#include <type_traits>

namespace concpt::declare {

    namespace {

        template<typename type_>
        requires(std::is_enum_v<type_>)
        struct enum_type;

        template<typename type_>
        requires(std::is_enum_v<type_>)
        struct enum_type {
            using type = type_;
        };


        template<typename enumType, typename type, std::size_t size_> requires (std::is_enum_v<enumType>)
        class enum_indexed_container_impl {
        public:
            using enum_type = enumType;
            using value_type = type;
            using size_type = std::size_t;
            using value_ref = type&;
            using value_cref = const type&;
            static constexpr size_type size = size_;


            enum_indexed_container_impl () : container() {}
            virtual ~enum_indexed_container_impl () = default;

            virtual value_ref operator[] (enum_type INDEX) noexcept {
                return this->container[static_cast<size_type>(INDEX)];
            }

            virtual value_cref operator[] (enum_type INDEX) const noexcept {
                return this->container[static_cast<size_type>(INDEX)];
            }

            virtual value_ref operator[] (const size_type INDEX) noexcept {
                return this->container[static_cast<size_type>(INDEX)];
            }

            virtual value_cref operator[] (const size_type INDEX) const noexcept {
                return this->container[static_cast<size_type>(INDEX)];
            }

            virtual void clear () noexcept {
                this->container = std::array<type, size>{};
            }
        protected:
            std::array<type, size_> container{};
        };
    }


    template <typename enumType, typename type, std::size_t size_>
    using enum_indexed_container_type = concpt::declare::enum_indexed_container_impl<enumType, type, size_>;


    template <typename enumType, typename type, std::size_t size_>
    enum_indexed_container_type<enumType, type, size_>
    enum_indexed_container () {
        return concpt::declare::enum_indexed_container_impl<enumType, type, size_>();
    }

    template<typename dataType, auto size> requires (std::is_enum_v<decltype(size)>)
    auto enum_indexed_container () {
        return concpt::declare::enum_indexed_container_impl<decltype(size), dataType, static_cast<std::size_t>(size)>();
    }


}

#endif //CONCEPTUAL_ENUM_INDEXED_CONTAINER_H
