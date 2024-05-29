//
// Created by Harshavardhan Karnati on 25/05/2024.
//

#ifndef CONCEPTUAL_ENUM_METHOD_CALLTRACKER_H
#define CONCEPTUAL_ENUM_METHOD_CALLTRACKER_H

#include <iostream>
#include <vector>
#include <type_traits>

namespace concpt::declare {
    namespace {
        template <typename enumType, std::size_t size>
        class method_call_tracker_impl {
            method_call_tracker_impl () {
                this->bools.resize(size);
            }

            void mark_called(enumType METHOD_FLAG) noexcept {
                this->bools[static_cast<std::size_t>(METHOD_FLAG)] = true;
            }

            bool is_called(enumType METHOD_FLAG) noexcept {
                return this->bools[static_cast<std::size_t>(METHOD_FLAG)];
            }
        private:
            std::vector<bool> bools;
        };
    }

    template<typename enumType, std::size_t size>
    using enum_method_call_tracker_t = concpt::declare::method_call_tracker_impl<enumType, size>;

    template<auto size> requires (std::is_enum_v<decltype(size)>)
    auto enum_method_call_tracker () {
        return enum_method_call_tracker_t<decltype(size), static_cast<std::size_t>(size)>();
    }
}

#endif //CONCEPTUAL_ENUM_METHOD_CALLTRACKER_H
