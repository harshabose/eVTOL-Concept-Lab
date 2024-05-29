//
// Created by Harshavardhan Karnati on 14/05/2024.
//

#ifndef CONCEPTUAL_OPERATIONAL_POINT_H
#define CONCEPTUAL_OPERATIONAL_POINT_H

#include <array>
#include <memory>

#include "useful_datatypes/enum_indexed_container.h"

namespace concpt {
    enum operational_params {altitude, temperature_offset, velocity, angle_of_attack, angle_of_sideslip,
            temperature, viscosity, speed_of_sound, pressure, dynamic_pressure, density, OPERATIONAL_LAST};

    class operational_point_build;
    namespace {
        class operational_point {
        friend class concpt::operational_point_build;
        public:
            virtual ~operational_point () = default;

            [[nodiscard]]
            inline
            float
            get(concpt::operational_params NAME) noexcept {
                return this->array_of_params[NAME];
            }

            [[nodiscard]] float calc_dynamic_pressure (const float &IN_VELOCITY) const noexcept {
                return 0.5f * this->array_of_params[concpt::operational_params::density] * (
                        IN_VELOCITY * IN_VELOCITY);
            }

        private:
            std::array<float, concpt::operational_params::OPERATIONAL_LAST> array_of_params{};
            bool added_altitude = false;
            bool added_velocity = false;

            void build_ () {
                this->array_of_params[concpt::operational_params::temperature] = this->get_temperature();
                this->array_of_params[concpt::operational_params::viscosity] = this->get_viscosity();
                this->array_of_params[concpt::operational_params::speed_of_sound] = this->get_speed_of_sound();
                this->array_of_params[concpt::operational_params::pressure] = this->get_pressure();
                this->array_of_params[concpt::operational_params::density] = this->get_density();
                this->array_of_params[concpt::operational_params::dynamic_pressure] = this->get_dynamic_pressure();
            }

            [[nodiscard]] float get_temperature () const noexcept {
                return ((15.0f - 6.5f * (this->array_of_params[concpt::operational_params::altitude] / 1000.0f)) + 273.15f +
                        this->array_of_params[concpt::operational_params::temperature_offset]);
            }

            [[nodiscard]] float get_viscosity () const {
                return 1.48e-06f * (std::pow(this->array_of_params[concpt::operational_params::temperature], 1.5f)) *
                       (1.0f / (this->array_of_params[concpt::operational_params::temperature] + 110.4f));
            }

            [[nodiscard]] float get_speed_of_sound () const {
                return (std::sqrt(concpt::aux::gamma_v<float> * (concpt::aux::R_v<float> / concpt::aux::M_air_v<float>) *
                                  this->array_of_params[concpt::operational_params::temperature]));
            }

            [[nodiscard]] float get_pressure (const float &IN_SEA_LEVEL_PRESSURE = 101325.0f) const {
                return IN_SEA_LEVEL_PRESSURE * std::pow((1.0f - 0.0065f * (this->array_of_params[concpt::operational_params::altitude] /
                                                                           this->array_of_params[concpt::operational_params::temperature])), 5.2561f);
            }

            [[nodiscard]] float get_density () const noexcept {
                return (1.225f * std::pow((1.0f - 22.558e-6f * this->array_of_params[concpt::operational_params::altitude]), 4.2559f));
            }

            [[nodiscard]] float get_dynamic_pressure () const noexcept {
                return 0.5f * this->array_of_params[concpt::operational_params::density] * (
                        this->array_of_params[concpt::operational_params::velocity] *
                        this->array_of_params[concpt::operational_params::velocity]);
            }
        };
    }   //end of (anonymous) namespace

    class operational_point_build {
    public:
        operational_point_build () : atmosphere(std::make_shared<concpt::operational_point>()){}

        [[nodiscard]]
        inline
        concpt::operational_point_build&
        add_param(concpt::operational_params NAME, const float &IN_VALUE) {
            if (NAME == concpt::operational_params::altitude)
                this->atmosphere->added_altitude = true;
            if (NAME == concpt::operational_params::velocity)
                this->atmosphere->added_velocity = true;

            this->atmosphere->array_of_params[NAME] = IN_VALUE;
            return *this;
        }

        [[nodiscard]]
        inline
        concpt::operational_point
        build () {
            this->atmosphere->build_();
            return *this->atmosphere;
        }

        [[nodiscard]]
        inline
        std::shared_ptr<concpt::operational_point>
        build_shared () {
            this->atmosphere->build_();
            return this->atmosphere;
        }
    private:
        std::shared_ptr<concpt::operational_point> atmosphere;
    };
    
}
#endif //CONCEPTUAL_OPERATIONAL_POINT_H
