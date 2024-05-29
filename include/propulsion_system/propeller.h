//
// Created by Harshavardhan Karnati on 04/04/2024.
//

#ifndef CONCEPTUAL_PROPELLER_H
#define CONCEPTUAL_PROPELLER_H

#include <iostream>
#include <utility>
#include <vector>
#include <array>
#include <string>
#include <memory>
#include <algorithm>
#include <utility>
#include <tuple>
#include <numbers>
#include <type_traits>
#include <cmath>
#include <complex>
#include <map>

#include "../nlopt/nlopt.h"
#include "../boost_1_84_0/boost/math/special_functions/bessel.hpp"

#include "propulsion_system_new.h"
#include "../airfoil.h"
#include "../operationalPoint.h"
#include "../unsupported/meta_checks.h"
#include "../unsupported/useful_expressions.h"
#include "../unsupported/useful_data_types.h"

namespace concpt {
    namespace declare {
        enum control_variable {PITCH, RPM, BOTH};

        enum class propeller_parameter_types {
            RADIUS,
            CHORD,
            RPM,
            PITCH,
            ERROR
            //add more
        };

        struct bld_sec_data {
        public:
            friend struct blade_details;
            template<typename locType, typename airfoilNameType, typename chordType, typename twistType, typename sweepType, typename offsetType>
            requires (meta_checks::is_number_v<locType> && meta_checks::is_string_v<airfoilNameType> && meta_checks::is_number_v<chordType> &&
                      meta_checks::is_number_v<twistType> && meta_checks::is_number_v<sweepType> && meta_checks::is_number_v<offsetType>)
            bld_sec_data (locType&& IN_LOCATION, airfoilNameType&& IN_AIRFOIL_NAME, chordType&& IN_CHORD,
                          twistType&& IN_TWIST, sweepType&& IN_SWEEP_ALIGNMENT, offsetType&& IN_OFFSET,
                          const std::shared_ptr<concpt::airfoil_polar> &IN_POLARS = nullptr)
            : location(std::forward<locType>(IN_LOCATION)), airfoil_name(std::forward<airfoilNameType>(IN_AIRFOIL_NAME)),
            chord(std::forward<chordType>(IN_CHORD)), twist(std::forward<twistType>(IN_TWIST)),
            mid_chord_alignment(std::forward<sweepType>(IN_SWEEP_ALIGNMENT)), face_offset(std::forward<offsetType>(IN_OFFSET)),
            airfoil_polars(IN_POLARS){}

            bld_sec_data(bld_sec_data&& other) noexcept :
                location(other.location),
                airfoil_name(std::move(other.airfoil_name)),
                chord(other.chord),
                twist(other.twist),
                mid_chord_alignment(other.mid_chord_alignment),
                face_offset(other.face_offset),
                airfoil_polars(std::move(other.airfoil_polars))
            {
                    other.airfoil_polars.reset();
            }

        private:
            float location;
            std::string airfoil_name;
            float chord;
            float twist;
            float mid_chord_alignment;
            float face_offset;

            std::shared_ptr<concpt::airfoil_polar>airfoil_polars = nullptr;
        };

        struct blade_details : public std::enable_shared_from_this<blade_details> {
            // root-----------tip
            // |----|--|---|----|
            // 0----1--2---3----4

            blade_details () = default;
            virtual ~blade_details () = default;

            template<class... sectionalData>
            requires (std::is_same_v<meta_checks::remove_all_qual<sectionalData>, concpt::declare::bld_sec_data> && ...)
            explicit blade_details (const std::size_t &IN_NUMBER_OF_BLADES, sectionalData&&... IN_BLADE_SECTIONAL_DATA)
                    : number_of_blades(IN_NUMBER_OF_BLADES) {
                this->airfoil_locations = {std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).location ...};
                if (!std::is_sorted(this->airfoil_locations.cbegin(), this->airfoil_locations.cend())) {
                    throw std::runtime_error("sectional data needs to span from root to tip");
                }
                this->radius = this->airfoil_locations.back();
                this->chord = {std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).chord ...};
                this->twist = {std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).twist ...};
                this->sweep = {std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).mid_chord_alignment ...};
                this->offset = {std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).face_offset ...};
                this->airfoil_names = {std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).airfoil_name ...};

                ([&]{
                    if (std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).airfoil_polars)
                        this->airfoil_polars = std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).airfoil_polars;
                }(),...);

                if (!this->airfoil_polars)
                    this->airfoil_polars = std::make_shared<concpt::airfoil_polar>(
                            this->airfoil_names,
                            true, true);
            }

            template<class sectionalData>
            requires (std::is_same_v<meta_checks::remove_all_qual<sectionalData>, concpt::declare::bld_sec_data>)
            void add_blade_details (sectionalData&& IN_BLADE_SECTIONAL_DATA) {
                const std::size_t index = this->find_blade_index(std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).location);
                this->airfoil_locations.insert(std::ranges::next(this->airfoil_locations.begin(), index + 1, this->airfoil_locations.end()),
                                               std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).location);
                this->chord.insert(std::ranges::next(this->chord.begin(), index + 1, this->chord.end()),
                                   std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).chord);
                this->twist.insert(std::ranges::next(this->twist.begin(), index + 1, this->twist.end()),
                                   std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).twist);
                this->sweep.insert(std::ranges::next(this->sweep.begin(), index + 1, this->sweep.end()),
                                   std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).mid_chord_alignment);
                this->offset.insert(std::ranges::next(this->offset.begin(), index + 1, this->offset.end()),
                                   std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).face_offset);
                this->airfoil_names.insert(std::ranges::next(this->airfoil_names.begin(), index + 1, this->airfoil_names.end()),
                                           std::forward<sectionalData>(IN_BLADE_SECTIONAL_DATA).airfoil_name);
                this->airfoil_polars->add_airfoils(this->airfoil_names[index + 1]);
            }


            blade_details(concpt::declare::blade_details &&other) noexcept
            :   radius(other.radius), number_of_blades(other.number_of_blades),
                mesh_radius(other.mesh_radius), mesh_azimuthal(other.mesh_azimuthal),
                chord(std::move(other.chord)), sweep(std::move(other.sweep)),
                offset(std::move(other.offset)), twist(std::move(other.twist)),
                airfoil_locations(std::move(other.airfoil_locations)),
                airfoil_polars(std::move(other.airfoil_polars)) {
                this->radius = other.radius;
                this->airfoil_polars.reset();
            }

            std::shared_ptr<blade_details> get_shared_ptr() {
                return shared_from_this();
            }

            float get_root_chord () const noexcept {
                return this->chord.front();
            }

            std::tuple<std::string, float, float, float, float> get_sectional_data (const float IN_BLADE_LOCATION) {
                auto linear_operator = []
                        (const float &Y0, const float &Y1, const float &X0, const float &X1, const float &IN_NEW_X) ->float {
                    const float slope = (Y0 - Y1) / (X0 - X1);
                    const float intercept = Y0 - slope * X0;
                    return slope * IN_NEW_X + intercept;
                };

                const std::size_t index = this->find_blade_index(IN_BLADE_LOCATION);
                const std::string current_airfoil = this->airfoil_names[index];

                const float current_chord = linear_operator(this->chord.at(index), this->chord.at(index + 1),
                                                      this->airfoil_locations.at(index), this->airfoil_locations.at(index + 1), IN_BLADE_LOCATION);

                const float current_twist = linear_operator(this->twist.at(index), this->twist.at(index + 1),
                                                      this->airfoil_locations.at(index), this->airfoil_locations.at(index + 1), IN_BLADE_LOCATION);

                const float current_sweep = linear_operator(this->sweep.at(index), this->sweep.at(index + 1),
                                                            this->airfoil_locations.at(index), this->airfoil_locations.at(index + 1), IN_BLADE_LOCATION);

                const float current_offset = linear_operator(this->offset.at(index), this->offset.at(index + 1),
                                                             this->airfoil_locations.at(index), this->airfoil_locations.at(index + 1), IN_BLADE_LOCATION);

                return std::make_tuple(current_airfoil, current_chord, current_twist, current_sweep, current_offset);
            }

            void set_mesh_parameters (const std::size_t &IN_MESH_RADIUS, const std::size_t &IN_MESH_AZIMUTHAL) noexcept {
                this->mesh_radius = IN_MESH_RADIUS;
                this->mesh_azimuthal = IN_MESH_AZIMUTHAL;
            }

            std::pair<float, float> get_aero_values (const float &BLADE_LOCATION, const float &IN_ALPHA, const float &IN_RE, const float &IN_MACH, const float &IN_MULTIPLIER = 1.0f) {
                std::string current_airfoil = this->airfoil_names[this->find_blade_index(BLADE_LOCATION)];
                if (IN_RE < this->airfoil_polars->surrogate_hash_map.at(current_airfoil).min_Re
                    || IN_RE > this->airfoil_polars->surrogate_hash_map.at(current_airfoil).max_Re) {
                    throw std::runtime_error("Reynolds number out-of-data..");
                }
                return this->airfoil_polars->get_aero_values(current_airfoil, IN_ALPHA, IN_RE, IN_MACH, IN_MULTIPLIER);
            }

            std::pair<std::vector<std::pair<float, float>>*, std::vector<std::pair<float, float>>*>
            get_airfoil_coordinates (const float &IN_BLADE_LOCATION) {
                std::string current_airfoil = this->airfoil_names[this->find_blade_index(IN_BLADE_LOCATION)];
                if (this->airfoil_polars->surrogate_hash_map.at(current_airfoil).airfoil_coordinates_upper.empty() ||
                        this->airfoil_polars->surrogate_hash_map.at(current_airfoil).airfoil_coordinates_lower.empty()) {
                    throw std::runtime_error("Airfoil coordinates are not available...");
                }
                return std::make_pair(&(this->airfoil_polars->surrogate_hash_map.at(current_airfoil).airfoil_coordinates_upper), &(this->airfoil_polars->surrogate_hash_map.at(current_airfoil).airfoil_coordinates_upper));
            }

            std::pair<std::vector<std::pair<float, float>>, std::vector<std::pair<float, float>>>
            get_cl_cd_chord_distribution (const float &IN_BLADE_LOCATION, const float &IN_ALPHA = 0.0f, const float &IN_RE = 0.0f, const float &IN_MACH_NUMBER = 0.0f) {
                std::string current_airfoil = this->airfoil_names[this->find_blade_index(IN_BLADE_LOCATION)];
                auto cl_cd_dis = std::make_pair(this->airfoil_polars->surrogate_hash_map.at(current_airfoil).airfoil_coordinates_upper,
                                                   this->airfoil_polars->surrogate_hash_map.at(current_airfoil).airfoil_coordinates_lower);

                // TODO: currently implies uniform distribution for lift and drag. (lift assumption is wrong)
                std::for_each(cl_cd_dis.first.begin(), cl_cd_dis.first.end(), [](auto& pair) { pair.second = 1.0f;});
                std::for_each(cl_cd_dis.second.begin(), cl_cd_dis.second.end(), [](auto& pair) { pair.second = 1.0f;});

                return cl_cd_dis;
            }

            float get_max_thickness_ratio (const float &IN_BLADE_LOCATION) {
                std::string current_airfoil = this->airfoil_names[this->find_blade_index(IN_BLADE_LOCATION)];
                const float max_thickness_ratio = this->airfoil_polars->surrogate_hash_map.at(current_airfoil).max_thickness_ratio;
                if (concpt::aux::check_equal(max_thickness_ratio, 0.0f)) {
                    throw std::runtime_error("No Thickness ratio value exists");
                } else {
                    return max_thickness_ratio;
                }
            }

            float radius{};
            std::size_t number_of_blades{};
            std::size_t mesh_radius = 250, mesh_azimuthal = 1;
        private:

            [[nodiscard]] std::size_t find_blade_index (const float &IN_BLADE_LOCATION) {
                auto if_ahead = [&IN_BLADE_LOCATION] (const float &IN_LOCATION) -> bool {return IN_LOCATION >= IN_BLADE_LOCATION;};
                return std::ranges::distance(airfoil_locations.begin(),
                                             std::ranges::next(std::ranges::find_if(this->airfoil_locations.begin(),
                                                                                    airfoil_locations.end(),
                                                                                    if_ahead),
                                                               -1
                                             )
                );
            }

            std::vector<float>chord, sweep, offset, twist;
            std::vector<float> airfoil_locations = {};
            std::vector<std::string> airfoil_names = {};
            std::shared_ptr<concpt::airfoil_polar>airfoil_polars = nullptr;
        };

        struct propeller_controller_settings : public std::enable_shared_from_this<propeller_controller_settings> {
            propeller_controller_settings() = default;
            virtual ~propeller_controller_settings () = default;

            propeller_controller_settings (const control_variable &CONTROLLER, const float &IN_RPM, const float &IN_PITCH,
                                           const std::array<double, 2> &IN_LOWER_BOUNDS = {0.0, 0.0},
                                           const std::array<double, 2> &IN_UPPER_BOUNDS = {100.0, 90.0})
                    : controller(CONTROLLER), initial_RPM(IN_RPM), initial_pitch(IN_PITCH) {
                if (this->controller == control_variable::BOTH) this->number_of_controllers = 2;
                this->set_up_bounds(IN_LOWER_BOUNDS, IN_UPPER_BOUNDS);
            }
            std::shared_ptr<propeller_controller_settings> get_shared_ptr() {
                return shared_from_this();
            }

            void set_up_bounds (const std::array<double, 2> &IN_LOWER_BOUNDS, const std::array<double, 2> &IN_UPPER_BOUNDS) {
                this->lower_bounds = IN_LOWER_BOUNDS;
                this->upper_bounds = IN_UPPER_BOUNDS;
                if (this->controller == control_variable::PITCH) {
                    std::reverse(this->lower_bounds.begin(), this->lower_bounds.end());
                    std::reverse(this->upper_bounds.begin(), this->upper_bounds.end());
                }
            }

            void update_controller_value (const double *IN_NEW_VALUES) {
                if (this->controller == control_variable::RPM) this->initial_RPM = static_cast<float>(IN_NEW_VALUES[0]);
                else if (this->controller == control_variable::PITCH) initial_pitch = static_cast<float>(IN_NEW_VALUES[0]);
                else {
                    this->initial_RPM = static_cast<float>(IN_NEW_VALUES[0]);
                    this->initial_pitch = static_cast<float>(IN_NEW_VALUES[1]);
                }
            }

            void set_up_nlopt_control_variables (double *IN_NLOPT_VAR) const noexcept {
                if (this->controller == control_variable::PITCH) IN_NLOPT_VAR[0] = static_cast<double>(this->initial_pitch);
                else {
                    IN_NLOPT_VAR[0] = static_cast<double>(this->initial_RPM);
                    IN_NLOPT_VAR[1] = static_cast<double>(this->initial_pitch);
                }
            }

            void reset_RPM_pitch () {
                if (this->controller == control_variable::PITCH) this->initial_pitch = 0.0f;
                else if (this->controller == control_variable::RPM) this->initial_RPM = 0.0f;
                else if (this->controller == control_variable::BOTH) {
                    this->initial_RPM = 0.0f;
                    this->initial_pitch = 0.0f;
                }
            }

            control_variable controller = PITCH;
            std::size_t number_of_controllers = 1;
            float initial_RPM{};
            float initial_pitch{};
            std::array<double, 2> lower_bounds = {0.0, 0.0};
            std::array<double, 2> upper_bounds = {0.0, 0.0};
        };

        struct detailed_output {
            enum output_type {
                VELOCITY,
                FORCE,
                ACOUSTIC
            };
            detailed_output () = default;
            virtual ~detailed_output () = default;

            explicit detailed_output (const std::shared_ptr<concpt::declare::blade_details> &IN_BLADE_SECTION)
            : blade_section(IN_BLADE_SECTION){
                this->velocity_field.reserve(this->blade_section->mesh_radius * this->blade_section->mesh_azimuthal);
                this->force_field.reserve(this->blade_section->mesh_radius * this->blade_section->mesh_azimuthal);
                this->acoustic_field.reserve(50 * this->blade_section->mesh_radius * this->blade_section->mesh_azimuthal);
            }

            void add_velocity_data_point (const std::array<float, 3> &IN_VELOCITY_DATA) {
                this->velocity_field.push_back(IN_VELOCITY_DATA);
            }

            void add_angle_of_attack_data_point (const float &IN_AOA) {
                this->angle_of_attack_field.push_back(IN_AOA);
            }

            void add_force_data_point (const std::array<float, 3> &IN_FORCE_DATA) {
                this->force_field.push_back(IN_FORCE_DATA);
            }

            void add_acoustic_data_point (const std::complex<float> &IN_ACOUSTIC_DATA) {
                this->acoustic_field.push_back(IN_ACOUSTIC_DATA);
            }

            void velocity_field_reset () noexcept {
                this->velocity_field.clear();
            }

            void angle_of_attak_reset () noexcept {
                this->angle_of_attack_field.clear();
            }

            void force_field_reset () noexcept {
                this->force_field.clear();
            }

            void acoustic_field_reset () noexcept {
                this->acoustic_field.clear();
            }

            void print_velocity_field (const std::string &IN_JSON_PATH = "") {
                if (IN_JSON_PATH.empty()) {
                    std::cout << "\n\n" << "VELOCITY-X: " << std::endl;
                    for (const auto &element : this->velocity_field) {
                        std::cout << element[0] << std::endl;
                    }

                    std::cout << "\n\n" << "VELOCITY-Y: " << std::endl;
                    for (const auto &element : this->velocity_field) {
                        std::cout << element[1] << std::endl;
                    }

                    std::cout << "\n\n" << "VELOCITY-Z: " << std::endl;
                    for (const auto &element : this->velocity_field) {
                        std::cout << element[2] << std::endl;
                    }
                } else {
                    std::ofstream json_file(IN_JSON_PATH);
                    if(!json_file.is_open()) {
                        throw std::runtime_error("Could not open file for writing: " + IN_JSON_PATH);
                    }

                    nlohmann::json json_data;
                    json_data["VELOCITY-X"] = nlohmann::json::array();
                    json_data["VELOCITY-Y"] = nlohmann::json::array();
                    json_data["VELOCITY-Z"] = nlohmann::json::array();

                    try {
                        for (const auto &element : this->velocity_field) {
                            json_data["VELOCITY-X"].push_back(element[0]);
                            json_data["VELOCITY-Y"].push_back(element[1]);
                            json_data["VELOCITY-Z"].push_back(element[2]);
                        }
                    } catch (std::exception &e) {
                        std::cerr << "ERROR while storing data in json object" << std::endl;
                    }

                    std::cout << "Writing to " << IN_JSON_PATH  << std::endl;
                    json_file << json_data.dump(4);
                    json_file.close();
                }
            }

            void print_force_field (const std::string &IN_JSON_PATH = "") {
                if (IN_JSON_PATH.empty()) {
                    std::cout << "\n\n" << "FORCE-X: " << std::endl;
                    for (const auto &element : this->force_field) {
                        std::cout << element[0] << std::endl;
                    }

                    std::cout << "\n\n" << "FORCE-Y: " << std::endl;
                    for (const auto &element : this->force_field) {
                        std::cout << element[1] << std::endl;
                    }

                    std::cout << "\n\n" << "FORCE-Z: " << std::endl;
                    for (const auto &element : this->force_field) {
                        std::cout << element[2] << std::endl;
                    }
                } else {
                    std::ofstream json_file(IN_JSON_PATH);
                    if(!json_file.is_open()) {
                        throw std::runtime_error("Could not open file for writing: " + IN_JSON_PATH);
                    }

                    nlohmann::json json_data;
                    json_data["FORCE-X"] = nlohmann::json::array();
                    json_data["FORCE-Y"] = nlohmann::json::array();
                    json_data["FORCE-Z"] = nlohmann::json::array();

                    try {
                        for (const auto &element : this->velocity_field) {
                            json_data["FORCE-X"].push_back(element[0]);
                            json_data["FORCE-Y"].push_back(element[1]);
                            json_data["FORCE-Z"].push_back(element[2]);
                        }
                    } catch (std::exception &e) {
                        std::cerr << "ERROR while storing data in json object" << std::endl;
                    }

                    std::cout << "Writing to " << IN_JSON_PATH  << std::endl;
                    json_file << json_data.dump(4);
                    json_file.close();
                }
            }

            void print_acoustic_field (const std::string &IN_JSON_PATH = "") {
                if (IN_JSON_PATH.empty()) {
                    std::cout << "\n\n" << "ACOUSTIC-REAL: " << std::endl;
                    for (const std::complex<float> &element : this->acoustic_field) {
                        std::cout << element.real() << std::endl;
                    }

                    std::cout << "\n\n" << "ACOUSTIC-IMAG: " << std::endl;
                    for (const std::complex<float> &element : this->acoustic_field) {
                        std::cout << element.imag() << std::endl;
                    }
                } else {
                    std::ofstream json_file(IN_JSON_PATH);
                    if(!json_file.is_open()) {
                        throw std::runtime_error("Could not open file for writing: " + IN_JSON_PATH);
                    }

                    nlohmann::json json_data;
                    json_data["ACOUSTIC-REAL"] = nlohmann::json::array();
                    json_data["ACOUSTIC-IMAG"] = nlohmann::json::array();

                    try {
                        for (const std::complex<float> &element : this->acoustic_field) {
                            json_data["ACOUSTIC-REAL"].push_back(element.real());
                            json_data["ACOUSTIC-IMAG"].push_back(element.imag());
                        }
                    } catch (std::exception &e) {
                        std::cerr << "ERROR while storing data in json object" << std::endl;
                    }

                    std::cout << "Writing to " << IN_JSON_PATH  << std::endl;
                    json_file << json_data.dump(4);
                    json_file.close();
                }
            }

            void print_angle_of_attack_field (const std::string &IN_JSON_PATH = "") {
                if (IN_JSON_PATH.empty()) {
                    std::cout << "\n\n" << "AOA: " << std::endl;
                    for (const auto &element : this->angle_of_attack_field) {
                        std::cout << element << std::endl;
                    }

                } else {
                    std::ofstream json_file(IN_JSON_PATH);
                    if(!json_file.is_open()) {
                        throw std::runtime_error("Could not open file for writing: " + IN_JSON_PATH);
                    }

                    nlohmann::json json_data;
                    json_data["AOA"] = nlohmann::json::array();

                    try {
                        for (const float &element : this->angle_of_attack_field) {
                            json_data["AOA"].push_back(element);
                        }
                    } catch (std::exception &e) {
                        std::cerr << "ERROR while storing data in json object" << std::endl;
                    }

                    std::cout << "Writing to " << IN_JSON_PATH  << std::endl;
                    json_file << json_data.dump(4);
                    json_file.close();
                }
            }

        private:
            std::vector<std::array<float, 3>> velocity_field;
            std::vector<std::array<float, 3>> force_field;
            std::vector<std::complex<float>> acoustic_field;
            std::vector<float> angle_of_attack_field;

            std::shared_ptr<concpt::declare::blade_details> blade_section;

            [[nodiscard]] std::size_t get_index (const float &IN_RADIUS, const float &IN_THETA) const {
                std::size_t radius_index = std::min(static_cast<size_t>(IN_RADIUS / this->blade_section->radius) * this->blade_section->mesh_radius, this->blade_section->mesh_radius - 1);
                std::size_t theta_index = std::min(static_cast<size_t>(IN_THETA / (2.0f * std::numbers::pi_v<float>)) * this->blade_section->mesh_azimuthal, this->blade_section->mesh_azimuthal - 1);
                return radius_index * this->blade_section->mesh_azimuthal + theta_index;
            }
        };
    }

    class propeller_acoustics : public std::enable_shared_from_this<propeller_acoustics> {
    public:

        propeller_acoustics () = default;

        virtual ~propeller_acoustics () = default;

        explicit propeller_acoustics (const std::shared_ptr<concpt::propeller> &IN_PROPELLER, const float &IN_OBSERVER_DISTANCE, const float &IN_ELEVATION);

        std::array<std::complex<float>, 3> get_psi_factor (const float &IN_BLADE_LOCATION, const float &IN_KX);

        std::array<float, 4> get_k_and_phase_factor (const float &IN_BLADE_LOCATION, const std::size_t &IN_HARMONICS,
                                                     const std::size_t &IN_NUM_BLADES, const float &IN_RADIUS, const float &IN_SECTIONAL_CHORD,
                                                     const float &IN_SECTIONAL_SWEEP, const float &IN_SECTIONAL_OFFSET,
                                                     const float &IN_AXIAL_MACH, const float &IN_TANGENTIAL_MACH) const;

        std::complex<float> get_forward_factor(const std::size_t &IN_HARMONICS, const float &IN_AXIAL_VELOCITY);

        float get_bessel_at(const float &IN_BLADE_LOCATION, const std::size_t &IN_HARMONICS,
                            const std::size_t &IN_NUM_BLADES, const float &IN_RADIUS,
                            const float &IN_AXIAL_MACH, const float &IN_TANGENTIAL_MACH) const;

        std::complex<float> get_sectional_acoustics (const float &IN_BLADE_LOCATION, const std::size_t &IN_HARMONICS,
                                                     const float &IN_SECTIONAL_CHORD, const float &IN_SECTIONAL_SWEEP, const float &IN_SECTIONAL_OFFSET,
                                                     const float &IN_AXIAL_VELOCITY, const float &IN_TANGENTIAL_VELOCITY,
                                                     const float &IN_CL, const float &IN_CD, const float &IN_DX);

        static std::complex<float> get_euler_equation(const float &IN_THETA) noexcept;

    private:
        std::weak_ptr<concpt::propeller> propeller;
        float elevation_angle_rads{}, elevation_angle{}, observer_distance{};
    };


    class propeller : public std::enable_shared_from_this<propeller> {
    public:
        friend class propeller_acoustics;
        friend class concpt::propeller_weight;
        propeller () = default;
        virtual ~propeller () = default;

        static std::shared_ptr<propeller> create_shared_ptr_inline (const std::shared_ptr<concpt::declare::blade_details> &IN_BLADE,
                                                         const std::shared_ptr<concpt::declare::propeller_controller_settings> &IN_CONTROLLER) {
            std::shared_ptr<propeller> return_ptr = std::make_shared<propeller>();
            return_ptr->add_sectional_airfoil_details(IN_BLADE);
            return_ptr->add_controllers(IN_CONTROLLER);
            return return_ptr;
        }

        void add_propulsion_system (const std::shared_ptr<concpt::propulsion_system> &IN_PROPULSION_SYSTEM) noexcept {
            this->propulsion_system = IN_PROPULSION_SYSTEM;
        }

        void add_sectional_airfoil_details (const std::shared_ptr<concpt::declare::blade_details> &IN_SECTIONAL_AIRFOIL) {
            this->blade_section = IN_SECTIONAL_AIRFOIL;
        }

        void add_controllers (const std::shared_ptr<concpt::declare::propeller_controller_settings> &IN_CONTROLLER) {
            this->controller_settings = IN_CONTROLLER;
        }

        void set_thrust_required (const float &IN_THRUST, const float &IN_PROP_PLANE_ANGLE = 0.0f) noexcept {
            this->thrust_required = IN_THRUST;
            this->propeller_plane_angle = concpt::aux::check_equal(IN_PROP_PLANE_ANGLE, 0.0f) ? this->propeller_plane_angle : IN_PROP_PLANE_ANGLE;
        }

        template<concpt::declare::propeller_parameter_types parameterName>
        inline auto getter () const {
            if constexpr (parameterName == concpt::declare::propeller_parameter_types::RADIUS) return this->blade_section->radius;
            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::CHORD) return this->blade_section->get_root_chord();
            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::RPM) return this->controller_settings->initial_RPM;
            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::PITCH) return this->controller_settings->initial_pitch;
            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::ERROR) {
                return std::abs(this->thrust_coefficient_obtained - this->thrust_coefficient) / this->thrust_coefficient;
            }
            else {
                throw std::runtime_error("Invalid parameterName");
                //compile-time static_assert won't work, I need to re-check all conditions in static_assert (code duplication)
            }
        }

        template<concpt::declare::propeller_parameter_types parameterName>
        void setter (const auto &IN_VALUE) const {
            if constexpr (parameterName == concpt::declare::propeller_parameter_types::RADIUS) this->blade_section->radius = IN_VALUE;
//            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::CHORD) this->blade_section->root_chord = IN_VALUE;
            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::RPM) this->controller_settings->initial_RPM = IN_VALUE;
            else if constexpr (parameterName == concpt::declare::propeller_parameter_types::PITCH) this->controller_settings->initial_pitch = IN_VALUE;
            else {
                throw std::runtime_error("Invalid parameterName");
                //compile-time static_assert won't work, I need to re-check all conditions in static_assert (code duplication)
            }
        }

        std::shared_ptr<propeller> get_shared_ptr() {
            if (!this->blade_section && this->propulsion_system.expired() && !this->controller_settings) throw std::runtime_error("shared_ptr(s) are null pointing...");
            return shared_from_this();
        }

        float solve_BEMT (const float &IN_THRUST, const float &IN_PROP_PLANE_ANGLE = 0.0f) {
            this->set_thrust_required(IN_THRUST, IN_PROP_PLANE_ANGLE);
            return this->solve_BEMT();
        }

        float solve_BEMT (const std::shared_ptr<concpt::declare::detailed_output> &IN_OUTPUT = nullptr, const std::string &IN_MESSAGE = "") {
            this->detailed_output = IN_OUTPUT;
            if (!this->blade_section || !this->controller_settings|| this->propulsion_system.expired()) {
                throw std::runtime_error("propeller class not built properly");
            }

            if (concpt::aux::check_equal(this->thrust_required, 0.0f)) return 0.0f;

            double control_variables_[2];
            this->controller_settings->set_up_nlopt_control_variables(control_variables_);
            this->controller_settings->reset_RPM_pitch();

            nlopt_opt solver = nlopt_create(NLOPT_LN_COBYLA, this->controller_settings->number_of_controllers);
            nlopt_set_lower_bounds(solver, this->controller_settings->lower_bounds.data());
            nlopt_set_upper_bounds(solver, this->controller_settings->upper_bounds.data());

            nlopt_set_min_objective(solver, concpt::propeller::nlopt_objective_function, this);
//            nlopt_set_min_objective(solver, concpt::propeller::nlopt_equality_constraint, this);
            nlopt_add_equality_constraint(solver, concpt::propeller::nlopt_equality_constraint, this, 1e-8);

            nlopt_set_xtol_rel(solver, 1e-5);
            nlopt_set_maxeval(solver, 100);
            double min_power;

            try {
                nlopt_optimize(solver, control_variables_, &min_power);
            } catch (std::exception &e) {
//                std::cerr << IN_MESSAGE << " NLOPT FAILED: " << e.what();
//                std::cerr << "Declaring Power and Thrust coeff obtained to max<float>..." << std::endl;
                this->power_required = std::numeric_limits<float>::max();
                this->thrust_coefficient_obtained = std::numeric_limits<float>::max();
            }

            nlopt_destroy(solver);

            return this->power_required;
        }

        void calculate_propeller_acoustics (const float &IN_OBSERVER_DISTANCE, const float &IN_ELEVATION_ANGLE, const float &IN_ROOT_CUT_OFF = 0.1f) {
            concpt::propeller_acoustics acoustics = concpt::propeller_acoustics(shared_from_this(), IN_OBSERVER_DISTANCE, IN_ELEVATION_ANGLE);
            if (this->detailed_output) {
                this->detailed_output->acoustic_field_reset();
            }
            try {
                float speed_of_sound, inflow_velocity, forward_velocity;
                std::tie(std::ignore, speed_of_sound) = this->set_BEMT_parameters(IN_ROOT_CUT_OFF);
                std::tie(inflow_velocity, std::ignore, forward_velocity) = this->get_induced_velocity();

                auto per_harmonic = [&, this] (const std::size_t &IN_HARMONICS) {
                    const std::complex<float> forward_factor = acoustics.get_forward_factor(IN_HARMONICS, inflow_velocity);
                    std::complex<float> total = {0.0f, 0.0f};

                    for (std::size_t i = 0; i < this->blade_section->mesh_azimuthal; i++) {
                        for (std::size_t j = 0; j < this->blade_section->mesh_radius - 1; j++) {
                            const float sectional_span = this->radius_divisions[j+1] - this->radius_divisions[i];
                            // TODO: add 3D angle
                            const float tangential_sectional_velocity = this->radius_divisions[j] * this->controller_settings->initial_RPM + forward_velocity * std::cos(this->psi_angles_divisions[i] * std::numbers::pi_v<float> / 180.0f);
                            const float axial_sectional_velocity = this->get_instantaneous_inflow_velocity(this->radius_divisions[j], this->psi_angles_divisions[i], inflow_velocity, std::atan(forward_velocity / inflow_velocity));
                            [[maybe_unused]] const float tip_velocity = this->blade_section->radius * this->controller_settings->initial_RPM;
                            // TODO: use tip instead of tangential

                            const float sectional_velocity = concpt::aux::get_euclidean_norm(tangential_sectional_velocity, axial_sectional_velocity);
                            const float phi = std::atan(axial_sectional_velocity / tangential_sectional_velocity);

                            float sectional_chord, sectional_pitch, sectional_sweep, sectional_offset;
                            std::tie(std::ignore, sectional_chord, sectional_pitch, sectional_sweep, sectional_offset) = this->blade_section->get_sectional_data(this->radius_divisions[j]);
                            sectional_pitch += this->controller_settings->initial_pitch;

                            const float sectional_AoA = sectional_pitch - (phi * 180.0f / std::numbers::pi_v<float>);
                            const auto [cl, cd] = this->get_blade_section_aero(this->radius_divisions[j], sectional_AoA, sectional_velocity, sectional_chord, sectional_span, true);
                            const std::complex<float> current_acoustic_factors = acoustics.get_sectional_acoustics(this->radius_divisions[j], IN_HARMONICS, sectional_chord,
                                                                                                                   sectional_sweep, sectional_offset,
                                                                                                                   axial_sectional_velocity, tangential_sectional_velocity,
                                                                                                                   std::abs(cl), std::abs(cd), sectional_span);
                            total += current_acoustic_factors;

                            if (this->detailed_output) {
                                this->detailed_output->add_acoustic_data_point(
                                        forward_factor * current_acoustic_factors *
                                        static_cast<float>(this->blade_section->number_of_blades) /
                                        static_cast<float>(this->blade_section->mesh_azimuthal)
                                        );
                            }
                        }
                    }
                    return (forward_factor * total * static_cast<float>(this->blade_section->number_of_blades) / static_cast<float>(this->blade_section->mesh_azimuthal));
                };

                const std::size_t max_harmonic = 50;
                this->acoustic_factors.clear();
                for (std::size_t harmonic = 1; harmonic < max_harmonic; harmonic++) {
                    this->acoustic_factors.push_back(per_harmonic(harmonic));
                }
            } catch (std::exception &e) {
                this->acoustic_factors.emplace_back(0.0f, 0.0f);
            }
        }

        [[nodiscard]] float get_propeller_dB () {
            const float pressure_ref = 2.0e-5f;
            float pressure_rms = 0.0f;

            for(std::size_t harmonics = 1; harmonics < this->acoustic_factors.size() + 1; harmonics++) {
                const float current_frequency = this->controller_settings->initial_RPM *
                        static_cast<float>(harmonics * this->blade_section->number_of_blades) / (2.0f * std::numbers::pi_v<float>);
                if (current_frequency > 20.0f && current_frequency < 20000) {
                    pressure_rms += 2.0f * (concpt::aux::power<2>(this->acoustic_factors[harmonics - 1].real()) +
                            concpt::aux::power<2>(this->acoustic_factors[harmonics - 1].imag()));
                }
            }
            pressure_rms = std::sqrt(pressure_rms);

            if (concpt::aux::check_equal(pressure_rms, 0.0f)) {
                return 0.0f;
            }
            return 20.0f * std::log10(pressure_rms / pressure_ref);
        }


    private:
        std::weak_ptr<concpt::propulsion_system>propulsion_system;
        std::shared_ptr<concpt::declare::propeller_controller_settings>controller_settings = nullptr;
        std::shared_ptr<concpt::declare::blade_details>blade_section = nullptr;
        std::shared_ptr<concpt::declare::detailed_output>detailed_output = nullptr;
        concpt::propeller_weight weight = concpt::propeller_weight(*this);

        std::vector<float>psi_angles_divisions;
        std::vector<float>radius_divisions;
        std::vector<std::complex<float>>acoustic_factors;
        float thrust_required{}, thrust_coefficient{}, max_thrust_coefficient = 2.0f;
        float thrust_coefficient_obtained{}, power_required{}, propeller_plane_angle{};

        [[nodiscard]] std::array<float, 2> set_BEMT_parameters (const float &ROOT_CUT_OFF = 0.1f) {
            this->psi_angles_divisions.clear();
            this->radius_divisions.clear();

            const auto [tip_loss_factor, effective_area, speed_of_sound] = this->calculate_thrust_coefficient(ROOT_CUT_OFF);

            for (std::size_t i = 0; i < this->blade_section->mesh_azimuthal; i++) {
                if (this->blade_section->mesh_azimuthal - 1 != 0) this->psi_angles_divisions.push_back(static_cast<float>(i) * 360.0f / (static_cast<float>(this->blade_section->mesh_azimuthal) - 1.0f));
                else this->psi_angles_divisions.push_back(0.0f);
            }

            if (this->blade_section->mesh_radius < 1) throw std::runtime_error("Number of radius divisions needs to be at least 1");

            for (std::size_t i = 0; i < this->blade_section->mesh_radius; i++) {
                const float root_radius = ROOT_CUT_OFF * this->blade_section->radius;
                const float tip_radius = this->blade_section->radius * tip_loss_factor - root_radius;
                const float increment = static_cast<float>(i) / (static_cast<float>(this->blade_section->mesh_radius) - 1.0f);
                this->radius_divisions.push_back(root_radius + tip_radius * increment);
            }
            return std::array<float, 2>{effective_area, speed_of_sound};
        }

        std::array<float, 3> calculate_thrust_coefficient (const float &ROOT_CUT_OFF) {
            if (std::shared_ptr<concpt::propulsion_system>propulsion_system_ = this->propulsion_system.lock()) {
                float area = std::numbers::pi_v<float> * this->blade_section->radius * this->blade_section->radius;
                this->thrust_coefficient = this->thrust_required / (propulsion_system_->atmosphere->density * area * (this->controller_settings->initial_RPM * this->blade_section->radius) * (this->controller_settings->initial_RPM * this->blade_section->radius));

                float tip_loss_factor = 1.0f - std::sqrt(2.0f * this->thrust_coefficient) / static_cast<float>(this->blade_section->number_of_blades);
                area *= ((tip_loss_factor * tip_loss_factor) - (ROOT_CUT_OFF * ROOT_CUT_OFF));

                this->thrust_coefficient = this->thrust_required / (propulsion_system_->atmosphere->density * area * (this->controller_settings->initial_RPM * this->blade_section->radius) * (this->controller_settings->initial_RPM * this->blade_section->radius));

                if (tip_loss_factor < 0.0f || area < 0.0f || this->thrust_coefficient < 0.0f || this->thrust_coefficient > this->max_thrust_coefficient) {
                    throw std::runtime_error("Negative 'tip loss factor' or propeller area or thrust co-eff encountered... Skipping the iteration...");
                }

                return std::array<float, 3>{tip_loss_factor, area, propulsion_system_->atmosphere->speedOfSound};
            } else {
                throw std::bad_weak_ptr();
            }
        }

        [[nodiscard]] std::tuple<float, float, float> get_induced_velocity () const {
            float advance_velocity, forward_velocity;
            float hover_inflow_sqr = (this->thrust_coefficient / 2.0f) * concpt::aux::power<2>(this->controller_settings->initial_RPM * this->blade_section->radius);
            if (std::shared_ptr<concpt::propulsion_system>propulsion_system_ = this->propulsion_system.lock()) {
                if (concpt::aux::check_equal(propulsion_system_->atmosphere->velocityAoA + this->propeller_plane_angle, 0.0f)) {
                    return std::make_tuple(
                            (propulsion_system_->atmosphere->velocity / 2.0f) + std::sqrt((propulsion_system_->atmosphere->velocity * propulsion_system_->atmosphere->velocity / 4.0f) + hover_inflow_sqr),
                            propulsion_system_->atmosphere->velocity,
                            0.0f);
                }
                advance_velocity = propulsion_system_->atmosphere->velocity * std::cos((propulsion_system_->atmosphere->velocityAoA + this->propeller_plane_angle) * std::numbers::pi_v<float> / 180.0f);
                forward_velocity = propulsion_system_->atmosphere->velocity * std::sin((propulsion_system_->atmosphere->velocityAoA + this->propeller_plane_angle) * std::numbers::pi_v<float> / 180.0f);
            } else {
                throw std::bad_weak_ptr();
            }

            float inflow_velocity_guess_alpha = hover_inflow_sqr;
            float inflow_velocity_guess_beta = 2.0f * inflow_velocity_guess_alpha;

            std::size_t iterative_count = 0;
            std::size_t iterative_max = 100;

            do {
                const float temp_alpha = (advance_velocity + (hover_inflow_sqr / concpt::aux::get_euclidean_norm(forward_velocity, inflow_velocity_guess_alpha)) - inflow_velocity_guess_alpha);
                const float temp_beta = (advance_velocity + (hover_inflow_sqr / concpt::aux::get_euclidean_norm(forward_velocity, inflow_velocity_guess_beta)) - inflow_velocity_guess_beta);

                const float derivative = (temp_alpha - temp_beta) / (inflow_velocity_guess_alpha - inflow_velocity_guess_beta);
                inflow_velocity_guess_alpha = std::exchange(inflow_velocity_guess_beta, inflow_velocity_guess_beta - temp_beta / derivative);
            } while (std::abs(inflow_velocity_guess_beta - inflow_velocity_guess_alpha) > 1e-3 && iterative_count++ < iterative_max);

            if (iterative_count >= iterative_max) throw std::runtime_error("Induced Velocity did not converge...");

            return std::make_tuple(inflow_velocity_guess_alpha, advance_velocity, forward_velocity);
        }

        [[nodiscard]] float get_instantaneous_inflow_velocity (const float &IN_RADIUS, const float &IN_PSI_ANGLE, const float &IN_INFLOW, const float &IN_X) const noexcept {
            if (concpt::aux::check_equal(IN_X, 0.0f)) return IN_INFLOW;
            const float Kx = (15.0f * std::numbers::pi_v<float> / 32.0f) * std::tan(IN_X / 2.0f);
//            const float Kz = 0.0f;
            const float angle_rads = IN_PSI_ANGLE * std::numbers::pi_v<float> / 180.0f;
            return IN_INFLOW * (1.0f + Kx * std::cos(angle_rads) * (IN_RADIUS / this->blade_section->radius) /*+ Kz * std::sin(angle_rads) * (IN_RADIUS / this->blade_section->radius)*/);
        }

        [[nodiscard]] std::pair<float, float> get_blade_section_aero (const float &IN_BLADE_LOCATION, const float &IN_AOA,
                                                                      const float &IN_VELOCITY, const float &IN_CHORD,
                                                                      const float &IN_SPAN, const bool ONLY_COEFFS = false) const {
            if (std::shared_ptr<concpt::propulsion_system>propulsion_system_ = this->propulsion_system.lock()) {
                const float renolds_number = propulsion_system_->atmosphere->density * IN_VELOCITY * IN_CHORD / propulsion_system_->atmosphere->viscosity;
                const float mach_number = IN_VELOCITY / propulsion_system_->atmosphere->speedOfSound;
                const float sectional_area = IN_CHORD * IN_SPAN;
                const float dynamic_pressure = ONLY_COEFFS ? 1.0f : (0.5f * propulsion_system_->atmosphere->density *
                        concpt::aux::power<2>(IN_VELOCITY) * sectional_area);

                return this->blade_section->get_aero_values((IN_BLADE_LOCATION), IN_AOA, renolds_number, mach_number, dynamic_pressure);
            } else {
                throw std::bad_weak_ptr();
            }
        }

        std::pair<float, float> calculate_propeller_forces_moments (const float &ROOT_CUT_OFF = 0.1f) {
            if (this->detailed_output) {
                this->detailed_output->velocity_field_reset();
                this->detailed_output->angle_of_attak_reset();
                this->detailed_output->force_field_reset();
            }

            float inflow_velocity, forward_velocity;
            const auto [effective_area, speed_of_sound] = this->set_BEMT_parameters(ROOT_CUT_OFF);
            std::tie(inflow_velocity, std::ignore, forward_velocity) = this->get_induced_velocity();

            float total_force_axial = 0.0f, total_torque = 0.0f;
            for (std::size_t i = 0; i < this->blade_section->mesh_azimuthal; i++) {
                for (std::size_t j = 0; j < this->blade_section->mesh_radius - 1; j++) {
                    const float sectional_span = this->radius_divisions[j + 1] - this->radius_divisions[j];
                    // TODO: add 3D angle
                    const float tangential_sectional_velocity = this->radius_divisions[j] * this->controller_settings->initial_RPM + forward_velocity * std::cos(this->psi_angles_divisions[i] * std::numbers::pi_v<float> / 180.0f);
                    const float axial_sectional_velocity = this->get_instantaneous_inflow_velocity(this->radius_divisions[j], this->psi_angles_divisions[i], inflow_velocity, std::atan(forward_velocity / inflow_velocity));
                    const float radial_sectional_velocity = forward_velocity * std::sin(this->psi_angles_divisions[i] * std::numbers::pi_v<float> / 180.0f);

                    const float sectional_velocity = concpt::aux::get_euclidean_norm(tangential_sectional_velocity, axial_sectional_velocity);
                    if (sectional_velocity >= speed_of_sound) std::cerr << "Blade element with local velocity exceeding SPEED OF SOUND encountered..." << std::endl;
                    const float phi = std::atan(axial_sectional_velocity / tangential_sectional_velocity);

                    float sectional_chord, sectional_pitch;
                    std::tie(std::ignore, sectional_chord, sectional_pitch, std::ignore, std::ignore) = this->blade_section->get_sectional_data(this->radius_divisions[j]);
                    sectional_pitch += this->controller_settings->initial_pitch;

                    const float sectional_AoA = sectional_pitch - (phi * 180.0f / std::numbers::pi_v<float>);
                    const auto [sectional_lift, sectional_drag] = this->get_blade_section_aero(this->radius_divisions[j], sectional_AoA, sectional_velocity, sectional_chord, sectional_span);

                    const float current_tangential = sectional_lift * std::sin(phi) + sectional_drag * std::cos(phi);
                    const float current_axial = sectional_lift * std::cos(phi) - sectional_drag * std::sin(phi);
                    total_force_axial += current_axial;

                    if (this->detailed_output) {
                        this->detailed_output->add_velocity_data_point(std::array<float, 3>{
                                tangential_sectional_velocity, radial_sectional_velocity, axial_sectional_velocity
                        });

                        this->detailed_output->add_angle_of_attack_data_point(sectional_AoA);

                        this->detailed_output->add_force_data_point(std::array<float, 3>{
                                current_tangential, 0.0f, current_axial
                        });
                    }

                    total_torque += this->radius_divisions[j] * current_tangential;
                }
            }
            const float multiplier = static_cast<float>(this->blade_section->number_of_blades) / static_cast<float>(this->blade_section->mesh_azimuthal);
            total_force_axial *= multiplier;
            total_torque *= multiplier;

            if (std::shared_ptr<concpt::propulsion_system> propulsion_system_ = this->propulsion_system.lock()) {
                const float obtained_thrust_coeff = total_force_axial / (propulsion_system_->atmosphere->density * effective_area * concpt::aux::power<2>(this->controller_settings->initial_RPM * this->blade_section->radius));
                const float required_power = total_torque * this->controller_settings->initial_RPM;

                this->thrust_coefficient_obtained = obtained_thrust_coeff;
                this->power_required = required_power;

                if (std::isnan(obtained_thrust_coeff) || std::isnan(required_power)) throw std::runtime_error("Power calculated to NaN...");
                return std::make_pair(obtained_thrust_coeff, required_power);
            } else {
                throw std::bad_weak_ptr();
            }
        }

        static double nlopt_objective_function (unsigned n, const double *x, double *grad, void *data) {
            auto cast_data = static_cast<concpt::propeller*>(data);
            cast_data->controller_settings->update_controller_value(x);

            cast_data->calculate_propeller_forces_moments();
            return static_cast<double>(cast_data->power_required);
        }

        static double nlopt_equality_constraint (unsigned n, const double *x, double *grad, void *data) {
            auto cast_data = static_cast<concpt::propeller*>(data);
            cast_data->controller_settings->update_controller_value(x);

            return static_cast<double>(std::abs(cast_data->thrust_coefficient_obtained - cast_data->thrust_coefficient));
        }
    };
}

#include "../../src/propeller.tpp"

#endif //CONCEPTUAL_PROPELLER_H
