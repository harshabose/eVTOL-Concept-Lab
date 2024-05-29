//
// Created by Harshavardhan Karnati on 04/03/2024.
//

#ifndef CONCEPTUAL_AIRFOIL_H
#define CONCEPTUAL_AIRFOIL_H

#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <set>
#include <string>
#include <utility>
#include <algorithm>
#include <cstdlib>
#include <type_traits>
#include <filesystem>

//#define EIGEN_USE_DYNAMIC
//#define USE_MACH_DATA
#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include "nlohmann/json.hpp"
#include "interpolate.h"
#include "unsupported/meta_checks.h"
#include "unsupported/useful_expressions.h"
#include "unsupported/debug_utils.h"


struct airfoil_surrogate_model {
#ifdef USE_MACH_DATA
    std::shared_ptr<itp::interpolate<3, 500000, 5>> CL;
    std::shared_ptr<itp::interpolate<3, 500000, 5>> CD;
    std::shared_ptr<itp::interpolate<2, 5000, 5>> positive_stall;
    std::shared_ptr<itp::interpolate<2, 5000, 5>> negative_stall;
    std::shared_ptr<itp::interpolate<2, 5000, 5>> max_cl_cd_angle;
#else
    std::shared_ptr<itp::interpolate<2, 50000, 5>> CL;
    std::shared_ptr<itp::interpolate<2, 50000, 5>> CD;
    std::shared_ptr<itp::interpolate<1, 500, 5>> positive_stall;
    std::shared_ptr<itp::interpolate<1, 500, 5>> negative_stall;
    std::shared_ptr<itp::interpolate<1, 500, 5>> max_cl_cd_angle;
#endif
    std::vector<std::pair<float, float>> airfoil_coordinates_upper;
    std::vector<std::pair<float, float>> airfoil_coordinates_lower;

    float min_CL{}, max_CL{};
    float min_CD{}, max_CD{};
    float min_Re{}, max_Re{};
    float max_mach{}, min_mach{};
    float min_alpha{}, max_alpha{};
    float max_thickness_ratio{};
    bool surrogate_built;

#ifdef USE_MACH_DATA
    airfoil_surrogate_model ()
            : CL(std::make_shared<itp::interpolate<3, 500000, 5>>()),
              CD(std::make_shared<itp::interpolate<3, 500000, 5>>()),
              positive_stall(std::make_shared<itp::interpolate<2, 5000, 5>>()),
              negative_stall(std::make_shared<itp::interpolate<2, 5000, 5>>()),
              max_cl_cd_angle(std::make_shared<itp::interpolate<2, 5000, 5>>()),
              surrogate_built(false){}
#else
    airfoil_surrogate_model ()
            : CL(std::make_shared<itp::interpolate<2, 50000, 5>>()),
              CD(std::make_shared<itp::interpolate<2, 50000, 5>>()),
              positive_stall(std::make_shared<itp::interpolate<1, 500, 5>>()),
              negative_stall(std::make_shared<itp::interpolate<1, 500, 5>>()),
              max_cl_cd_angle(std::make_shared<itp::interpolate<1, 500, 5>>()),
              surrogate_built(false){}
#endif
};

namespace concpt {
    class airfoil_polar {
    public:
        std::unordered_map<std::string, airfoil_surrogate_model> surrogate_hash_map;

        airfoil_polar () = default;
        virtual ~airfoil_polar () = default;

        explicit airfoil_polar (const std::vector<std::string> &IN_AIRFOILS, const bool IN_TRAIN_IN_PLACE = false,
                                const bool IN_BUILD_IN_PLACE = false) : airfoil_index(IN_AIRFOILS) {
            std::sort(this->airfoil_index.begin(), this->airfoil_index.end());
            this->airfoil_index.erase(std::unique(this->airfoil_index.begin(), this->airfoil_index.end()), this->airfoil_index.end());
            this->number_of_airfoils = this->airfoil_index.size();
            this->build_hash_map();
            if (IN_TRAIN_IN_PLACE) this->build_training_data();
            if (IN_BUILD_IN_PLACE) this->build_surrogate_models();
        }

        template<class... airfoilType>
        requires (meta_checks::is_string_v<airfoilType> && ...)
        void add_airfoils (airfoilType&&... IN_AIRFOILS) {
            this->airfoil_index.insert(this->airfoil_index.end(), {std::forward<airfoilType>(IN_AIRFOILS) ...});
            std::sort(this->airfoil_index.begin(), this->airfoil_index.end());
            this->airfoil_index.erase(std::unique(this->airfoil_index.begin(), this->airfoil_index.end()), this->airfoil_index.end());
            this->number_of_airfoils = this->airfoil_index.size();

            this->build_hash_map();
            this->build_training_data();
            this->build_surrogate_models();
        }

        airfoil_surrogate_model& hash_airfoil (const std::string &IN_AIRFOIL_NAME) {
            return this->surrogate_hash_map.at(IN_AIRFOIL_NAME);
        }

        [[nodiscard]] std::string get_airfoil_from_index (std::size_t INDEX) const {
            return this->airfoil_index.at(INDEX);
        }

        [[nodiscard]] std::vector<std::string> get_airfoil_list () const {
            return this->airfoil_index;
        }

        std::pair<float, float> get_aero_values (const std::string &IN_AIRFOIL, const float &IN_ALPHA, const float &IN_RE,
                                                 const float &IN_MACH = 0.0f, const float &IN_MULTIPLIER = 1.0f) {
#ifdef USE_MACH_DATA
            return std::make_pair(this->surrogate_hash_map.at(IN_AIRFOIL).CL->eval_at_with_weighted_sum(
                    IN_ALPHA, IN_RE, IN_MACH) * IN_MULTIPLIER, this->surrogate_hash_map.at(IN_AIRFOIL).CD->eval_at_with_weighted_sum(
                                          IN_ALPHA, IN_RE, IN_MACH) * IN_MULTIPLIER);
#else
            return std::make_pair(this->surrogate_hash_map.at(IN_AIRFOIL).CL->eval_at_with_weighted_sum(IN_ALPHA, IN_RE) * IN_MULTIPLIER,
                                  this->surrogate_hash_map.at(IN_AIRFOIL).CD->eval_at_with_weighted_sum(IN_ALPHA, IN_RE) * IN_MULTIPLIER);
#endif
        }

    private:
        std::vector<std::string> airfoil_index;
        // path to server locations
        std::string scriptPath = "/Users/harsha/Desktop/Conceptual/src/airfoil_generator.py";
        std::string save_path = "/Users/harsha/Desktop/Conceptual/assets/airfoil_data";
        std::string python_path = "/opt/homebrew/bin/python3";
        std::size_t number_of_airfoils = 0;
        std::size_t num_airfoil_done = 0;


        void call_python_script (const std::string& IN_CURRENT_AIRFOIL) {
            std::string current_save_path = this->save_path + "/" + IN_CURRENT_AIRFOIL;
            if (!std::filesystem::exists(current_save_path)) {
                DEBUG_LOG("Working on airfoil -> " << IN_CURRENT_AIRFOIL << ". [" << this->num_airfoil_done << "/" << this->number_of_airfoils << "]");
                std::string CL_command = this->python_path + " " + this->scriptPath + " " + IN_CURRENT_AIRFOIL + " " + current_save_path;
                int result = system(CL_command.c_str());
                if (result != 0) {
                    std::cerr << "Error executing the Python script with airfoil name -> " << IN_CURRENT_AIRFOIL
                              << ", and save path ->" << current_save_path << std::endl;
                } else {
                    std::cout << "Completed airfoil -> " << IN_CURRENT_AIRFOIL << ", saved at -> " << current_save_path << std::endl;
                    this->num_airfoil_done++;
                }
            } else {
                DEBUG_LOG("Current airfoil (" << IN_CURRENT_AIRFOIL << ") training data already exits");
                DEBUG_LOG("Skipping current airfoil");
                this->num_airfoil_done++;
            }
        }

        void build_training_data () {
            for (auto& current_airfoil : this->airfoil_index) {
                this->call_python_script(current_airfoil);
            }
            if (this->num_airfoil_done == this->number_of_airfoils) {
                DEBUG_LOG("Build training data for all airfoils...");
            }
            else std::cerr << "Not all airfoils' training data build. Check log..." << std::endl;
        }

        void build_surrogate_models () {
            this->num_airfoil_done = 0;
            for (auto& each_airfoil : this->airfoil_index) {
                DEBUG_LOG("Building Surrogate Models of airfoil -> " << each_airfoil << " [" << this->num_airfoil_done++ << "/"
                                                                     << this->number_of_airfoils << "]");
                this->build_surrogate_model_at(each_airfoil);
                DEBUG_LOG("Surrogate Build Complete for airfoil -> " << each_airfoil);

                try {
                    this->get_airfoil_coordinates(each_airfoil);
                } catch (std::exception &e) {
                    std::cerr << e.what() << std::endl;
                    std::cerr << "Skipping coordinates capture..." << std::endl;
                }
            }
        }

        void build_surrogate_model_at (const std::string& IN_CURRENT_AIRFOIL) {
            if (this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).surrogate_built) {
                return;
            }
            try {
                std::vector<float> CL_, CD_, Re_, alpha_, positive_stall, negative_stall, max_cl_cd, unique_re;
                std::size_t number_of_alpha, number_of_Re;
#ifdef USE_MACH_DATA
                std::vector<float> mach_, unique_mach;
                std::size_t number_of_mach;
#endif
                const std::string current_airfoil_path = this->save_path + "/" + IN_CURRENT_AIRFOIL + "/" + IN_CURRENT_AIRFOIL + "_training" + ".json";
                std::ifstream data_file(current_airfoil_path, std::ios::ate);

                if (!data_file.is_open() && data_file.tellg() != 0) {
                    throw std::runtime_error("Could not open file for reading or it is empty: " + current_airfoil_path);
                }
                data_file.seekg(0, std::ios::beg);
                const nlohmann::json json_data = nlohmann::json::parse(data_file);
                data_file.close();

                try {
                    if (!(json_data.contains("CL") && json_data.contains("CD") && json_data.contains("Re") &&
                          json_data.contains("alpha") && json_data.contains("mach"))) {
                        throw std::runtime_error("JSON Data structure is unexpected...");
                    }
                    if (!(json_data["CL"].is_array() && json_data["CD"].is_array() && json_data["Re"].is_array() &&
                          json_data["alpha"].is_array() && json_data["mach"].is_array())) {
                        throw std::runtime_error("JSON Data datatype is unknown...");
                    }
                    CL_ = json_data.at("CL").get<std::vector<float>>();
                    CD_ = json_data.at("CD").get<std::vector<float>>();
                    alpha_ = json_data.at("alpha").get<std::vector<float>>();
                    Re_ = json_data.at("Re").get<std::vector<float>>();

                    {
                        std::set<float> uniq_RE(Re_.begin(), Re_.end());
                        std::set<float> uniq_ALPHA(alpha_.begin(), alpha_.end());
                        number_of_alpha = uniq_ALPHA.size();
                        number_of_Re = uniq_RE.size();
                    }

#ifndef USE_MACH_DATA
                    CL_.resize(number_of_alpha * number_of_Re);
                    CD_.resize(number_of_alpha * number_of_Re);
                    alpha_.resize(number_of_alpha * number_of_Re);
                    Re_.resize(number_of_alpha * number_of_Re);
#endif

#ifdef USE_MACH_DATA
                    mach_ = json_data.at("mach").get<std::vector<float>>();
                    {
                        std::set<float> uniq_MACH(mach_.begin(), mach_.end());
                        number_of_mach = uniq_MACH.size();
                    };

                    std::tie(positive_stall, negative_stall, unique_re, unique_mach) = this->get_stall_angles(CL_, Re_, alpha_, number_of_alpha, number_of_Re, number_of_mach, mach_);
                    max_cl_cd = this->get_max_cl_cd_angles(CL_, CD_, Re_, alpha_, number_of_alpha, number_of_Re, number_of_mach, mach_);
#else
                    std::tie(positive_stall, negative_stall, unique_re, std::ignore) = this->get_stall_angles(CL_, Re_, alpha_, number_of_alpha, number_of_Re);
                    max_cl_cd = this->get_max_cl_cd_angles(CL_, CD_, Re_, alpha_, number_of_alpha, number_of_Re);
#endif

                } catch (std::exception &e) {
                    throw std::runtime_error("No JSON with map CL or CD or Re or alpha");
                }

                try {
#ifndef EIGEN_USE_DYNAMIC
    #ifdef USE_MACH_DATA
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->add_training_data<500000>(CL_, alpha_, mach_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->add_training_data<500000>(CD_, alpha_, mach_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->add_training_data<5000>(positive_stall, unique_mach, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->add_training_data<5000>(negative_stall, unique_mach, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->add_training_data<5000>(max_cl_cd, unique_mach, unique_re);
    #else
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->add_training_data<50000>(CL_, alpha_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->add_training_data<50000>(CD_, alpha_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->add_training_data<500>(positive_stall, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->add_training_data<500>(negative_stall, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->add_training_data<500>(max_cl_cd, unique_re);
    #endif
#else
    #ifdef USUSE_MACH_DATA
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->add_training_data(CL_, alpha_, mach_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->add_training_data(CD_, alpha_, mach_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->add_training_data(positive_stall, unique_mach, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->add_training_data(negative_stall, unique_mach, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->add_training_data(max_cl_cd, unique_mach, unique_re);
    #else
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->add_training_data(CL_, alpha_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->add_training_data(CD_, alpha_, Re_);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->add_training_data(positive_stall, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->add_training_data(negative_stall, unique_re);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->add_training_data(max_cl_cd, unique_re);
    #endif

#endif
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->set_ptr_trained_data(this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->get_ptr_trained_data());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->set_ptr_trained_data(this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->get_ptr_trained_data());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->set_ptr_trained_data(this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->get_ptr_trained_data());

                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_CL = *std::ranges::max_element(CL_.begin(), CL_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_CD = *std::ranges::max_element(CD_.begin(), CD_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_alpha = *std::ranges::max_element(alpha_.begin(), alpha_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_Re = *std::ranges::max_element(Re_.begin(), Re_.end());


                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).min_CL = *std::ranges::min_element(CL_.begin(), CL_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).min_CD = *std::ranges::min_element(CD_.begin(), CD_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).min_alpha = *std::ranges::min_element(alpha_.begin(), alpha_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).min_Re = *std::ranges::min_element(Re_.begin(), Re_.end());
#ifdef USE_MACH_DATA
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_mach = *std::ranges::max_element(mach_.begin(), mach_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).min_mach = *std::ranges::min_element(mach_.begin(), mach_.end());
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->add_major_axis_factors(1.0f, 0.1f, 0.01f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->add_major_axis_factors(1.0f, 0.1f, 0.01f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->add_major_axis_factors(0.1f, 1.0f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->add_major_axis_factors(0.1f, 1.0f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->add_major_axis_factors(0.1f, 1.0f);
#endif
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CL->add_major_axis_factors(1.0f, 0.01f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).CD->add_major_axis_factors(1.0f, 0.01f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).positive_stall->add_major_axis_factors(0.1f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).negative_stall->add_major_axis_factors(0.1f);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_cl_cd_angle->add_major_axis_factors(0.1f);


                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).surrogate_built = true;
                } catch (std::exception &e) {
                    throw std::runtime_error("Error while inputting training data to surrogate model");
                }
            } catch (std::exception &e) {
                std::cerr << e.what() << std::endl;
                std::cerr << "Error while converting JSON -> Eigen data..." << std::endl;
                throw;
            }
        }


        void build_hash_map () {
            DEBUG_LOG("Generating airfoil surrogate model skeleton hash map...");
            for (auto& each_airfoil : this->airfoil_index)
                this->surrogate_hash_map.try_emplace(each_airfoil, airfoil_surrogate_model());
        }

        std::vector<float>::iterator get_stall_angles_helper (std::vector<float>::iterator IN_START, std::vector<float>::iterator IN_END, const bool positive_angle = true) {
            if (IN_START == IN_END || std::abs(std::ranges::distance(IN_START, IN_END)) <= 1) {
                return IN_END;
            }

            auto current = std::ranges::next(IN_START + (positive_angle ? 1 : -1));
            auto previous = IN_START;

            while (current != IN_END) {
                if (positive_angle) {
                    if (*current < *previous) return current; // Found the index
                } else {
                    if (*current > *previous) return current; // Found the index
                }

                previous = current;
                std::ranges::advance(current, (positive_angle ? 1 : -1));
            }
            std::cerr << "Failed to find the stall angle..." << std::endl;
            return IN_END;
        }

        std::array<std::vector<float>, 4>
        get_stall_angles (std::vector<float> &IN_CL, std::vector<float> &IN_RE, std::vector<float> &IN_ALPHA,
                          const std::size_t &IN_NUM_ALPHA, const std::size_t &IN_NUM_RE, const std::size_t &IN_NUM_MACH = 1,
                          const std::vector<float> &IN_MACH = {0.0f}) {
            std::vector<float>OUT_POSITIVE_STALL_ANGLE;
            std::vector<float>OUT_NEGATIVE_STALL_ANGLE;
            std::vector<float>OUT_RE;
            std::vector<float>OUT_MACH;

            for (std::size_t i = 0; i < IN_NUM_MACH; i++) {
                OUT_MACH.push_back(*std::ranges::next(std::ranges::begin(IN_MACH), i * IN_NUM_ALPHA * IN_NUM_RE));
                for (std::size_t j = 0; j < IN_NUM_RE; j++) {
                    OUT_RE.push_back(*std::ranges::next(std::ranges::begin(IN_RE), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA * 0.5));
                    auto start = std::ranges::next(std::ranges::begin(IN_CL), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA * 0.5);
                    auto end_positive = std::ranges::next(std::ranges::begin(IN_CL), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA);
                    OUT_POSITIVE_STALL_ANGLE.push_back(*(std::ranges::next(std::ranges::begin(IN_ALPHA), std::ranges::distance(std::ranges::begin(IN_CL), this->get_stall_angles_helper(start, end_positive)))));
                    auto end_negative = std::ranges::next(std::ranges::begin(IN_CL), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA);
                    OUT_NEGATIVE_STALL_ANGLE.push_back(*(std::ranges::next(std::ranges::begin(IN_ALPHA), std::ranges::distance(std::ranges::begin(IN_CL), this->get_stall_angles_helper(start, end_negative, false)))));
                }
            }
            return std::array<std::vector<float>, 4>{std::move(OUT_POSITIVE_STALL_ANGLE), std::move(OUT_NEGATIVE_STALL_ANGLE), std::move(OUT_RE), std::move(OUT_MACH)};
        }

        std::vector<float>::iterator
        get_max_cl_cd_angles_helper (std::vector<float>::iterator IN_START_CL, std::vector<float>::iterator IN_START_CD,
                                     std::vector<float>::iterator IN_END_CL, std::vector<float>::iterator IN_END_CD) {
            if (IN_START_CL == IN_END_CL || std::abs(std::ranges::distance(IN_START_CL, IN_END_CL)) <= 1) {
                return IN_END_CL;
            }

            auto current_cl = std::ranges::next(IN_START_CL +  1);
            auto current_cd = std::ranges::next(IN_START_CD + 1);
            auto previous_cl = IN_START_CL;
            auto previous_cd = IN_START_CD;

            while (current_cl != IN_END_CL || current_cd != IN_END_CD) {
                if ((*current_cl)/(*current_cd) < (*previous_cl)/(*previous_cd)) return current_cl;

                previous_cl = current_cl;
                previous_cd = current_cd;
                std::ranges::advance(current_cl, 1);
                std::ranges::advance(current_cd, 1);
            }
            std::cerr << "Failed to find the max cl-by-cd angle..." << std::endl;
            return IN_END_CL;
        }

        std::vector<float>
        get_max_cl_cd_angles (std::vector<float> &IN_CL, std::vector<float> &IN_CD, std::vector<float> &IN_RE, std::vector<float> &IN_ALPHA,
                              const std::size_t &IN_NUM_ALPHA, const std::size_t &IN_NUM_RE, const std::size_t &IN_NUM_MACH = 1,
                              const std::vector<float> &IN_MACH = {0.0f}) {
            std::vector<float>OUT_MAX_CL_CD_ANGLE;

            for (std::size_t i = 0; i < IN_NUM_MACH; i++) {
                for (std::size_t j = 0; j < IN_NUM_RE; j++) {
                    auto start_cl = std::ranges::next(std::ranges::begin(IN_CL), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA * 0.5);
                    auto start_cd = std::ranges::next(std::ranges::begin(IN_CD), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA * 0.5);
                    auto end_cl = std::ranges::next(std::ranges::begin(IN_CL), i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA);
                    auto end_cd = std::ranges::next(std::ranges::begin(IN_CL), i * i * IN_NUM_ALPHA * IN_NUM_RE + j * IN_NUM_ALPHA + IN_NUM_ALPHA);

                    OUT_MAX_CL_CD_ANGLE.push_back(*(std::ranges::next(std::ranges::begin(IN_ALPHA),
                                                                      std::ranges::distance(std::ranges::begin(IN_CL),
                                                                                            this->get_max_cl_cd_angles_helper(start_cl, start_cd, end_cl, end_cd)))));
                }
            }
            return OUT_MAX_CL_CD_ANGLE;
        }

        void get_airfoil_coordinates (const std::string &IN_CURRENT_AIRFOIL) {
            if (!this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).airfoil_coordinates_upper.empty() &&
                !this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).airfoil_coordinates_lower.empty()) {
                return;
            }

            const std::string current_airfoil_path = this->save_path + "/" + IN_CURRENT_AIRFOIL + "/" + IN_CURRENT_AIRFOIL + "_coordinates" + ".json";
            std::ifstream data_file(current_airfoil_path, std::ios::ate);

            if (!data_file.is_open() && data_file.tellg() != 0) {
                throw std::runtime_error("Could not open file for reading or it is empty: " + current_airfoil_path);
            }
            data_file.seekg(0, std::ios::beg);
            const nlohmann::json json_data = nlohmann::json::parse(data_file);
            data_file.close();

            try {
                if (!(json_data.contains("UPPER_X_COORD") && json_data.contains("UPPER_Y_COORD") &&
                      json_data.contains("LOWER_X_COORD") && json_data.contains("LOWER_Y_COORD") &&
                      json_data.contains("MAX_THICKNESS"))) {
                    throw std::runtime_error("JSON Data structure is unexpected...");
                }
                if (!(json_data["UPPER_X_COORD"].is_array() && json_data["UPPER_Y_COORD"].is_array() &&
                      json_data["LOWER_X_COORD"].is_array() && json_data["LOWER_Y_COORD"].is_array() &&
                      json_data["MAX_THICKNESS"].is_number_float())) {
                    throw std::runtime_error("JSON Data is of unknown datatype..");
                }

                const std::vector<float> upper_coordinates_x = json_data.at("UPPER_X_COORD").get<std::vector<float>>();
                const std::vector<float> upper_coordinates_y = json_data.at("UPPER_Y_COORD").get<std::vector<float>>();
                const std::vector<float> lower_coordinates_x = json_data.at("LOWER_X_COORD").get<std::vector<float>>();
                const std::vector<float> lower_coordinates_y = json_data.at("LOWER_Y_COORD").get<std::vector<float>>();

                this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).max_thickness_ratio = json_data.at("MAX_THICKNESS").get<float>();
                for (std::size_t i = 0; i < upper_coordinates_x.size(); i++) {
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).airfoil_coordinates_upper.emplace_back(upper_coordinates_x[i] - 0.5f, upper_coordinates_y[i]);
                    this->surrogate_hash_map.at(IN_CURRENT_AIRFOIL).airfoil_coordinates_lower.emplace_back(lower_coordinates_x[i] - 0.5f, lower_coordinates_y[i]);
                }
            } catch (std::exception &e) {
                std::cerr << e.what() << std::endl;
                std::cerr << "Error while attaching converting JSON coordinates to vector data..." << std::endl;
            }
        }
    };
}



void generate_airfoil_polars ();

#endif //CONCEPTUAL_AIRFOIL_H
