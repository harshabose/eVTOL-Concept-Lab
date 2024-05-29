/**
 * @file interpolate.h
 * @brief Header file defining the interpolate class for k-nearest points average interpolation.
 *
 * @author Harshavardhan Karnati
 * @date 07/03/2024
 */

#ifndef CONCEPTUAL_INTERPOLATE_H
#define CONCEPTUAL_INTERPOLATE_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <numeric>
#include <concepts>
#include <queue>
#include <utility>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "unsupported/debug_utils.h"
#include "unsupported/meta_checks.h"
#include "unsupported/useful_data_types.h"

namespace itp {
    /**
 * @brief Compile-time check if the given type is a std::vector<float> or not.
 * @tparam T Type to check
 */
    template <class... T>
    static inline constexpr bool check_vector = ((std::is_same_v<meta_checks::remove_all_qual<T>, std::vector<float>>) && ...);

    template<typename T>
    struct is_float_array : std::false_type {};

    template<std::size_t N>
    struct is_float_array<std::array<float, N>> : std::true_type {};
/**
 * @brief Compile-time check if the given type is a std::array<float, N> or not.
 * @tparam N Size of the array
 * @tparam T Type to check
 */
    template<class... T>
    static inline constexpr bool check_array = (is_float_array<meta_checks::remove_all_qual<T>>::value && ...);

/**
 * @brief Compile-time check if the given type is a (std::array<float, N> or std::vector<float>) or not.
 * @tparam N Size of the array
 * @tparam T Type to check
 */
    template <class... T>
    static inline constexpr bool check_vector_array = itp::check_vector<T...> || itp::check_array<T...>;

/**
 * @brief Compile-time check if the given type is a Eigen::Vector<float, N> or not.
 * @tparam N Size of the Eigen::Vector
 * @tparam T Type to check
 */
    template <std::size_t N, class... T>
    static inline constexpr bool check_eigen = ((std::is_same_v<meta_checks::remove_all_qual<T>, Eigen::Vector<float, N>>) && ...);

    template <std::size_t N1, std::size_t N2, class...T>
    static inline constexpr bool check_eigen_map = ((std::is_same_v<meta_checks::remove_all_qual<T>, Eigen::Map<Eigen::Matrix<float, N1, N2>>>) && ...);

    template <typename type>
    static inline constexpr bool check_eigen_dynamic = std::is_base_of_v<Eigen::MatrixBase<meta_checks::remove_all_qual<type>>, meta_checks::remove_all_qual<type>>;

    /**
 * @brief Compile-time check if the given type is a (Eigen::Vector<float, N> or std::array<float, N> or std::vector<float>)
 * or not
 * @tparam N Size of the Eigen::Vector or std::array
 * @tparam T Type to check
 */
    template <std::size_t N, class... T>
    static inline constexpr bool check_all = itp::check_eigen<N, T...> || itp::check_vector<T...> || itp::check_array<T...>;




/**
 * @class interpolate
 * @brief Interpolator class for k-nearest points average interpolation.
 *
 * @details The interpolate class provides functionality for k-nearest points average interpolation
 * (k-value is given by mean_size). It allows users to add training data, evaluate the function at
 * a specified point using various input formats, and includes methods optimised for different data sizes.
 *
 * @tparam dimension The dimensionality of the input space.
 * @tparam max_training_data_size The maximum size allocated for training data storage.
 * @tparam mean_size The size of the subset used to calculate the mean during interpolation.
 *
 * @note Uses Eigen library for efficient linear algebra operations. Ensure Eigen3 is properly installed
 * and included in the project for optimal performance. By default, the code assumes the path is local.
 */
    template <std::size_t dimension, std::size_t max_training_data_size, std::size_t mean_size>
    class interpolate {
    public:


        /**
         * @brief Default constructor for the interpolate class.
         */
        interpolate () : scaling_factors(Eigen::Matrix<float, Eigen::Dynamic, 1>::Ones(dimension)) {
#ifdef EIGEN_USE_DYNAMIC
            this->func_data = std::make_shared<Eigen::Matrix<float, Eigen::Dynamic, 1>>(max_training_data_size, 1);
            this->var_data = std::make_shared<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(max_training_data_size, dimension);
#else
            this->func_data = std::make_shared<Eigen::Vector<float, max_training_data_size>>();
            this->var_data = std::make_shared<Eigen::Matrix<float, max_training_data_size, dimension>>();
#endif
        };


        /**
         * @brief Virtual destructor for the interpolate class.
         */
        virtual ~interpolate () = default;


        /**
         * @brief Adds training data to the interpolator.
         *
         * @details This function adds training data, consisting of function values and corresponding variable values,
         * to the interpolator. The function values are represented by the Eigen::Vector @p IN_FUNC_DATA,
         * and the variable values are represented by the Eigen::Matrix @p IN_VAR_DATA. Both function and
         * variable data are added to the interpolator's internal storage.
         *
         * @tparam current_training_size The size of the function data vector and the number of rows in the variable data matrix.
         *
         * @param [in] IN_FUNC_DATA The function values to be added.
         * @param [in] IN_VAR_DATA The corresponding variable values to be added.
         *
         * @throw std::runtime_error Thrown if the total size of the existing data and the new data exceeds the maximum allocated size.
         *
         * @note This method updates the internal state of the interpolator and increases the total number of training points.
         */
#ifdef EIGEN_USE_DYNAMIC
        template<typename derived1, typename derived2>
        requires (std::is_same_v<typename derived1::Scalar, float> && (std::is_same_v<typename derived2::Scalar, float>))
        void add_training_data (const Eigen::MatrixBase<derived1> &IN_FUNC_DATA, const Eigen::MatrixBase<derived2> &IN_VAR_DATA) {
            if (IN_FUNC_DATA.rows() != IN_VAR_DATA.rows()) {
                throw std::runtime_error("'IN_FUNC_DATA' and 'IN_VAR_DATA' row count needs to be same...");
            }
            if (IN_VAR_DATA.cols() != dimension) {
                throw std::runtime_error("'IN_VAR_DATA' col count needs to be 'dimension'...");
            }
            if (this->interpolated_data_size + IN_FUNC_DATA.rows() > max_training_data_size) {
                std::cerr << "Current size: " << this->interpolated_data_size
                          << ", Attempted size: " << this->interpolated_data_size + IN_FUNC_DATA.rows()
                          << ", Maximum size: " << max_training_data_size << std::endl;
                throw std::runtime_error("Input Training data size exceeds max size");
            }

            std::size_t current_training_size = IN_FUNC_DATA.rows();

            this->func_data->segment(this->interpolated_data_size, current_training_size) = IN_FUNC_DATA;
            this->var_data->block(this->interpolated_data_size, 0, current_training_size, dimension) = IN_VAR_DATA;

            this->interpolated_data_size += current_training_size;
            if (this->interpolated_data_size == max_training_data_size) DEBUG_LOG("max training data size reached. No more points are accepted");
        }
#else
        template<std::size_t current_training_size>
        void add_training_data (const Eigen::Vector<float, current_training_size> &IN_FUNC_DATA,
                                const Eigen::Matrix<float, current_training_size, dimension> &IN_VAR_DATA) {

            // check if max training size is reach
            if (this->interpolated_data_size + current_training_size > max_training_data_size) {
                std::cerr << "Current size: " << this->interpolated_data_size
                          << ", Attempted size: " << this->interpolated_data_size + current_training_size
                          << ", Maximum size: " << max_training_data_size << std::endl;
                throw std::runtime_error("Input Training data size exceeds max size");
            }

            this->func_data->template segment<current_training_size>(this->interpolated_data_size) = IN_FUNC_DATA;
            this->var_data->template block<current_training_size, dimension>(this->interpolated_data_size, 0) = IN_VAR_DATA;

            this->interpolated_data_size += current_training_size;
            if (this->interpolated_data_size == max_training_data_size) DEBUG_LOG("max training data size reached. No more points are accepted");
        }
#endif

        /**
         * @brief Adds training data to the interpolator with variadic input for variable values.
         *
         * @details This function adds training data to the interpolator, where the function values are
         * represented by the Eigen::Vector @p IN_FUNC_DATA, and the variable values are represented by the
         * variadic parameter pack @p IN_VAR_DATAs. The number of variable values must match the specified
         * dimension, and each variable value in the pack must be of type Eigen::Vector<float, current_training_size>
         *
         * @tparam current_training_size The size of the function data vector.
         * @tparam var_type Variadic template parameters enforcing Eigen::Vector<float, current_training_size>
         *
         * @param [in] IN_FUNC_DATA The function values to be added.
         * @param [in] IN_VAR_DATAs Variadic input representing the variable values to be added.
         *
         * @throw std::runtime_error Thrown if the total size of the existing data and the new data exceeds the maximum
         * allocated size.
         *
         * @note This method updates the internal state of the interpolator and increases the total number of training points.
         * @note This method copies the values. It is a good idea to clear the data in the higher scope or use mapped version
         */
#ifdef EIGEN_USE_DYNAMIC
        template <typename derivedFunc, typename... derivedVars>
        requires (sizeof...(derivedVars) == dimension && std::is_same_v<typename derivedFunc::Scalar, float> && (std::is_same_v<typename derivedVars::Scalar, float> && ...))
        void add_training_data (const Eigen::MatrixBase<derivedFunc> &IN_FUNC_DATA, const Eigen::MatrixBase<derivedVars> &...IN_VAR_DATAs) {
            if ((... || (IN_FUNC_DATA.rows() != IN_VAR_DATAs.rows()))) [[unlikely]] {
                throw std::runtime_error("'IN_FUNC_DATA' and 'IN_VAR_DATA' row count needs to be same...");
            }

            if (this->interpolated_data_size + IN_FUNC_DATA.rows() > max_training_data_size) {
                std::cerr << "Current size: " << this->interpolated_data_size
                          << ", Attempted size: " << this->interpolated_data_size + IN_FUNC_DATA.rows()
                          << ", Maximum size: " << max_training_data_size << std::endl;
                throw std::runtime_error("Input Training data size exceeds max size");
            }

            std::size_t current_training_size = IN_FUNC_DATA.rows();

            this->func_data->segment(this->interpolated_data_size, current_training_size) = IN_FUNC_DATA;

            [&IN_VAR_DATAs..., &current_training_size, this] <std::size_t... i> (std::index_sequence<i...>){
                ((this->var_data->block(this->interpolated_data_size, i, current_training_size, 1) = IN_VAR_DATAs),...);
            }(std::make_index_sequence<dimension>{});

            this->interpolated_data_size += current_training_size;
            if (this->interpolated_data_size == max_training_data_size)
                DEBUG_LOG("max training data size reached. No more points are accepted");
        };
#else
        template<std::size_t current_training_size, class func_type, class... var_type>
        requires (itp::check_eigen<current_training_size, func_type, var_type...> ||
                  itp::check_eigen_map<current_training_size, 1, func_type, var_type...>)
        void add_training_data (const func_type &IN_FUNC_DATA, const var_type&... IN_VAR_DATAs) {
            // check if max training size is reach
            if (this->interpolated_data_size + current_training_size > max_training_data_size) {
                std::cerr << "Current size: " << this->interpolated_data_size
                          << ", Attempted size: " << this->interpolated_data_size + current_training_size
                          << ", Maximum size: " << max_training_data_size << std::endl;
                throw std::runtime_error("Input Training data size exceeds max size");
            }

            this->func_data->template segment<current_training_size>(this->interpolated_data_size) = IN_FUNC_DATA;

            [&IN_VAR_DATAs..., this] <std::size_t... i> (std::index_sequence<i...>){
                ((this->var_data->template block<current_training_size, 1>(this->interpolated_data_size, i) = IN_VAR_DATAs),...);
            }(std::make_index_sequence<dimension>{});

            this->interpolated_data_size += current_training_size;
            if (this->interpolated_data_size == max_training_data_size)
                    DEBUG_LOG("max training data size reached. No more points are accepted");
        }
#endif

        /**
         * @brief Adds training data to the interpolator with variadic input for variable values.
         *
         * @details This function adds training data to the interpolator, where the function values are
         * represented by the Eigen::Vector @p IN_FUNC_DATA, and the variable values are represented by the
         * variadic parameter pack @p IN_VAR_DATAs. The number of variable values must match the specified
         * dimension, and each variable value in the pack must be of type  or std::vector or
         * std::array<float, current_training_size>.
         *
         * @tparam current_training_size The size of the function data vector.
         * @tparam var_type Variadic template parameters enforcing types std::vector or
         * std::array<float, current_training_size>.
         *
         * @param [in] IN_FUNC_DATA The function values to be added.
         * @param [in] IN_VAR_DATAs Variadic input representing the variable values to be added.
         *
         * @throw std::runtime_error Thrown if the total size of the existing data and the new data exceeds the maximum
         * allocated size.
         *
         * @note This method updates the internal state of the interpolator and increases the total number of training points.
         * @note This method copies the values. It is a good idea to clear the data in the higher scope or use mapped version
         */
#ifdef EIGEN_USE_DYNAMIC
        template<class func_type, class... var_type>
        requires ((sizeof...(var_type) == dimension) && (itp::check_vector_array<func_type, var_type...>))
        void add_training_data (func_type &&IN_FUNC_DATA, var_type&&... IN_VAR_DATAs) {
            std::size_t current_training_size = std::forward<func_type>(IN_FUNC_DATA).size();

            auto ensure_eigen_format = [&current_training_size] <class type> (type&& IN_CONTAINER) -> decltype(auto) {
                if constexpr (itp::check_vector_array<type>) {
                    if (std::forward<type>(IN_CONTAINER).size() != current_training_size) [[unlikely]] {
                        std::cerr << "Required size: " << dimension << ", Given size: "
                                  << std::forward<type>(IN_CONTAINER).size() << std::endl;
                        throw std::invalid_argument("Container must have a size of 'dimension");
                    }
                    return static_cast<Eigen::Matrix<float, Eigen::Dynamic, 1>>(
                            Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>>(std::forward<type>(IN_CONTAINER).data(),
                                                                                std::forward<type>(IN_CONTAINER).size()));
                } else if constexpr (itp::check_eigen_dynamic<type>) {
                    if (std::forward<type>(IN_CONTAINER).rows() != current_training_size) {
                        std::cerr << "Required size: " << current_training_size << ", Current size: "
                                << std::forward<type>(IN_CONTAINER).rows() << std::end;
                        throw std::runtime_error("Eigen Object size is not compatable...");
                    }
                    return std::forward<type>(IN_CONTAINER);
                } else {
                    std::cerr << "Only works with std::vector or std::array. Unknown data-type given" << std::endl;
                    throw std::invalid_argument("Only works with std::vector or std::array. Unknown data-type given");
                }
            };

            this->add_training_data(ensure_eigen_format(std::forward<func_type>(IN_FUNC_DATA)), ensure_eigen_format(std::forward<var_type>(IN_VAR_DATAs))...);
        };
#else
        template<std::size_t current_training_size, class func_type, class... var_type>
        requires ((sizeof...(var_type) == dimension) && (itp::check_vector_array<func_type, var_type...>) &&
                (!std::is_rvalue_reference_v<func_type>) && (!(std::is_rvalue_reference_v<var_type> && ...)))
        void add_training_data (func_type &&IN_FUNC_DATA, var_type&&... IN_VAR_DATAs) {

            auto ensure_eigen_format = [] <class type> (type&& IN_CONTAINER) -> decltype(auto) {
                if constexpr (itp::check_vector_array<type>) {
                    if (std::forward<type>(IN_CONTAINER).size() != current_training_size) {
                        std::cerr << "Required size: " << dimension << ", Given size: " << std::forward<type>(IN_CONTAINER).size() << std::endl;
                        throw std::invalid_argument("Container must have a size of 'dimension");
                    }
                    return Eigen::Map<Eigen::Vector<float, current_training_size>>(std::forward<type>(IN_CONTAINER).data());
                } else if constexpr (itp::check_eigen<current_training_size, type>){
                    return std::forward<type>(IN_CONTAINER);
                } else {
                    std::cerr << "Only works with std::vector or std::array. Unknown data-type given" << std::endl;
                    throw std::invalid_argument("Only works with std::vector or std::array. Unknown data-type given");
                }
            };

            this->add_training_data<current_training_size>(ensure_eigen_format(std::forward<func_type>(IN_FUNC_DATA)), ensure_eigen_format(std::forward<var_type>(IN_VAR_DATAs))...);
        }
#endif


        /**
         * @brief Evaluates the function at a specified point using k-nearest points average interpolation.
         *
         * @details This function calculates the interpolated value of the function at the given point
         * @p IN_POINT using k-nearest points average interpolation technique. The distance between
         * @p IN_POINT and training data points is computed, and the mean of the k-nearest values is
         * returned. The choice of k depends on the size of the training data, and different algorithms
         * are used for efficiency.
         *
         * @param [in] IN_POINT The input point to interpolate at, represented by an Eigen::Vector<float, dimension>.
         * @return Interpolated value at the specified point.
         *
         * @throw None
         *
         * @note This method is marked as `noexcept` for better performance optimisations.
         */

#ifdef EIGEN_USE_DYNAMIC
        template <typename derived>
        requires (itp::check_eigen_dynamic<derived> && std::is_same_v<typename derived::Scalar, float>)
        float eval_at (const Eigen::MatrixBase<derived> &IN_POINT, const bool &USE_WEIGHTS = false) {
            if (IN_POINT.rows() != dimension) throw std::runtime_error("'IN_POINT' needs to be of size 'dimension'");

            return this->find_mean_of_first_minCoeff_using_nth(IN_POINT, USE_WEIGHTS);
        }
#else
        float eval_at (const Eigen::Vector<float, dimension> &IN_POINT, const bool &USE_WEIGHTS = false) noexcept {
            return this->find_mean_of_first_minCoeff_using_nth(IN_POINT, USE_WEIGHTS);    // can be replaced with find_mean_of_first_minCoeff_using_que
        }
#endif

        /**
         * @brief Evaluates the function at a specified point using k-nearest points average interpolation.
         *
         * @details This overload of the `eval_at` function allows users to provide the input point @p IN_POINT
         * as a container, either std::vector<float> or std::array<float, dimension>. It checks if the
         * size of the container matches the specified dimension and then internally maps the container
         * data to an Eigen::Vector for further interpolation avoiding additional copies with minimal performance
         * overhead.
         *
         * @tparam type The type of container, enforced at compile-time to be std::vector<float> or std::array<float, dimension>
         * @param [in] IN_POINT The input point to interpolate at, represented by a compatible container.
         * @return Interpolated value at the specified point.
         *
         * @throw std::invalid_argument Thrown if the size of the container does not match the specified dimension.
         *
         * @note This method is **NOT** marked as `noexcept`. `IN_POINT` as a std::vector has no compile-time resource to
         * get size. Thus, this overload is not marked as 'noexcept' and might be inferior in terms of performance and
         * optimisation compared to other overloads.
         */
#ifdef EIGEN_USE_DYNAMIC
        template <class type>
        requires (itp::check_vector_array<type>)
        float eval_at (type&& IN_POINT, const bool &USE_WEIGHTS = false) {
            if (std::forward<type>(IN_POINT).size() != dimension) {
                std::cerr << "Required size: " << dimension << ", Given size: " << std::forward<type>(IN_POINT).size() << std::endl;
                throw std::invalid_argument("IN_POINT must have a size of 'dimension");
            }
            return this->eval_at(static_cast<Eigen::Matrix<float, Eigen::Dynamic, 1>>(
                    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>>(std::forward<type>(IN_POINT).data(),
                                                                        std::forward<type>(IN_POINT).size())), USE_WEIGHTS);

        }

#else
        template <class type>
        requires (itp::check_vector_array<type>)
        float eval_at (type&& IN_POINT, const bool &USE_WEIGHTS = false) {
            // checks if input size matches dimension
            if (std::forward<type>(IN_POINT).size() != dimension) {
                std::cerr << "Required size: " << dimension << ", Given size: " << std::forward<type>(IN_POINT).size() << std::endl;
                throw std::invalid_argument("IN_POINT must have a size of 'dimension");
            }
            Eigen::Vector<float, dimension> ret(std::forward<type>(IN_POINT).data());
            return this->eval_at(ret, USE_WEIGHTS);
        }
#endif

        /**
         * @brief Evaluates the function at a specified point using k-nearest points average interpolation with variadic arguments.
         *
         * @details This overload of the `eval_at` function allows users to directly input the variables as variadic arguments.
         * The arguments are expected to be floating-point values, and their number must match the specified dimension.
         * The function internally forwards the arguments to an Eigen::Vector for further interpolation avoiding additional
         * copies with minimal performance overhead.
         *
         * @tparam types Variadic template for IN_POINT. Compile-time enforced to be of size `dimension` and floating-point type.
         * @param [in] IN_POINT Perfect forwarding referenced variables. Input point to interpolate at.
         * @return Interpolated value at the specified point.
         *
         * @throw None
         *
         * @note This method is marked as `noexcept` for better performance optimisations.
         */
#ifdef EIGEN_USE_DYNAMIC
        template <class... types>
        requires ((sizeof...(types) == dimension) && (meta_checks::is_number_v<types> && ...))
        float eval_at (types&&... IN_POINT) noexcept {
            return this->eval_at(std::vector<float>{std::forward<types>(IN_POINT)...}, false);
        }
#else
        template <class... types>
        requires ((sizeof...(types) == dimension) && (meta_checks::is_number_v<types> && ...))
        float eval_at (types&&... IN_POINT) noexcept {
            return this->eval_at(std::array<float, dimension>{std::forward<types>(IN_POINT)...}, false);
        }
#endif

#ifdef EIGEN_USE_DYNAMIC
        template <typename... types>
        requires ((sizeof...(types) == dimension) && (meta_checks::is_number_v<types> && ...))
        float eval_at_with_weighted_sum (types&&... IN_POINT) {
            return this->eval_at(std::vector<float>{std::forward<types>(IN_POINT)...}, true);
        }
#else
        template <class... types>
        requires ((sizeof...(types) == dimension) && (meta_checks::is_number_v<types> && ...))
        float eval_at_with_weighted_sum (types&&... IN_POINT) {
            bool keep = true;
            std::array<float, dimension> x_ = std::array<float, dimension>{std::forward<types>(IN_POINT)...};
            const float x = this->eval_at(x_, keep);
            return x;
        }
#endif
        /**
         * @brief Retrieves a shared pointer to the training points data.
         *
         * @details This method provides access to the training points data stored within an instance by returning a std::shared_ptr
         * pointing to the training points data matrix. Utilizing a shared pointer ensures safe, memory-efficient handling and sharing
         * of the training points data among different parts of the application without duplicating the data in memory. This approach
         * is particularly useful in scenarios where multiple instances need to access or modify the same underlying training
         * points data. It can be paired with set_ptr_trained_data to link different instances to a shared training points dataset.
         *
         * @return A shared pointer to the training points data matrix of the object.
         */

        auto get_ptr_trained_data () {
            return this->var_data;
        }


        /**
         * @brief Sets the training points data pointer to a new shared pointer.
         *
         * @details This method allows the current instance to point to an existing training points data set by accepting a
         * std::shared_ptr<Eigen::Matrix> as its parameter. By setting the internal training points data pointer to the provided
         * shared pointer, it facilitates the sharing of training points data among multiple instances, thereby promoting data
         * consistency and memory efficiency across different components of the system. This is particularly useful for
         * scenarios where the training points data is large or when it's beneficial for multiple instances to work on the same
         * dataset without copying the data. This method complements get_ptr_trained_data by enabling the reassignment
         * and sharing of training points data.
         *
         * @param [in] IN_POINTER A const reference to a std::shared_ptr that points to the new training points data to be used by this instance.
         */
#ifdef EIGEN_USE_DYNAMIC
        template <typename derived>
        requires (itp::check_eigen_dynamic<derived> && std::is_same_v<typename derived::Scalar, float>)
        void set_ptr_trained_data (const std::shared_ptr<derived> &IN_POINTER) {
            if (IN_POINTER->rows() != this->func_data->rows() || IN_POINTER->cols() != dimension) {
                throw std::runtime_error("Eigen Object not compatable...");
            }
            this->var_data = IN_POINTER;
        }
#else
        void set_ptr_trained_data (const std::shared_ptr<Eigen::Matrix<float, max_training_data_size, dimension>>& IN_POINTER) noexcept {
            this->var_data = IN_POINTER;
        }
#endif
        /**
         * @brief Sets the training function data pointer to a new shared pointer.
         *
         * @details This method provides access to the training function data stored within an instance by returning a std::shared_ptr
         * pointing to the training function data matrix. Utilizing a shared pointer ensures safe, memory-efficient handling and sharing
         * of the training function data among different parts of the application without duplicating the data in memory. This approach
         * is particularly useful in scenarios where multiple instances need to access or modify the same underlying training
         * points data. It can be paired with set_ptr_trained_data to link different instances to a shared training function dataset.
         *
         * @return A shared pointer to the training function data matrix of the object.
         */

        auto get_ptr_func_data () {
            return this->func_data;
        }

        /**
         * @brief Sets the training function data pointer to a new shared pointer.
         *
         * @details This method allows the current instance to point to an existing training function data set by accepting a
         * std::shared_ptr<Eigen::Matrix> as its parameter. By setting the internal training function data pointer to the provided
         * shared pointer, it facilitates the sharing of training function data among multiple instances, thereby promoting data
         * consistency and memory efficiency across different components of the system. This is particularly useful for
         * scenarios where the training function data is large or when it's beneficial for multiple instances to work on the same
         * dataset without copying the data. This method complements get_ptr_trained_data by enabling the reassignment
         * and sharing of training function data.
         *
         * @param [in] IN_POINTER A const reference to a std::shared_ptr that points to the new training functon data to be used
         * by this instance.
         */

#ifdef EIGEN_USE_DYNAMIC
        template <typename derived>
        requires (itp::check_eigen_dynamic<derived> && std::is_same_v<typename derived::Scalar, float>)
        void set_ptr_func_data (const std::shared_ptr<derived>& IN_POINTER) {
            if (IN_POINTER->rows() != this->var_data->rows() || IN_POINTER->cols() != 1) {
                throw std::runtime_error("Eigen Object not compatable...");
            }
            this->func_data = IN_POINTER;
        }
#else
        void set_ptr_func_data (const std::shared_ptr<Eigen::Matrix<float, max_training_data_size, dimension>>& IN_POINTER) noexcept {
            this->func_data = IN_POINTER;
        }
#endif

        template<typename ... types>
        requires (sizeof...(types) == dimension && (meta_checks::is_number_v<types> && ...))
        void add_major_axis_factors (types&&... IN_FACTORS) noexcept {
            this->scaling_factors = Eigen::Matrix<float, dimension, 1>(std::forward<types>(IN_FACTORS)...);
        }

#ifdef EIGEN_USE_DYNAMIC
        void reset_major_axis_factors () noexcept {
            this->scaling_factors = Eigen::Matrix<float, Eigen::Dynamic, 1>::Ones(dimension);
        }
#else
        void reset_major_axis_factors () noexcept {
            this->scaling_factors = Eigen::Matrix<float, dimension, 1>::Ones(dimension);
        }
#endif

    private:
        /** @brief Tracks number of training data inputted in `interpolate::func_data` and `interpolate::var_data` */
        std::size_t interpolated_data_size = 0;
#ifdef EIGEN_USE_DYNAMIC
        std::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, 1>> func_data;
        std::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> var_data;
        Eigen::Matrix<float, Eigen::Dynamic, 1> scaling_factors;
#else
        /** @brief Stores function values at the `interpolate::var_data` */
        std::shared_ptr<Eigen::Vector<float, max_training_data_size>> func_data;

        /** @brief Stores training data points */
        std::shared_ptr<Eigen::Matrix<float, max_training_data_size, dimension>> var_data;
        Eigen::Matrix<float, dimension, 1> scaling_factors;
#endif

        /**
         * @brief Calculate the mean of the first `mean_size` elements in `IN_FUNC` based on the minimum valued indices of
         * `IN_VALS` using `std::nth_element`
         *
         * @details This function calculates the mean of the input vector @p IN_FUNC based on the of the first `mean_size`
         * minimum values in @p IN_VALS using a std::nth_element method for efficient retrieval of the minimum values.
         * The implementation is tailored for a large data sizes (max_training_data_size).
         *
         * @param [in, out] IN_VALS Input vector containing values for index calculations. The size should
         * `max_training_data_size` and needs to available in compile time.
         * @param [in] IN_FUNC Input vector for which the mean is calculated based on selected indices.
         *
         * @return The mean value of the `IN_FUNC` elements whose indices are first `mean_size` minimum valued in IN_VALS.
         *
         * @throw None
         *
         * @note This method is marked as `noexcept` for better performance optimisations.
         * @note This version utilises a std::nth_element for efficient retrieval of minimum values from a large Eigen::Vector.
         */
#ifdef EIGEN_USE_DYNAMIC
        template <typename derived1, typename derived2>
        requires (itp::check_eigen_dynamic<derived1> && itp::check_eigen_dynamic<derived2> &&
                  std::is_same_v<typename derived1::Scalar, float> && std::is_same_v<typename derived2::Scalar, float>)
        float find_mean_of_first_minCoeff_using_nth (const Eigen::MatrixBase<derived1> &IN_VALS, const Eigen::MatrixBase<derived2> &IN_POINT,
                                                     const bool &USE_WEIGHTS = false) {
            Eigen::Matrix<float, Eigen::Dynamic, 1> result = IN_POINT;
            result = (result.array() == 0).select(0.0001, result);

            std::vector<std::size_t> indices(IN_VALS.size());
            std::iota(indices.begin(), indices.end(), 0);

            std::nth_element(indices.begin(), indices.begin() + mean_size, indices.end(),
                             [&IN_VALS] (const std::size_t i , const std::size_t j) -> bool {return IN_VALS(i) < IN_VALS(j);});

            if (USE_WEIGHTS) {
                float weighted_sum = 0.0f, weight = 0.0f;
                for (std::size_t i = 0; i < mean_size; i++) {
                    const float current_norm = (
                            (this->var_data->row(indices[i]).array() - result.transpose().array()) /
                                    result.transpose().array()
                                ).abs().matrix().norm();
                    if (current_norm == 0) [[unlikely]] {
                        return this->func_data->operator()(indices[i]);
                    }
                    weight += (1.0f / current_norm);
                    weighted_sum += (1.0f / current_norm) * this->func_data->operator()(indices[i]);
                }
                return weighted_sum / weight;
            }

            return this->func_data->operator()(std::vector<std::size_t>(indices.begin(), indices.begin() + mean_size)).sum() / static_cast<float>(mean_size);
        }

#else
        float find_mean_of_first_minCoeff_using_nth (const Eigen::Vector<float, dimension> &IN_POINT,
                                                      const bool &USE_WEIGHTS = false) {

            Eigen::Array<float, 1, dimension> result = (((IN_POINT.array() == 0).select(0.0001, IN_POINT)).transpose()).array();

            auto distance_formula_with_scaling = [&result, this] (const std::size_t &i) -> float {
                Eigen::Array<float, 1, dimension> x_ = this->var_data->row(i).array().operator-(result);
                x_ = x_.operator/(result * this->scaling_factors.transpose().array());
                return std::sqrt(x_.square().sum());
            };
            auto distance_formula = [&var_data_ = this->var_data, &result] (const std::size_t &i) -> float {
                Eigen::Array<float, 1, dimension> x_ = var_data_->row(i).array().operator/(result) - Eigen::Array<float, 1, dimension>::Ones();
                return std::sqrt(x_.square().sum());
            };
            auto compare = [&distance_formula_with_scaling] (const std::size_t &i_1, const std::size_t & i_2) -> bool {
                return distance_formula_with_scaling(i_1) < distance_formula_with_scaling(i_2);
            };

            auto que = concpt::declare::const_size_priority_deque<std::size_t>(mean_size, compare);

            for (std::size_t i = 0; i < this->interpolated_data_size; i++) {
                que.push(i);
            }

            const std::vector<std::size_t>& lowest_indices = que.get_container();

            if (USE_WEIGHTS) {
                float weighted_sum = 0.0f, weight = 0.0f;
                for (std::size_t i = 0; i < mean_size; i++) {
                    const float current_norm = distance_formula(i);
                    if (current_norm == 0) [[unlikely]] {
                        return this->func_data->operator()(lowest_indices[i]);
                    }
                    weight += (1.0f / current_norm);
                    weighted_sum += (1.0f / current_norm) * this->func_data->operator()(lowest_indices[i]);
                }
                return weighted_sum / weight;
            }

            return this->func_data->operator()(std::vector<std::size_t>(lowest_indices.begin(), lowest_indices.end())).sum() / static_cast<float>(mean_size);
        }
#endif
    };
}



#endif //CONCEPTUAL_INTERPOLATE_H