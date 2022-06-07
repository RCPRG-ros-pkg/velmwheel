/* ============================================================================================================================ *//**
 * @file       global_localization_algo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 6th April 2022 7:48:15 pm
 * @modified   Thursday, 26th May 2022 12:59:09 am
 * @project    engineering-thesis
 * @brief      Set of abstract algorithms used across the package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GLOBAL_LOCALIZATION_IMPL_ALGO_H__
#define __VELMWHEEL_GLOBAL_LOCALIZATION_IMPL_ALGO_H__

/* =========================================================== Includes =========================================================== */

#include "velmwheel/global_localization_algo.hpp"

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace algorithm {

/* ========================================================= Helper types ========================================================= */

namespace details {

    /**
     * @brief Helper functor type holding reference to the std::vector object 
     *    and pushing back element to this vector at each call
     * 
     * @tparam T 
     *    type of elements held in references vector
     */
    template<typename T>
    class Pusher {
    public:

        /// Initialize object with the reference to some std::vector
        explicit Pusher(std::vector<T> &vec) : vec { vec } { }

        /// Pushes element to the referenced vector
        void operator()(const T &t) { vec.push_back(t); }

    private:

        /// Reference to some std::vector
        std::vector<T> &vec;

    };

}

/* ======================================================= General functions ====================================================== */

template<class T>
constexpr T&& identity::operator()( T&& t ) const noexcept {
    return std::forward<T>(t);
}


template<typename InputIt, typename Compare, typename Transform>
InputIt find_first_minimum(
    InputIt first,
    InputIt last,
    Compare compare,
    Transform transform
) {

    InputIt it = first;

    // Get transform of the first element
    auto last_value = transform(*it);

    // Find first grow
    while(++it != last) {

        // Get transform of the current element
        auto value = transform(*it);
        // If the value if greater than the previous one, return
        if(compare(last_value, value))
            return --it;
    }

    return it;
}

/* ====================================================== Computing functions ===================================================== */

template<typename Elem, typename Metric>
std::vector<MetricValue<Metric, Elem>> compute_metrics(
    const std::vector<Elem> &elements,
    const Elem &value,
    Metric metric
) {
    std::vector<MetricValue<Metric, Elem>> ret;

    // If vector is empty, return
    if(elements.empty())
        return ret;
    
    // Calculate metric for each element
    std::transform(elements.begin(), elements.end(), ret.begin(),
        [&value, &metric](const Elem &e) {
            return metric(value, e);
        }
    );

    // Return resulting vector
    return ret;
}


template<typename Elem, typename Metric, typename Compare>
std::vector<CorrespondingElement<MetricValue<Metric, Elem>>> compute_metrics_sorted(
    const std::vector<Elem> &elements,
    const Elem &value,
    Metric metric,
    Compare compare
) {

    std::vector<CorrespondingElement<MetricValue<Metric, Elem>>> ret;

    // If vector is empty, return
    if(elements.empty())
        return ret;
        
    // Reserve memory for result
    ret.reserve(elements.size());

    // Calculate metric for each element
    for(unsigned i = 0; i < elements.size(); ++i) {
        ret.push_back(CorrespondingElement<MetricValue<Metric, Elem>>{
            .idx    = i,
            .metric = metric(value, elements[i])
        });
    }

    // Sort the resulting vector
    std::sort(ret.begin(), ret.end(), 
        [&compare](
            const CorrespondingElement<MetricValue<Metric, Elem>> &a,
            const CorrespondingElement<MetricValue<Metric, Elem>> &b
        ) {
            return compare(a, b);
        }
    );
                
    // Return resulting vector
    return ret;
}


template<typename Elem, typename Metric>
std::vector<std::vector<MetricValue<Metric, Elem>>> compute_all_metrics(
    const std::vector<Elem> &elements,
    Metric metric
) {

    std::vector<std::vector<CorrespondingElement<MetricValue<Metric, Elem>>>> ret;

    // If vector is empty, return
    if(elements.empty())
        return ret;
        
    // Reserve memory for result
    ret.reserve(elements.size());
    
    // Calculate all metrics for each element
    std::transform(elements.begin(), elements.end(), ret.begin(),
        [&elements, &metric](const Elem &e) {
            return compute_metrics(elements, e, metric);
        }
    );

    // Return resulting vector
    return ret;
}


template<typename Elem, typename Metric, typename Compare>
std::vector<CorrespondingElement<MetricValue<Metric, Elem>>> compute_all_metrics_sorted(
    const std::vector<Elem> &elements,
    Metric metric,
    Compare compare
) {
    std::vector<std::vector<CorrespondingElement<MetricValue<Metric, Elem>>>> ret;

    // If vector is empty, return
    if(elements.empty())
        return ret;
        
    // Reserve memory for result
    ret.reserve(elements.size());
    
    // Calculate all metrics for each element
    std::transform(elements.begin(), elements.end(), ret.begin(),
        [&elements, &metric, &compare](const Elem &e) {
            return compute_metrics_sorted(elements, e, metric, compare);
        }
    );

    // Return resulting vector
    return ret;
}

/* ============================================= Matching functions (overlapped match) ============================================ */

template<typename Elem, typename BestMatch, typename Accumulator, typename Transform, typename Compare> 
void match_overlapping(
    const std::vector<Elem> &source,
    BestMatch best_match,
    Accumulator accumulator,
    const std::optional<BestMatchMetric<BestMatch, Elem>> &max_metric,
    Transform transform,
    Compare compare
) {
    // Iterate over source elements to find match for them
    for(std::size_t i = 0; i < source.size(); ++i) {

        // Transform the element
        Elem source_elem_transformed = transform(source[i]);

        // Find the best match target vector
        auto best_match_ret = best_match(i, source_elem_transformed);
        CorrespondingElement<BestMatchMetric<BestMatch, Elem>> best_match = best_match_ret;
        
        /**
         * @note The strange two-step resolve of the @ref best_match output of the
         *    callable is required to properly resolve return type by the compiler.
         *    This is because the return type of the callable depends on the type of 
         *    callable itself
         */

        // Check if match is acceptable; if so, add element to the matched set
        if(not max_metric.has_value() or best_match.metric <= *max_metric) {
            accumulator(Correspondance<BestMatchMetric<BestMatch, Elem>>{
                .source_idx = i,
                .target_idx = best_match.idx,
                .metric     = best_match.metric
            });
        }

    }
}


template<typename Elem, typename BestMatch, typename Transform, typename Compare>
std::vector<Correspondance<BestMatchMetric<BestMatch, Elem>>> match_overlapping(
    const std::vector<Elem> &source,
    BestMatch best_match,
    const std::optional<BestMatchMetric<BestMatch, Elem>> &max_metric,
    Transform transform,
    Compare compare
) {

    std::vector<Correspondance<BestMatchMetric<BestMatch, Elem>>> ret;

    // Match elements
    match_overlapping(
        source,
        std::forward<BestMatch>(best_match),
        details::Pusher(ret),
        max_metric,
        std::forward<Transform>(transform),
        std::forward<Compare>(compare)
    );

    return ret;
}


template<typename Elem, typename Metric, typename Accumulator, typename Transform, typename Compare>
void match_overlapping_unordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    Accumulator accumulator,
    const std::optional<MetricValue<Metric, Elem>> &max_metric,
    Transform transform,
    Compare compare
) {
    // If any of vectors is empty, return
    if(source.empty() or target.empty())
        return;

    // Helper functor implementing specific method of finding the best match for source element
    auto best_match = 
        [&target, &metric, &compare](
            [[maybe_unused]] std::size_t idx,
            const Elem &source_elem_transformed
        ) {
            
            // Find the best match
            auto best_match_it = std::min_element(target.begin(), target.end(), 
                [&source_elem_transformed, &metric, &compare](
                    const Elem &a,
                    const Elem &b
                ) {
                    return compare(
                        metric(source_elem_transformed, a),
                        metric(source_elem_transformed, b)
                    );
                }
            );

            // Return result
            return CorrespondingElement<MetricValue<Metric, Elem>> {
                .idx    = static_cast<std::size_t>(std::distance(target.begin(), best_match_it)),
                .metric = metric(source_elem_transformed, *best_match_it)
            };
        };

    // Call more generic implementation
    match_overlapping(
        source,
        best_match,
        std::forward<Accumulator>(accumulator),
        max_metric,
        std::forward<Transform>(transform),
        std::forward<Compare>(compare)
    );
}


template<typename Elem, typename Metric, typename Transform, typename Compare>
std::vector<Correspondance<MetricValue<Metric, Elem>>> match_overlapping_unordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    const std::optional<MetricValue<Metric, Elem>> &max_metric,
    Transform transform,
    Compare compare
) {

    std::vector<Correspondance<MetricValue<Metric, Elem>>> ret;

    // Match elements
    match_overlapping_unordered(
        source,
        target,
        std::forward<Metric>(metric),
        details::Pusher(ret),
        max_metric,
        std::forward<Transform>(transform),
        std::forward<Compare>(compare)
    );

    return ret;
}


template<typename Elem, typename Metric, typename Accumulator, typename Transform, typename Compare>
void match_overlapping_ordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    Accumulator accumulator,
    const std::optional<MetricValue<Metric, Elem>> &max_metric,
    Transform transform,
    Compare compare
) {
    // If any of vectors is empty, return
    if(source.empty() or target.empty())
        return;
        
    // Helper functor implementing specific method of finding the best match for source element
    auto best_match = 
        [&target, &metric, &compare](
            [[maybe_unused]] std::size_t idx,
            const Elem &source_elem_transformed
        ) {
            // Find the best match
            auto best_match_it = find_first_minimum(target.begin(), target.end(), compare,
                [&metric, &source_elem_transformed](const Elem &a) {
                    return metric(source_elem_transformed, a);
                }
            );

            // Return result
            return CorrespondingElement<MetricValue<Metric, Elem>> {
                .idx    = static_cast<std::size_t>(std::distance(target.begin(), best_match_it)),
                .metric = metric(source_elem_transformed, *best_match_it)
            };
        };

    // Call more generic implementation
    match_overlapping(
        source,
        best_match,
        std::forward<Accumulator>(accumulator),
        max_metric,
        std::forward<Transform>(transform),
        std::forward<Compare>(compare)
    );
}


template<typename Elem, typename Metric, typename Transform, typename Compare>
std::vector<Correspondance<MetricValue<Metric, Elem>>> match_overlapping_ordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    const std::optional<MetricValue<Metric, Elem>> &max_metric,
    Transform transform,
    Compare compare
) {

    std::vector<Correspondance<MetricValue<Metric, Elem>>> ret;

    // Match elements
    match_overlapping_ordered(
        source,
        target,
        std::forward<Metric>(metric),
        details::Pusher(ret),
        max_metric,
        std::forward<Transform>(transform),
        std::forward<Compare>(compare)
    );

    return ret;
}

/* ================================================================================================================================ */

} // End namespace algorithm
} // End namespace velmwheel

#endif
