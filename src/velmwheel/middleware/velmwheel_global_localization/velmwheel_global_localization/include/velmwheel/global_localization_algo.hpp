/* ============================================================================================================================ *//**
 * @file       global_localization_algo.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 6th April 2022 7:48:15 pm
 * @modified   Thursday, 26th May 2022 12:58:34 am
 * @project    engineering-thesis
 * @brief      Set of abstract algorithms used across the package
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __VELMWHEEL_GLOBAL_LOCALIZATION_ALGO_H__
#define __VELMWHEEL_GLOBAL_LOCALIZATION_ALGO_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <algorithm>
#include <optional>
#include <vector>
#include <utility>

/* ========================================================== Namespaces ========================================================== */

namespace velmwheel {
namespace algorithm {

/* ============================================================ Traits ============================================================ */

/**
 * @brief Helper alias providing type of the value returned by @tparam Transform 
 *    called with constant reference to @tparam Elem object
 * 
 * @tparam Transform 
 *    type of some transform callable
 * @tparam Elem 
 *    type of the element that the metric can be called on
 */
template<typename Tranform, typename Elem>
using TransformValue = std::invoke_result_t<Tranform, const Elem&>;

/**
 * @brief Helper alias providing type of the value returned by @tparam Metric 
 *    called with two constant references to @tparam Elem objects
 * 
 * @tparam Metric 
 *    type of some metric callable
 * @tparam Elem 
 *    type of the element that the metric can be called on
 */
template<typename Metric, typename Elem>
using MetricValue = std::invoke_result_t<Metric, const Elem&, const Elem&>;

/* ========================================================== Data types ========================================================== */

/**
 * @brief Auxiliary structure describing a corresponding entity unambiguously
 *    identified by integer-typed ID and value of some metric computed on it
 */
template<typename Metric>
struct CorrespondingElement {

    /// Type of the metric used
    using MetricT = Metric;

    /// Index of the entity
    std::size_t idx;
    /// Metric taken on entitis
    Metric metric;

};

/**
 * @brief Auxiliary structure describing pair of corresponding entities unambiguously
 *    identified by integer-typed IDs and value of some metric computed on both of them
 */
template<typename Metric>
struct Correspondance {

    /// Type of the metric used
    using MetricT = Metric;

    /// Index of the source entity
    std::size_t source_idx;
    /// Index of the target entity
    std::size_t target_idx;
    /// Metric taken on entitis
    Metric metric;

};

/**
 * @brief Helper alias providing type of the metric associated used by either 
 *    @ref CorrespondingElement or @ref Correspondance structure returned by 
 *    some callable of type BestMatch when called with <i>(const &Elem)</i> 
 *    argument
 *    
 * @tparam BestMatch 
 *    type of some some callable
 * @tparam Elem 
 *    type of the element taken by the callable
 */
template<typename BestMatch, typename Elem>
using BestMatchMetric = typename std::invoke_result_t<BestMatch, const Elem&>::MetricT;

/* ======================================================= General functions ====================================================== */

/**
 * @brief Functor structure providing identity transformation for an arbitrary object
 */
struct identity {

    /**
     * @returns 
     *    std::forward<T>(t)
     */
    template<class T>
    constexpr T&& operator()( T&& t ) const noexcept;

};

/**
 * @brief Finds the first element in the @p [first,last) range for which value of @p transform(*(it+1))
 *    element is greater than @p transform(*it)
 * 
 * @tparam InputIt 
 *    type of the iterator
 * @tparam Compare 
 *    type of the callable used to compare transformed elements
 * @tparam Transform 
 *    type of the callable transforming input elements
 * @param first 
 *    first element in the range (inclusive)
 * @param last 
 *    last element in the range (exclusive)
 * @param compare 
 *    callable to be used compare transformed elements
 * @param transform 
 *    transformation to be used on elements before comparison
 * 
 * @retval iterator
 *   iterator to the first element for which value of @p transform(*it) element is lesser than
 *   @p transform(*(it-1)) on success
 * @retval last 
 *    on failure
 */
template<typename InputIt, typename Compare, typename Transform>
InputIt find_first_minimum(
    InputIt first,
    InputIt last,
    Compare compare = std::less<TransformValue<Transform, typename std::iterator_traits<InputIt>::value_type>>{},
    Transform transform = identity{}
);

/* ====================================================== Computing functions ===================================================== */

/**
 * @brief For each element in the @p elements vector computes the @p metric on this element
 *    and @p value
 * 
 * @tparam Elem 
 *    type of elements in the @p elements
 * @tparam Metric 
 *    type of the @p metric callable
 * @param elements 
 *    list of elements to compute metrics on
 * @param value 
 *    value to calculate metrics for
 * @param metric 
 *    callable used to compute metric on two elements
 * 
 * @returns 
 *    vector containing metrics computed on the @p value and all elements in the @p elements
 *    vector; i.e. for returned vector @p ret , @p ret[i] contains metric on the @p value
 *    element and the the @p elements[i] element
 */
template<typename Elem, typename Metric>
std::vector<MetricValue<Metric, Elem>> compute_metrics(
    const std::vector<Elem> &elements,
    const Elem &value,
    Metric metric
);

/**
 * @brief For each element in the @p elements vector computes the @p metric on this element
 *    and @p value . Sorts resulting vector in order of non-descending metrics
 * 
 * @tparam Elem 
 *    type of elements in the @p elements
 * @tparam Metric 
 *    type of the @p metric callable
 * @tparam Compare 
 *    type of the @p compare callable
 * @param elements 
 *    list of elements to compute metrics on
 * @param value 
 *    value to calculate metrics for
 * @param metric 
 *    callable used to compute metric on two elements
 * @param compare 
 *    callable used to compare two metrics
 * 
 * @returns 
 *    vector containing structures holding metrics computed on the @p value and subsequent
 *    elements in the @p elements vector along with indeces of these elements; vector is sorted
 *    in order of non-descending metrics
 */
template<typename Elem, typename Metric, typename Compare>
std::vector<CorrespondingElement<MetricValue<Metric, Elem>>> compute_metrics_sorted(
    const std::vector<Elem> &elements,
    const Elem &value,
    Metric metric,
    Compare compare = std::less<MetricValue<Metric, Elem>>{}
);

/**
 * @brief For each element in the @p elements vector computes a vector containing value of 
 *    the @p metric calculated on this element and all elements in the vector (including itself)
 * 
 * @tparam Elem 
 *    type of elements in the @p elements
 * @tparam Metric 
 *    type of the @p metric callable
 * @param elements 
 *    list of elements to compute metrics on
 * @param metric 
 *    callable used to compute metric on two elements
 * 
 * @returns 
 *    vector of vectors containing metrics on the given element and all elements in the @p elements
 *    vector; i.e. for returned vector @p ret , @p ret[i][j] contains metric on the @p elements[i] 
 *    element and the the @p elements[j] element
 */
template<typename Elem, typename Metric>
std::vector<std::vector<MetricValue<Metric, Elem>>> compute_all_metrics(
    const std::vector<Elem> &elements,
    Metric metric
);

/**
 * @brief For each element in the @p elements vector computes a vector containing value of the @p metric
 *    calculated on this element and all elements in the vector (including itself). Sorts resulting vectors
 *    in order of non-descending metrics
 * 
 * @tparam Elem 
 *    type of elements in the @p elements
 * @tparam Metric 
 *    type of the @p metric callable
 * @tparam Compare 
 *    type of the @p compare callable
 * @param elements 
 *    list of elements to compute metrics on
 * @param metric 
 *    callable used to compute metric on two elements
 * @param compare 
 *    callable used to compare two metrics
 * 
 * @returns 
 *    vector of vectors containing structures holding metrics computed on subsequent elements of the
 *    @p elements and all elements of the @p elements vector (including itself) along with indeces of these
 *    elements; vector is sorted in order of non-descending metrics
 */
template<typename Elem, typename Metric, typename Compare>
std::vector<CorrespondingElement<MetricValue<Metric, Elem>>> compute_all_metrics_sorted(
    const std::vector<Elem> &elements,
    Metric metric,
    Compare compare = std::less<MetricValue<Metric, Elem>>{}
);

/* ============================================= Matching functions (overlapped match) ============================================ */

/**
 * @brief Matches each element of the @p source vector to elements found by @p best_match callable
 *    for which metric is not greater than @p max_metric . At each matc calls @p accumulator 
 *    with the correspondance-describing structure. Function allows matching single target element
 *    ( identified by the @a idx of the structure returned by the @p best_match ) to many @p source 
 *    elements.
 * 
 * @tparam Elem 
 *    type of elements held by vectors
 * @tparam BestMatch 
 *    type of the @p best_match callable
 * @tparam Accumulator 
 *    type of the callable entity called at each match of the elements taking 
 *    <i>(const Correspondance<MetricValue<Metric, Elem>> &)</i> reference as a call argument
 * @tparam Transform 
 *    type of the callable entity used on each @p source element before calculating metric
 * @tparam Compare 
 *    type of the callable entity entity used to compare 
 * @param source 
 *    set of source elements to be matched against @p target elements
 * @param best_match 
 *    callable that called with the <i>(std::size_t idx,const Elem &elem)</i> returns 
 *    CorrespondingElement<BestMatchMetric<BestMatch, Elem>> describing the corresponding element
 * @param accumulator 
 *    callable entity called at each match of the elements taking <i>(const Correspondance<MetricValue<Metric, Elem>> &)</i>
 *    reference as a call argument
 * @param max_metric 
 *    maximal value of the metric that allows two elements to be matched
 * @param transform 
 *    callable entity used on each @p source element before calculating metric
 * @param compare 
 *    entity used to compare metrics
 */
template<
    typename Elem,
    typename BestMatch,
    typename Accumulator,
    typename Transform = identity,
    typename Compare = std::less<BestMatchMetric<BestMatch, Elem>>
> void match_overlapping(
    const std::vector<Elem> &source,
    BestMatch best_match,
    Accumulator accumulator,
    const std::optional<BestMatchMetric<BestMatch, Elem>> &max_metric = std::optional<BestMatchMetric<BestMatch, Elem>>{},
    Transform transform = identity{},
    Compare compare = std::less<BestMatchMetric<BestMatch, Elem>>{}
);

/**
 * @brief Matches each element of the @p source vector to elements found by @p best_match callable
 *    for which metric is not greater than @p max_metric . Returns vector of structures descring 
 *    found correspondances. Function allows matching single target element ( identified by the
 *    @a idx of the structure returned by the @p best_match ) to many @p source  elements.
 * 
 * @tparam Elem 
 *    type of elements held by vectors
 * @tparam BestMatch 
 *    type of the @p best_match callable
 * @tparam Transform 
 *    type of the callable entity used on each @p source element before calculating metric
 * @tparam Compare 
 *    type of the callable entity entity used to compare 
 * @param source 
 *    set of source elements to be matched against @p target elements
 * @param best_match 
 *    callable that called with the <i>(const &Elem)</i> returns CorrespondingElement<BestMatchMetric<BestMatch, Elem>>
 *    describing the corresponding element
 * @param metric 
 *    given metric
 * @param max_metric 
 *    maximal value of the metric that allows two elements to be matched
 * @param transform 
 *    callable entity used on each @p source element before calculating metric
 * @param compare 
 *    entity used to compare metrics
 * 
 * @returns 
 *    vector of structures holding pairs of indeces of of matched
 */
template<
    typename Elem, 
    typename BestMatch, 
    typename Transform = identity, 
    typename Compare = std::less<BestMatchMetric<BestMatch, Elem>>
> std::vector<Correspondance<BestMatchMetric<BestMatch, Elem>>> match_overlapping(
    const std::vector<Elem> &source,
    BestMatch best_match,
    const std::optional<BestMatchMetric<BestMatch, Elem>> &max_metric = std::optional<BestMatchMetric<BestMatch, Elem>>{},
    Transform transform = identity{},
    Compare compare = std::less<BestMatchMetric<BestMatch, Elem>>{}
);

/**
 * @brief Matches each element of the @p source vector to such an element from the @p target 
 *    vector for which value of the @p metric(source,target) is the smallest. On each match the 
 *    @p accumulator callable is called with a structure describing found match as an argument
 * 
 * @tparam Elem 
 *    type of elements held by vectors
 * @tparam Metric 
 *    type of the callable metric entity taking  <i>(const Elem&, const Elem&)</i>
 * @tparam Accumulator 
 *    type of the callable entity called at each match of the elements taking 
 *    <i>(const Correspondance<MetricValue<Metric, Elem>> &)</i> reference as a call argument
 * @tparam Transform 
 *    type of the callable entity used on each @p source element before calculating metric
 * @tparam Compare 
 *    type of the callable entity entity used to compare 
 * @param source 
 *    set of source elements to be matched against @p target elements
 * @param target 
 *    set of target elements to be @p source elements are matched against
 * @param metric 
 *    given metric
 * @param accumulator 
 *    callable entity called at each match of the elements taking <i>(const Correspondance<MetricValue<Metric, Elem>> &)</i>
 *    reference as a call argument
 * @param max_metric 
 *    maximal value of the metric that allows two elements to be matched
 * @param transform 
 *    callable entity used on each @p source element before calculating metric
 * @param compare 
 *    entity used to compare metrics
 */
template<
    typename Elem, 
    typename Metric, 
    typename Accumulator, 
    typename Transform = identity, 
    typename Compare = std::less<MetricValue<Metric, Elem>>
> void match_overlapping_unordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    Accumulator accumulator,
    const std::optional<MetricValue<Metric, Elem>> &max_metric = std::optional<MetricValue<Metric, Elem>>{},
    Transform transform = identity{},
    Compare compare = std::less<MetricValue<Metric, Elem>>{}
);

/**
 * @brief Matches each element of the @p source vector to such an element from the @p target 
 *    vector for which value of the @p metric(source,target) is the smallest
 * 
 * @tparam Elem 
 *    type of elements held by vectors
 * @tparam Metric 
 *    type of the callable metric entity taking  <i>(const Elem&, const Elem&)</i>
 * @tparam Transform 
 *    type of the callable entity used on each @p source element before calculating metric
 * @tparam Compare 
 *    type of the callable entity entity used to compare 
 * @param source 
 *    set of source elements to be matched against @p target elements
 * @param target 
 *    set of target elements to be @p source elements are matched against
 * @param metric 
 *    given metric
 * @param max_metric 
 *    maximal value of the metric that allows two elements to be matched
 * @param transform 
 *    callable entity used on each @p source element before calculating metric
 * @param compare 
 *    entity used to compare metrics
 * 
 * @returns 
 *    vector of structures holding pairs of indeces of of matched
 */
template<
    typename Elem, 
    typename Metric, 
    typename Transform = identity, 
    typename Compare = std::less<MetricValue<Metric, Elem>>
> std::vector<Correspondance<MetricValue<Metric, Elem>>> match_overlapping_unordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    const std::optional<MetricValue<Metric, Elem>> &max_metric = std::optional<MetricValue<Metric, Elem>>{},
    Transform transform = identity{},
    Compare compare = std::less<MetricValue<Metric, Elem>>{}
);

/**
 * @brief Matches each element of the @p source vector to such an element from the @p target 
 *    vector for which value of the @p metric(source,target) is the smallest. On each match the 
 *    @p accumulator callable is called with a structure describing found match as an argument.
 *    
 *    Function assumes that for each @a source_elem the vector resulting from transforming 
 *    @p target vector with @p metric(source_elem,target[j]) operation contains exactly one
 *    minimum (although this does not need to be strict minimum) value.
 * 
 * @tparam Elem 
 *    type of elements held by vectors
 * @tparam Metric 
 *    type of the callable metric entity taking  <i>(const Elem&, const Elem&)</i>
 * @tparam Accumulator 
 *    type of the callable entity called at each match of the elements taking 
 *    <i>(const Correspondance<MetricValue<Metric, Elem>> &)</i> reference as a call argument
 * @tparam Transform 
 *    type of the callable entity used on each @p source element before calculating metric
 * @tparam Compare 
 *    type of the callable entity entity used to compare 
 * @param source 
 *    set of source elements to be matched against @p target elements
 * @param target 
 *    set of target elements to be @p source elements are matched against
 * @param metric 
 *    given metric
 * @param accumulator 
 *    callable entity called at each match of the elements taking <i>(const Correspondance<MetricValue<Metric, Elem>> &)</i>
 *    reference as a call argument
 * @param max_metric 
 *    maximal value of the metric that allows two elements to be matched
 * @param transform 
 *    callable entity used on each @p source element before calculating metric
 * @param compare 
 *    entity used to compare metrics
 */
template<
    typename Elem, 
    typename Metric, 
    typename Accumulator, 
    typename Transform = identity, 
    typename Compare = std::less<MetricValue<Metric, Elem>>
> void match_overlapping_ordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    Accumulator accumulator,
    const std::optional<MetricValue<Metric, Elem>> &max_metric = std::optional<MetricValue<Metric, Elem>>{},
    Transform transform = identity{},
    Compare compare = std::less<MetricValue<Metric, Elem>>{}
);

/**
 * @brief Matches each element of the @p source vector to such an element from the @p target 
 *    vector for which value of the @p metric(source,target) is the smallest. 
 *    
 *    Function assumes that for each @a source_elem the vector resulting from transforming 
 *    @p target vector with @p metric(source_elem,target[j]) operation contains exactly one
 *    minimum (although this does not need to be strict minimum) value.
 * 
 * @tparam Elem 
 *    type of elements held by vectors
 * @tparam Metric 
 *    type of the callable metric entity taking  <i>(const Elem&, const Elem&)</i>
 * @tparam Transform 
 *    type of the callable entity used on each @p source element before calculating metric
 * @tparam Compare 
 *    type of the callable entity entity used to compare 
 * @param source 
 *    set of source elements to be matched against @p target elements
 * @param target 
 *    set of target elements to be @p source elements are matched against
 * @param metric 
 *    given metric
 * @param max_metric 
 *    maximal value of the metric that allows two elements to be matched
 * @param transform 
 *    callable entity used on each @p source element before calculating metric
 * @param compare 
 *    entity used to compare metrics
 * 
 * @returns 
 *    vector of structures holding pairs of indeces of of matched
 */
template<
    typename Elem,
    typename Metric,
    typename Transform = identity,
    typename Compare = std::less<MetricValue<Metric, Elem>>
> std::vector<Correspondance<MetricValue<Metric, Elem>>> match_overlapping_ordered(
    const std::vector<Elem> &source,
    const std::vector<Elem> &target,
    Metric metric,
    const std::optional<MetricValue<Metric, Elem>> &max_metric = std::optional<MetricValue<Metric, Elem>>{},
    Transform transform = identity{},
    Compare compare = std::less<MetricValue<Metric, Elem>>{}
);

/* =========================================== Matching functions (non-overlapped match) ========================================== */

/**
 * @todo Think through implementation of non-overlaping methods of elements matching
 */

/* ================================================================================================================================ */

} // End namespace algorithm
} // End namespace velmwheel

/* ==================================================== Implementation includes =================================================== */

#include "velmwheel/impl/global_localization_algo.hpp"

/* ================================================================================================================================ */

#endif