/* ============================================================================================================================ *//**
 * @file       locks.hpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Friday, 22nd April 2022 1:09:36 am
 * @modified   Thursday, 28th April 2022 5:44:54 pm
 * @project    engineering-thesis
 * @brief      Declarations of helper locking classes
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#ifndef __CIFX_ETHERCAT_COMMON_LOCKS_H__
#define __CIFX_ETHERCAT_COMMON_LOCKS_H__

/* =========================================================== Includes =========================================================== */

// System includes
#include <atomic>

/* ========================================================== Namespaces ========================================================== */

namespace cifx {
namespace ethercat {
namespace locks {

/* ========================================================== Empty lock ========================================================== */

/**
 * @brief Auxiliary class implementing stub for locking object
 */
struct EmptyLock {

    // Stub for lock() method
    inline void lock() {}

    // Stub for unlock() method
    inline void unlock() {}

};

/* =========================================================== Spinlock =========================================================== */

/**
 * @brief Auxiliary class implementing atomic-based spinlock
 * 
 */
class SpinLock {
private:

    // State of the lock
    enum class State : bool { 
        Locked,
        Unlocked
    };

public:

    /// Constructs the spinlock
    SpinLock() : state(State::Unlocked) {}

    /// Locks the spinlock
    void lock() { while (state.exchange(State::Locked, std::memory_order_acquire) == State::Locked) {} }
    
    /// Unlocks the spinlock
    void unlock() { state.store(State::Unlocked, std::memory_order_release); }

private:

    // State of the lock
    std::atomic<State> state;
    
};

/* ================================================================================================================================ */

} // End namespace locks
} // End namespace ethercat
} // End namespace cifx

#endif
