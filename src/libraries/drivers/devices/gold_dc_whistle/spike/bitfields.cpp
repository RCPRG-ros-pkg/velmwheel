/* ============================================================================================================================ *//**
 * @file       bitfields.cpp
 * @author     Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Wednesday, 1st June 2022 7:14:09 pm
 * @modified   Thursday, 2nd June 2022 2:40:41 pm
 * @project    engineering-thesis
 * @brief      Example program comparing packed bitfields and std::bitset. Tested on GCC 9.4.0
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

#include <iostream>
#include <cstdint>
#include <bitset>

/* =========================================================== Bitfield =========================================================== */

struct [[gnu::packed]] bitfield {
    
    uint8_t a : 1;
    uint8_t b : 1;
    uint8_t c : 1;
    uint8_t d : 1;
    uint8_t e : 1;
    uint8_t f : 1;
    uint8_t g : 1;
    uint8_t h : 1;

    // Requires custom asignment
    bitfield &operator=(uint8_t value) { 
        a = bool(value & (1U << 0));
        b = bool(value & (1U << 1));
        c = bool(value & (1U << 2));
        d = bool(value & (1U << 3));
        e = bool(value & (1U << 4));
        f = bool(value & (1U << 5));
        g = bool(value & (1U << 6));
        h = bool(value & (1U << 7));
        return *this;
    }

    bitfield() = default;

    // Requires custom constructor
    bitfield(uint8_t value) { *this = value; }

} bits;

/// Requires custom printer
std::ostream &operator<<(std::ostream & stream, const bitfield &bf) {
    stream << static_cast<unsigned>(bf.h)
           << static_cast<unsigned>(bf.g)
           << static_cast<unsigned>(bf.f)
           << static_cast<unsigned>(bf.e)
           << static_cast<unsigned>(bf.d)
           << static_cast<unsigned>(bf.c)
           << static_cast<unsigned>(bf.b)
           << static_cast<unsigned>(bf.a);
    return stream;
};

/* =========================================================== Bitfield =========================================================== */

union [[gnu::packed]] unibitfield {
    uint8_t value;
    struct {
        
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t c : 1;
        uint8_t d : 1;
        uint8_t e : 1;
        uint8_t f : 1;
        uint8_t g : 1;
        uint8_t h : 1;

    } bits;

    // Requires custom asignment
    unibitfield &operator=(uint8_t value) { this->value = value; return *this; }

};

/// Requires custom printer
std::ostream &operator<<(std::ostream & stream, const unibitfield &bf) { stream << std::bitset<8>(bf.value); return stream; }

/* ============================================================ Bitset ============================================================ */

template<typename Enum, std::size_t N = sizeof(Enum) * 8>
class named_bitsed : public std::bitset<N> {
public:
    using std::bitset<N>::bitset;
    typename std::bitset<N>::reference operator[](Enum field)       { return std::bitset<N>::operator[](static_cast<std::size_t>(field)); }
    bool                               operator[](Enum field) const { return std::bitset<N>::operator[](static_cast<std::size_t>(field)); }
};

enum class Field : std::size_t {
    a = 0,
    b = 1,
    c = 2,
    d = 3,
    e = 4,
    f = 5,
    g = 6,
    h = 7
};

/**
 * @note In fact only the bitset is C++-conformant. @ref bitfield and @ref unibitfield 
 *    are implementation-specific and rely on [[gnu::packed]] attribute
 */
struct bitset : public named_bitsed<Field, 8> { using named_bitsed<Field, 8>::operator=; };

/* ============================================================= Main ============================================================= */

int main(int argc, char const *argv[])
{
    // Compare sizes
    std::cout << "Size" << std::endl;
    std::cout << " - bitfield    : " << 8 * sizeof(bitfield)    << std::endl;
    std::cout << " - unibitfield : " << 8 * sizeof(unibitfield) << std::endl;
    std::cout << " - bitset      : " << 8 * sizeof(bitset)      << std::endl;

    // Initial value
    uint8_t init = 0b01100010;

    // Initialize objects
    bitfield    bf{ init };
    unibitfield ubf{ init };
    bitset      bs{ init };

    // Compare Initial content
    std::cout << "Initial content"            << std::endl;
    std::cout << " - bitfield    : 0b" << bf  << std::endl;
    std::cout << " - unibitfield : 0b" << ubf << std::endl;
    std::cout << " - bitset      : 0b" << bs  << std::endl;

    uint8_t value = 0b11110000;

    // Reassign value
    bf  = value;
    ubf = value;
    bs  = value;

    // Compare post-asignment content
    std::cout << "Post-asignment content"       << std::endl;
    std::cout << " - bitfield    : 0b" << bf  << std::endl;
    std::cout << " - unibitfield : 0b" << ubf << std::endl;
    std::cout << " - bitset      : 0b" << bs  << std::endl;

    // Compare Initial content
    std::cout << "Field content"                                             << std::endl;
    std::cout << " - bitfield[a]    : " << static_cast<unsigned>(bf.a)       << std::endl;
    std::cout << " - bitfield[b]    : " << static_cast<unsigned>(bf.b)       << std::endl;
    std::cout << " - bitfield[c]    : " << static_cast<unsigned>(bf.c)       << std::endl;
    std::cout << " - bitfield[d]    : " << static_cast<unsigned>(bf.d)       << std::endl;
    std::cout << " - bitfield[e]    : " << static_cast<unsigned>(bf.e)       << std::endl;
    std::cout << " - bitfield[f]    : " << static_cast<unsigned>(bf.f)       << std::endl;
    std::cout << " - bitfield[g]    : " << static_cast<unsigned>(bf.g)       << std::endl;
    std::cout << " - bitfield[h]    : " << static_cast<unsigned>(bf.h)       << std::endl;
    std::cout << " - unibitfield[a] : " << static_cast<unsigned>(ubf.bits.a) << std::endl;
    std::cout << " - unibitfield[b] : " << static_cast<unsigned>(ubf.bits.b) << std::endl;
    std::cout << " - unibitfield[c] : " << static_cast<unsigned>(ubf.bits.c) << std::endl;
    std::cout << " - unibitfield[d] : " << static_cast<unsigned>(ubf.bits.d) << std::endl;
    std::cout << " - unibitfield[e] : " << static_cast<unsigned>(ubf.bits.e) << std::endl;
    std::cout << " - unibitfield[f] : " << static_cast<unsigned>(ubf.bits.f) << std::endl;
    std::cout << " - unibitfield[g] : " << static_cast<unsigned>(ubf.bits.g) << std::endl;
    std::cout << " - unibitfield[h] : " << static_cast<unsigned>(ubf.bits.h) << std::endl;
    std::cout << " - bitset[a]      : " << bs[Field::a]                      << std::endl;
    std::cout << " - bitset[b]      : " << bs[Field::b]                      << std::endl;
    std::cout << " - bitset[c]      : " << bs[Field::c]                      << std::endl;
    std::cout << " - bitset[d]      : " << bs[Field::d]                      << std::endl;
    std::cout << " - bitset[e]      : " << bs[Field::e]                      << std::endl;
    std::cout << " - bitset[f]      : " << bs[Field::f]                      << std::endl;
    std::cout << " - bitset[g]      : " << bs[Field::g]                      << std::endl;
    std::cout << " - bitset[h]      : " << bs[Field::h]                      << std::endl;

    // Modify bit value
    bf.a         = 1;
    ubf.bits.a   = 1;
    bs[Field::a] = 1;

    // Compare Initial content
    std::cout << "Post-modification field content"                           << std::endl;
    std::cout << " - bitfield[a]    : " << static_cast<unsigned>(bf.a)       << std::endl;
    std::cout << " - bitfield[b]    : " << static_cast<unsigned>(bf.b)       << std::endl;
    std::cout << " - bitfield[c]    : " << static_cast<unsigned>(bf.c)       << std::endl;
    std::cout << " - bitfield[d]    : " << static_cast<unsigned>(bf.d)       << std::endl;
    std::cout << " - bitfield[e]    : " << static_cast<unsigned>(bf.e)       << std::endl;
    std::cout << " - bitfield[f]    : " << static_cast<unsigned>(bf.f)       << std::endl;
    std::cout << " - bitfield[g]    : " << static_cast<unsigned>(bf.g)       << std::endl;
    std::cout << " - bitfield[h]    : " << static_cast<unsigned>(bf.h)       << std::endl;
    std::cout << " - unibitfield[a] : " << static_cast<unsigned>(ubf.bits.a) << std::endl;
    std::cout << " - unibitfield[b] : " << static_cast<unsigned>(ubf.bits.b) << std::endl;
    std::cout << " - unibitfield[c] : " << static_cast<unsigned>(ubf.bits.c) << std::endl;
    std::cout << " - unibitfield[d] : " << static_cast<unsigned>(ubf.bits.d) << std::endl;
    std::cout << " - unibitfield[e] : " << static_cast<unsigned>(ubf.bits.e) << std::endl;
    std::cout << " - unibitfield[f] : " << static_cast<unsigned>(ubf.bits.f) << std::endl;
    std::cout << " - unibitfield[g] : " << static_cast<unsigned>(ubf.bits.g) << std::endl;
    std::cout << " - unibitfield[h] : " << static_cast<unsigned>(ubf.bits.h) << std::endl;
    std::cout << " - bitset[a]      : " << bs[Field::a]                      << std::endl;
    std::cout << " - bitset[b]      : " << bs[Field::b]                      << std::endl;
    std::cout << " - bitset[c]      : " << bs[Field::c]                      << std::endl;
    std::cout << " - bitset[d]      : " << bs[Field::d]                      << std::endl;
    std::cout << " - bitset[e]      : " << bs[Field::e]                      << std::endl;
    std::cout << " - bitset[f]      : " << bs[Field::f]                      << std::endl;
    std::cout << " - bitset[g]      : " << bs[Field::g]                      << std::endl;
    std::cout << " - bitset[h]      : " << bs[Field::h]                      << std::endl;

    return 0;
}
