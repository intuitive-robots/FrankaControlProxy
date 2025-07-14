#ifndef BYTE_ORDER_HPP
#define BYTE_ORDER_HPP
#include <cstdint>
#include <cstring>

namespace protocol {
//transfer to big-endian
//check if the system is little-endian
constexpr bool is_little_endian() {
#if defined(_WIN32)
    return true;
#elif defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
    return __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__;
#else
    const uint16_t x = 1;
    return *reinterpret_cast<const uint8_t*>(&x) == 1;
#endif
}

#if defined(_MSC_VER)
    #include <intrin.h>
    #define bswap16 _byteswap_ushort
    #define bswap32 _byteswap_ulong
    #define bswap64 _byteswap_uint64
#else
    #define bswap16 __builtin_bswap16
    #define bswap32 __builtin_bswap32
    #define bswap64 __builtin_bswap64
#endif

//uint32 
inline uint32_t to_big_endian_u32(uint32_t val) {
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return bswap32(val);
#else
    return val; 
#endif
}

inline uint32_t from_big_endian_u32(uint32_t val) {
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return bswap32(val);
#else
    return val;
#endif
}

//uint16
inline uint16_t to_big_endian_u16(uint16_t val) {   
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return bswap16(val);
#else
    return val;
#endif
} 

inline uint16_t from_big_endian_u16(uint16_t val) {
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    return bswap16(val);
#else
    return val;
#endif
}

 //double 
inline double to_big_endian_f64(double val) {
    static_assert(sizeof(double) == 8, "Unexpected double size");
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    uint64_t tmp;
    std::memcpy(&tmp, &val, sizeof(tmp));
    tmp = bswap64(tmp);
    std::memcpy(&val, &tmp, sizeof(val));
#endif
    return val;
}

inline double from_big_endian_f64(double val) {
    static_assert(sizeof(double) == 8, "Unexpected double size");
#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    uint64_t tmp;
    std::memcpy(&tmp, &val, sizeof(tmp));
    tmp = bswap64(tmp);
    std::memcpy(&val, &tmp, sizeof(val));
#endif
    return val;
}

} // namespace protocol
#endif // BYTE_ORDER_HPP