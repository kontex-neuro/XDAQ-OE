#pragma once
#include <zpp_bits.h>

#include <concepts>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <vector>

namespace xdaq
{
namespace protocol
{
constexpr std::size_t header_length = 4;

using namespace zpp::bits::literals;

template <zpp::bits::string_literal cmd>
struct cmd_id {
    constexpr static auto name = cmd;
    constexpr static auto id = zpp::bits::id_v<zpp::bits::sha256<cmd>(), header_length>;
};

using id_t = std::remove_const_t<decltype(cmd_id<"">::id)>;


struct set_ttl_out : cmd_id<"set_ttl_out"> {
    struct Request {
        std::uint32_t ttl;
    };
    struct Response {
    };
};

struct is_stream_enabled : cmd_id<"is_stream_enabled"> {
    struct Request {
        std::uint32_t stream;
    };
    struct Response {
        bool enabled;
    };
};

struct start_run : cmd_id<"start_run"> {
    struct Request {
    };
    struct Response {
    };
};

struct stop_run : cmd_id<"stop_run"> {
    struct Request {
    };
    struct Response {
    };
};

struct is_running : cmd_id<"is_running"> {
    struct Request {
    };
    struct Response {
        bool running;
    };
};

struct set_stream_enabled : cmd_id<"set_stream_enabled"> {
    struct Request {
        std::uint32_t stream;
        bool enabled;
    };
    struct Response {
    };
};

struct scan_chips : cmd_id<"scan_chips"> {
    struct Request {
    };
    struct Response {
        std::vector<std::tuple<std::uint8_t, std::uint8_t>> chips;
    };
};

// Concept checking for valid protocol
template <typename T>
concept Protocol = requires(T t) {
    T::name;
    T::id;
    typename T::Request;
    typename T::Response;
};


template <typename T>
    requires std::is_trivially_copyable_v<T>
constexpr auto get_serialized_len()
{
    std::array<std::byte, 0x1000> data;
    zpp::bits::out out{data};
    out(T{}).or_throw();
    return out.position();
}

template <typename T>
concept FixSizedMessage = requires { get_serialized_len<T>(); };

template <typename T>
concept VariableSizedMessage = !FixSizedMessage<T>;

}  // namespace protocol
}  // namespace xdaq