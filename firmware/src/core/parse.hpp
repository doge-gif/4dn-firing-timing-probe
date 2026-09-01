#pragma once
#include <cstdint>
#include <string_view>

namespace core {

bool parse_u32(std::string_view s, std::uint32_t& out);
bool parse_f32(std::string_view s, float& out);

} // namespace core
