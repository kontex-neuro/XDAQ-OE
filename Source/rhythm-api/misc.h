#pragma once
#if __cplusplus < 202002L
namespace std
{
namespace numbers
{
constexpr double pi = 3.14159265358979323846;
}
}  // namespace std
#else
#include <numbers>
#endif