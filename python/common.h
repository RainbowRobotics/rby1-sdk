#pragma once

#include <chrono>

constexpr unsigned int kDoublePrecision = 3;

std::chrono::system_clock::time_point timespec_to_time_point(const timespec& ts);

std::chrono::nanoseconds timespec_to_nanoseconds(const timespec& ts);