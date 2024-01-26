#pragma once

#include <image.h>

#include <filesystem>
#include <cmath>
#include <string>
#include <optional>

static std::filesystem::path GetCurrentDir() {
    std::filesystem::path path = __FILE__;
    if (path.has_parent_path()) {
        return path.parent_path();
    } else {
        throw std::runtime_error{"Bad file name"};
    }
}

static const auto kBasePath = GetCurrentDir().string();

inline double PixelDistance(const RGB& lhs, const RGB& rhs) {
    return sqrt(std::pow(lhs.r - rhs.r, 2.0) + std::pow(lhs.g - rhs.g, 2.0) +
                std::pow(lhs.b - rhs.b, 2.0));
}

inline void Compare(const Image& actual, const Image& expected) {
    static const double kEps = 2;
    int matches = 0;

    REQUIRE(actual.Width() == expected.Width());
    REQUIRE(actual.Height() == expected.Height());
    for (int y = 0; y < actual.Height(); ++y) {
        for (int x = 0; x < actual.Width(); ++x) {
            auto actual_data = actual.GetPixel(y, x);
            auto expected_data = expected.GetPixel(y, x);
            auto diff = PixelDistance(actual_data, expected_data);
            matches += diff < kEps;
        }
    }
    double similarity = static_cast<double>(matches) / (actual.Width() * actual.Height());
    REQUIRE(similarity >= 0.99);
}