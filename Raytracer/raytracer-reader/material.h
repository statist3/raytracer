#pragma once

#include <vector.h>
#include <string>

struct Material {
    std::string name;
    Vector ambient_color;
    Vector diffuse_color;
    Vector specular_color;
    Vector intensity;
    double specular_exponent = 0.0;
    double refraction_index = 0.0000001;
    std::array<double, 3> albedo = {1.0, 0.0, 0.0};
};
