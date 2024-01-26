#pragma once

#include <array>
#include <cmath>
#include <initializer_list>
#include <algorithm>

class Vector {
public:
    Vector() {};
    Vector(std::initializer_list<double> list) {
        int index = 0;
        for (auto it = list.begin(); it != list.end(); ++it) {
            data_[index] = *it;
            ++index;
        }
    }
    Vector(std::array<double, 3> data) : data_(data) {}

    double& operator[](size_t ind) {
        return data_[ind];
    }
    double operator[](size_t ind) const {
        return data_[ind];
    }

    Vector Normalize() const {
        double len = Length(*this);
        if (len == 0) return *this; // Предотвращаем деление на ноль
        return Vector({data_[0] / len, data_[1] / len, data_[2] / len});
    }

    Vector operator-() const {
        return {-data_[0], -data_[1], -data_[2]};
    }

private:
    std::array<double, 3> data_ = {0.0, 0.0, 0.0};
};

// Объявление функции Length
inline double Length(const Vector& vec) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

inline double DotProduct(const Vector& lhs, const Vector& rhs) {
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

inline Vector CrossProduct(const Vector& a, const Vector& b) {
    return Vector{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

inline Vector operator+(const Vector& a, const Vector& b) {
    return Vector{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

inline Vector operator-(const Vector& a, const Vector& b) {
    return Vector{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline Vector operator*(const double& scalar, const Vector& a) {
    return Vector{a[0] * scalar, a[1] * scalar, a[2] * scalar};
}

inline double Angle(const Vector& lhs, const Vector& rhs) {
    return DotProduct(lhs, rhs) / (Length(lhs) * Length(rhs));
}
