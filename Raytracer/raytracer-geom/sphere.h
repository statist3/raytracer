#pragma once

#include "vector.h"
#include "ray.h"
#include "intersection.h"
#include <optional>

class Sphere {
public:
    // Конструктор: инициализирует сферу с заданным центром и радиусом
    Sphere(Vector center, double radius) : center_(center), radius_(radius) {}

    // Возвращает центр сферы
    const Vector& GetCenter() const {
        return center_;
    }

    // Возвращает радиус сферы
    double GetRadius() const {
        return radius_;
    }

private:
    Vector center_; // Центр сферы
    double radius_; // Радиус сферы
};
