#pragma once

#include <vector.h>

class Intersection {
public:
    Intersection(Vector pos, Vector norm, double dist) : position_(pos), distance_(dist) {
        norm.Normalize();
        normal_ = norm;
    }

    const Vector& GetPosition() const {
        return position_;
    }
    const Vector& GetNormal() const {
        return normal_;
    }
    double GetDistance() const {
        return distance_;
    }

private:
    Vector position_;
    Vector normal_;
    double distance_;
};
