#pragma once

#include <vector.h>

class Triangle {
public:
    Triangle(std::initializer_list<Vector> list) {
        int index = 0;
        for (auto it = list.begin(); it != list.end(); ++it) {
            vertices_[index][0] = (*it)[0];
            vertices_[index][1] = (*it)[1];
            vertices_[index][2] = (*it)[2];
            ++index;
        }
    }
    double Area() const {
        auto vec = CrossProduct(GetVertex(0) - GetVertex(2), GetVertex(1) - GetVertex(2));
        return Length(vec) / 2.0;
    }

    const Vector& GetVertex(size_t ind) const {
        return vertices_[ind];
    }

    const Vector& operator[](size_t index) const {
        return vertices_[index];
    }

private:
    std::array<Vector, 3> vertices_;
};
