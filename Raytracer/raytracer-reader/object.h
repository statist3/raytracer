#pragma once

#include <triangle.h>
#include <material.h>
#include <vector>
#include <sphere.h>

struct Object {
    const Material* material = nullptr;
    Triangle polygon = {};
    std::vector<Vector> normals = {};
    const Vector* GetNormal(size_t index) const {
        const Vector* res = &(normals[index]);
        return res;
    }
};

struct SphereObject {
    SphereObject() {
    }
    SphereObject(const Material* new_material, Sphere new_sphere) {
        sphere = new_sphere;
        material = new_material;
    }
    const Material* material = nullptr;
    Sphere sphere = Sphere({0.0, 0.0, 0.0}, 0.0);
};
