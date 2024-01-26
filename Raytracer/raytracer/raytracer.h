#pragma once

#include <image.h>
#include <scene.h>
#include <options/camera_options.h>
#include <options/render_options.h>
#include <string>
#include <geometry.h>
#include <iostream>

//  const double kEps = 1e-6;

std::vector<std::vector<double>> MoveMatrix(Vector& camera_look_from, Vector& forward,
                                            Vector& right, Vector& up) {
    std::vector<std::vector<double>> move_matrix(4, std::vector<double>(4));
    move_matrix[0][0] = right[0];
    move_matrix[0][1] = right[1];
    move_matrix[0][2] = right[2];
    move_matrix[0][3] = 0.0;
    move_matrix[1][0] = up[0];
    move_matrix[1][1] = up[1];
    move_matrix[1][2] = up[2];
    move_matrix[1][3] = 0.0;
    move_matrix[2][0] = forward[0];
    move_matrix[2][1] = forward[1];
    move_matrix[2][2] = forward[2];
    move_matrix[2][3] = 0.0;
    move_matrix[3][0] = camera_look_from[0];
    move_matrix[3][1] = camera_look_from[1];
    move_matrix[3][2] = camera_look_from[2];
    move_matrix[3][3] = 1.0;

    return move_matrix;
}

Ray GenerateRay(const CameraOptions& camera_options,
                const std::vector<std::vector<double>>& move_matrix, double x_coordinate,
                double y_coordinate) {

    Vector camera_look_from(camera_options.look_from);
    //  Vector camera_look_to(camera_options.look_to);
    Vector vector({x_coordinate, y_coordinate, -1});
    vector.Normalize();
    Vector world_vector;
    world_vector[0] = move_matrix[0][0] * vector[0] + move_matrix[1][0] * vector[1] +
                      move_matrix[2][0] * vector[2] + move_matrix[3][0] * 1;
    world_vector[1] = move_matrix[0][1] * vector[0] + move_matrix[1][1] * vector[1] +
                      move_matrix[2][1] * vector[2] + move_matrix[3][1] * 1;
    world_vector[2] = move_matrix[0][2] * vector[0] + move_matrix[1][2] * vector[1] +
                      move_matrix[2][2] * vector[2] + move_matrix[3][2];
    world_vector = (world_vector - camera_look_from);
    world_vector.Normalize();
    Ray world_ray(camera_look_from, world_vector);
    return world_ray;
}
std::vector<std::vector<double>> LookAt(const CameraOptions& camera_options) {
    Vector camera_look_from(camera_options.look_from);
    Vector camera_look_to(camera_options.look_to);
    Vector forward = camera_look_from - camera_look_to;
    forward.Normalize();
    Vector right = CrossProduct(Vector({0.0, 1.0, 0.0}), forward);
    Vector up;
    if ((std::fabs(right[0]) < kEps) && (std::fabs(right[1]) < kEps) &&
        (std::fabs(right[2]) < kEps)) {
        right = Vector({1.0, 0.0, 0.0});
    } else {
        right.Normalize();
    }
    up = CrossProduct(forward, right);
    auto move_matrix = MoveMatrix(camera_look_from, forward, right, up);
    return move_matrix;
}

bool LightIsSeen(const Light& light, const Intersection& intersection, const Scene& scene) {
    Vector direction = light.position - intersection.GetPosition();
    direction.Normalize();
    Vector eps{kEps, kEps, kEps};
    Ray ray(intersection.GetPosition() + kEps * intersection.GetNormal(), direction);  // ??
    for (const auto& object : scene.GetObjects()) {
        auto intersection_light = GetIntersection(ray, object.polygon);
        if (intersection_light.has_value()) {
            if (intersection_light.value().GetDistance() <
                Length(light.position - intersection.GetPosition())) {
                return false;
            }
        }
    }

    for (const auto& object : scene.GetSphereObjects()) {
        auto intersection_light = GetIntersection(ray, object.sphere);
        if (intersection_light.has_value()) {
            if (intersection_light.value().GetDistance() <
                Length(light.position - intersection.GetPosition())) {
                return false;
            }
        }
    }

    return true;
}

std::optional<SphereObject> FindNearestSphere(const Ray& ray, const Scene& scene) {
    double min_dist = 1000000.0;
    bool was_intersection = false;

    SphereObject nearest_sphere;
    for (const auto& object : scene.GetSphereObjects()) {
        auto intersection = GetIntersection(ray, object.sphere);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
            nearest_sphere = object;
            was_intersection = true;
        }
    }
    if (!was_intersection) {
        return {};
    } else {
        return nearest_sphere;
    }
}

std::optional<Object> FindNearestObject(const Ray& ray, const Scene& scene) {
    double min_dist = 1000000.0;
    bool was_intersection = false;

    Object nearest_object;
    for (const auto& object : scene.GetObjects()) {
        auto intersection = GetIntersection(ray, object.polygon);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
            nearest_object = object;
            was_intersection = true;
        }
    }
    if (!was_intersection) {
        return {};
    } else {
        return nearest_object;
    }
}

Vector CountIllumination(const Ray& ray, const Intersection& intersection, Object object,
                         bool intersection_with_sphere, const Material material, const Scene& scene,
                         bool in_sphere, int depth, int recursive_depth) {

    if (recursive_depth > depth) {
        return {0.0, 0.0, 0.0};
    }
    Vector illumination_main;
    Vector illumination_refract;
    Vector illumination_reflect;
    for (const auto& light : scene.GetLights()) {
        if (LightIsSeen(light, intersection, scene)) {
            Vector normal_in_intersection = intersection.GetNormal();
            if (object.normals.size() == 3 && (object.normals[0][0] != 0.0) &&
                (object.normals[0][1] != 0.0) && (object.normals[0][2] != 0.0)) {
                auto coord = GetBarycentricCoords(object.polygon, intersection.GetPosition());
                normal_in_intersection = coord[0] * object.normals[0] +
                                         coord[1] * object.normals[1] +
                                         coord[2] * object.normals[2];
            }
            normal_in_intersection.Normalize();
            Vector vl = light.position - intersection.GetPosition();
            vl.Normalize();
            double scalar_d = std::max(0.0, DotProduct(normal_in_intersection, vl));
            Vector diffuse = scalar_d * (material.diffuse_color * light.intensity);
            Vector specular;
            Vector ve, vr;
            vr = 2 * (DotProduct(normal_in_intersection, vl)) * normal_in_intersection - vl;
            ve = -1 * ray.GetDirection();
            double tmp = std::max(0.0, DotProduct(ve, vr));
            double scalar_ls = std::pow(tmp, material.specular_exponent);
            specular = scalar_ls * (material.specular_color * light.intensity);
            illumination_main = illumination_main + (material.albedo[0] * diffuse) +
                                (material.albedo[0] * specular);
        }
    }
    Object tmp_object;
    if (!in_sphere) {  // если я вне сферы, то считаю отраженный.
        Vector reflect = Reflect(ray.GetDirection(), intersection.GetNormal());
        Ray ray_reflect(intersection.GetPosition() + kEps * intersection.GetNormal(),
                        reflect);  // надо найти где пересекает
        auto sphere_intersection = FindNearestSphere(ray_reflect, scene);  // нашел
        auto object_intersection = FindNearestObject(ray_reflect, scene);
        if ((sphere_intersection.has_value()) && (object_intersection.has_value())) {
            double sphere_dist =
                GetIntersection(ray_reflect, sphere_intersection.value().sphere)->GetDistance();
            double object_dist =
                GetIntersection(ray_reflect, object_intersection.value().polygon)->GetDistance();
            if (sphere_dist < object_dist) {
                illumination_reflect = CountIllumination(
                    ray_reflect,
                    GetIntersection(ray_reflect, sphere_intersection.value().sphere).value(),
                    tmp_object, true, *(sphere_intersection->material), scene, false, depth,
                    recursive_depth + 1);
            } else {
                illumination_reflect = CountIllumination(
                    ray_reflect,
                    GetIntersection(ray_reflect, object_intersection.value().polygon).value(),
                    object_intersection.value(), false, *(object_intersection->material), scene,
                    false, depth, recursive_depth + 1);
            }
        } else if (sphere_intersection.has_value()) {
            illumination_reflect = CountIllumination(
                ray_reflect,
                GetIntersection(ray_reflect, sphere_intersection.value().sphere).value(),
                tmp_object, true, *(sphere_intersection->material), scene, false, depth,
                recursive_depth + 1);
        } else if (object_intersection.has_value()) {
            illumination_reflect = CountIllumination(
                ray_reflect,
                GetIntersection(ray_reflect, object_intersection.value().polygon).value(),
                object_intersection.value(), false, *(object_intersection->material), scene, false,
                depth, recursive_depth + 1);
        }
    }

    if (intersection_with_sphere &&
        (!in_sphere)) {  // луч попал в сферу, преломленный должен пойти в сферу
        auto refract =
            Refract(ray.GetDirection(), intersection.GetNormal(), 1.0 / material.refraction_index);
        if (refract.has_value()) {
            // надо найти ближайший луч и запуститься рекурсивно
            refract.value().Normalize();
            Ray ray_refract(intersection.GetPosition() - kEps * intersection.GetNormal(),
                            refract.value());
            auto sphere_intersection = FindNearestSphere(ray_refract, scene);  // нашел
            if (sphere_intersection.has_value()) {
                Material material_new = *sphere_intersection->material;
                material_new.albedo[2] = 1;
                illumination_refract = CountIllumination(
                    ray_refract,
                    GetIntersection(ray_refract, sphere_intersection.value().sphere).value(),
                    tmp_object, true, material_new, scene, true, depth, recursive_depth + 1);
            }
        }
    } else if (intersection_with_sphere &&
               in_sphere) {  // луч был изнутри сферы, преломленный должен пойти из сферы
        auto refract =
            Refract(ray.GetDirection(), intersection.GetNormal(), material.refraction_index);
        if (refract.has_value()) {
            // надо найти ближайший луч и запуститься рекурсивно
            refract.value().Normalize();
            Ray ray_refract(intersection.GetPosition() - kEps * intersection.GetNormal(),
                            refract.value());
            auto sphere_intersection = FindNearestSphere(ray_refract, scene);  // нашел
            auto object_intersection = FindNearestObject(ray_refract, scene);
            if ((sphere_intersection.has_value()) && (object_intersection.has_value())) {
                double sphere_dist =
                    GetIntersection(ray_refract, sphere_intersection.value().sphere)->GetDistance();
                double object_dist =
                    GetIntersection(ray_refract, object_intersection.value().polygon)
                        ->GetDistance();
                if (sphere_dist < object_dist) {
                    illumination_refract = CountIllumination(
                        ray_refract,
                        GetIntersection(ray_refract, sphere_intersection.value().sphere).value(),
                        tmp_object, true, *(sphere_intersection->material), scene, false, depth,
                        recursive_depth + 1);
                } else {
                    illumination_refract = CountIllumination(
                        ray_refract,
                        GetIntersection(ray_refract, object_intersection.value().polygon).value(),
                        object_intersection.value(), false, *(object_intersection->material), scene,
                        false, depth, recursive_depth + 1);
                }
            } else if (sphere_intersection.has_value()) {
                illumination_refract = CountIllumination(
                    ray_refract,
                    GetIntersection(ray_refract, sphere_intersection.value().sphere).value(),
                    tmp_object, true, *(sphere_intersection->material), scene, false, depth,
                    recursive_depth + 1);
            } else if (object_intersection.has_value()) {
                illumination_refract = CountIllumination(
                    ray_refract,
                    GetIntersection(ray_refract, object_intersection.value().polygon).value(),
                    object_intersection.value(), false, *(object_intersection->material), scene,
                    false, depth, recursive_depth + 1);
            }
        }
    } else {
        auto refract =
            Refract(ray.GetDirection(), intersection.GetNormal(), 1.0 / material.refraction_index);
        if (refract.has_value()) {
            // надо найти ближайший луч и запуститься рекурсивно
            refract.value().Normalize();
            Ray ray_refract(intersection.GetPosition() - kEps * intersection.GetNormal(),
                            refract.value());
            auto sphere_intersection = FindNearestSphere(ray_refract, scene);  // нашел
            auto object_intersection = FindNearestObject(ray_refract, scene);
            if ((sphere_intersection.has_value()) && (object_intersection.has_value())) {
                double sphere_dist =
                    GetIntersection(ray_refract, sphere_intersection.value().sphere)->GetDistance();
                double object_dist =
                    GetIntersection(ray_refract, object_intersection.value().polygon)
                        ->GetDistance();
                if (sphere_dist < object_dist) {
                    illumination_refract = CountIllumination(
                        ray_refract,
                        GetIntersection(ray_refract, sphere_intersection.value().sphere).value(),
                        tmp_object, true, *(sphere_intersection->material), scene, false, depth,
                        recursive_depth + 1);
                } else {
                    illumination_refract = CountIllumination(
                        ray_refract,
                        GetIntersection(ray_refract, object_intersection.value().polygon).value(),
                        object_intersection.value(), false, *(object_intersection->material), scene,
                        false, depth, recursive_depth + 1);
                }
            } else if (sphere_intersection.has_value()) {
                illumination_refract = CountIllumination(
                    ray_refract,
                    GetIntersection(ray_refract, sphere_intersection.value().sphere).value(),
                    tmp_object, true, *(sphere_intersection->material), scene, false, depth,
                    recursive_depth + 1);
            } else if (object_intersection.has_value()) {
                illumination_refract = CountIllumination(
                    ray_refract,
                    GetIntersection(ray_refract, object_intersection.value().polygon).value(),
                    object_intersection.value(), false, *(object_intersection->material), scene,
                    false, depth, recursive_depth + 1);
            }
        }
    }
    return illumination_main + material.intensity + material.ambient_color +
           material.albedo[1] * illumination_reflect + material.albedo[2] * illumination_refract;
}

Vector KFullIntersection(const Ray& ray, const Scene& scene, int depth) {
    double min_dist = 1000000.0;
    Object nearest_object;
    bool was_intersection = false;
    for (const auto& object : scene.GetObjects()) {
        auto intersection = GetIntersection(ray, object.polygon);
        if (intersection.has_value()) {
            if (intersection.value().GetDistance() < min_dist) {
                min_dist = intersection.value().GetDistance();
                nearest_object = object;
                was_intersection = true;
            }
        }
    }
    SphereObject nearest_sphere;
    for (const auto& object : scene.GetSphereObjects()) {
        auto intersection = GetIntersection(ray, object.sphere);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
            nearest_sphere = object;
            was_intersection = true;
        }
    }
    if (!was_intersection) {
        return {0.0, 0.0, 0.0};
    }
    Vector illumination;
    if (nearest_sphere.sphere.GetRadius() > kEps) {
        illumination = CountIllumination(ray, GetIntersection(ray, nearest_sphere.sphere).value(),
                                         nearest_object, true, *(nearest_sphere.material), scene,
                                         false, depth, 0);
    } else {
        illumination = CountIllumination(ray, GetIntersection(ray, nearest_object.polygon).value(),
                                         nearest_object, false, *(nearest_object.material), scene,
                                         false, depth, 0);
    }
    return illumination;
}

Image KFullRender(Scene& scene, const CameraOptions& camera_options, int depth) {
    Image image(camera_options.screen_width, camera_options.screen_height);
    std::vector<std::vector<Vector>> rgb(camera_options.screen_height,
                                         std::vector<Vector>(camera_options.screen_width));
    auto move_matrix = LookAt(camera_options);
    double scale = tan(camera_options.fov / 2.0);
    double image_ratio =
        static_cast<double>(camera_options.screen_width) / camera_options.screen_height;
    double max = 0.0;
    for (int height_index = 0; height_index < camera_options.screen_height; ++height_index) {
        for (int width_index = 0; width_index < camera_options.screen_width; ++width_index) {
            double x_coordinate =
                (2 * (width_index + 0.5) / (static_cast<double>(camera_options.screen_width)) - 1) *
                image_ratio * scale;
            double y_coordinate = (1 - 2 * (height_index + 0.5) /
                                           (static_cast<double>(camera_options.screen_height))) *
                                  scale;
            auto world_ray = GenerateRay(camera_options, move_matrix, x_coordinate, y_coordinate);
            rgb[height_index][width_index] = KFullIntersection(world_ray, scene, depth);
            if (rgb[height_index][width_index][0] > max) {
                max = rgb[height_index][width_index][0];
            }
            if (rgb[height_index][width_index][1] > max) {
                max = rgb[height_index][width_index][1];
            }
            if (rgb[height_index][width_index][2] > max) {
                max = rgb[height_index][width_index][2];
            }
        }
    }
    for (int height_index = 0; height_index < camera_options.screen_height; ++height_index) {
        for (int width_index = 0; width_index < camera_options.screen_width; ++width_index) {
            auto rgb_vector = rgb[height_index][width_index];

            if (max > 0.0) {
                rgb_vector[0] =
                    rgb_vector[0] * (1.0 + rgb_vector[0] / (max * max)) / (1.0 + rgb_vector[0]);
                rgb_vector[1] =
                    rgb_vector[1] * (1.0 + rgb_vector[1] / (max * max)) / (1.0 + rgb_vector[1]);
                rgb_vector[2] =
                    rgb_vector[2] * (1.0 + rgb_vector[2] / (max * max)) / (1.0 + rgb_vector[2]);
            }

            rgb_vector[0] = std::pow(rgb_vector[0], 1.0 / 2.2);
            rgb_vector[1] = std::pow(rgb_vector[1], 1.0 / 2.2);
            rgb_vector[2] = std::pow(rgb_vector[2], 1.0 / 2.2);

            RGB pixel;
            pixel.r = static_cast<int>(255.0 * rgb_vector[0]);
            pixel.g = static_cast<int>(255.0 * rgb_vector[1]);
            pixel.b = static_cast<int>(255.0 * rgb_vector[2]);
            image.SetPixel(pixel, height_index, width_index);
        }
    }

    return image;
}

Image Render(const std::string& filename, const CameraOptions& camera_options,
             const RenderOptions& render_options) {
    Scene scene = ReadScene(filename);
    return KFullRender(scene, camera_options, render_options.depth);
}

// -0.539752 0.233919 0.0585771
