#pragma once

#include <image.h>
#include <scene.h>
#include <options/camera_options.h>
#include <options/render_options.h>
#include <string>
#include <geometry.h>
#include <iostream>

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

std::array<double, 3> RayIntersectWithSceneKDepth(const Ray& ray, double& max_dist,
                                                  const Scene& scene) {
    std::array<double, 3> pixel;
    double min_dist = 100000000.0;
    for (const auto& object : scene.GetObjects()) {
        auto intersection = GetIntersection(ray, object.polygon);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
        }
    }
    for (const auto& object : scene.GetSphereObjects()) {
        auto intersection = GetIntersection(ray, object.sphere);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
        }
    }
    if ((min_dist > max_dist) &&
        !(min_dist <= 100000000.0 + kEps && min_dist >= 100000000.0 - kEps)) {
        max_dist = min_dist;
    }
    if (min_dist <= 100000000.0 + kEps && min_dist >= 100000000.0 - kEps) {
        pixel[0] = -1.0;
        pixel[1] = -1.0;
        pixel[2] = -1.0;
    } else {
        pixel[0] = min_dist;
        pixel[1] = min_dist;
        pixel[2] = min_dist;
    }
    return pixel;
}
Image KDepthRender(Scene& scene, double& max_dist, const CameraOptions& camera_options) {
    Image image(camera_options.screen_width, camera_options.screen_height);
    std::vector<std::vector<std::array<double, 3>>> rgb(
        camera_options.screen_height,
        std::vector<std::array<double, 3>>(camera_options.screen_width));
    auto move_matrix = LookAt(camera_options);
    double scale = tan(camera_options.fov / 2.0);
    double image_ratio =
        static_cast<double>(camera_options.screen_width) / camera_options.screen_height;
    for (int height_index = 0; height_index < camera_options.screen_height; ++height_index) {
        for (int width_index = 0; width_index < camera_options.screen_width; ++width_index) {
            double x_coordinate =
                (2 * (width_index + 0.5) / (static_cast<double>(camera_options.screen_width)) - 1) *
                image_ratio * scale;
            double y_coordinate = (1 - 2 * (height_index + 0.5) /
                                           (static_cast<double>(camera_options.screen_height))) *
                                  scale;
            auto world_ray = GenerateRay(camera_options, move_matrix, x_coordinate, y_coordinate);
            rgb[height_index][width_index] =
                RayIntersectWithSceneKDepth(world_ray, max_dist, scene);
        }
    }

    for (int height_index = 0; height_index < camera_options.screen_height; ++height_index) {
        for (int width_index = 0; width_index < camera_options.screen_width; ++width_index) {
            auto pixel = image.GetPixel(height_index, width_index);
            if (rgb[height_index][width_index][0] >= 0) {
                pixel.r = static_cast<int>(255 * rgb[height_index][width_index][0] / max_dist);
                pixel.g = static_cast<int>(255 * rgb[height_index][width_index][1] / max_dist);
                pixel.b = static_cast<int>(255 * rgb[height_index][width_index][2] / max_dist);
                image.SetPixel(pixel, height_index, width_index);
            } else {
                pixel.r = 255;
                pixel.g = 255;
                pixel.b = 255;
                image.SetPixel(pixel, height_index, width_index);
            }
        }
    }
    return image;
}

std::array<double, 3> RayIntersectWithSceneKNormal(const Ray& ray, const Scene& scene) {
    std::array<double, 3> pixel;
    double min_dist = 100000000.0;
    Object near_object;
    Vector near_normal;
    for (const auto& object : scene.GetObjects()) {
        auto intersection = GetIntersection(ray, object.polygon);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
            if (object.normals[0][0] != 0 && object.normals[0][1] != 0 &&
                object.normals[0][2] != 0) {
                auto baricentric =
                    GetBarycentricCoords(object.polygon, intersection.value().GetPosition());
                near_normal = baricentric[0] * object.normals[0] +
                              baricentric[1] * object.normals[1] +
                              baricentric[2] * object.normals[2];
            } else {
                near_normal = intersection.value().GetNormal();
            }
        }
    }
    for (const auto& object : scene.GetSphereObjects()) {
        auto intersection = GetIntersection(ray, object.sphere);
        if (!intersection.has_value()) {
            continue;
        }
        if (intersection.value().GetDistance() < min_dist) {
            min_dist = intersection.value().GetDistance();
            near_normal = intersection.value().GetNormal();
        }
    }

    if (min_dist <= 100000000.0 + kEps && min_dist >= 100000000.0 - kEps) {
        pixel[0] = 0.0;
        pixel[1] = 0.0;
        pixel[2] = 0.0;
    } else {
        pixel[0] = 0.5 * near_normal[0] + 0.5;
        pixel[1] = 0.5 * near_normal[1] + 0.5;
        pixel[2] = 0.5 * near_normal[2] + 0.5;
    }
    return pixel;
}

Image KNormalRender(Scene& scene, const CameraOptions& camera_options) {
    Image image(camera_options.screen_width, camera_options.screen_height);

    auto move_matrix = LookAt(camera_options);
    double scale = tan(camera_options.fov / 2.0);
    double image_ratio =
        static_cast<double>(camera_options.screen_width) / camera_options.screen_height;
    for (int height_index = 0; height_index < camera_options.screen_height; ++height_index) {
        for (int width_index = 0; width_index < camera_options.screen_width; ++width_index) {
            double x_coordinate =
                (2 * (width_index + 0.5) / (static_cast<double>(camera_options.screen_width)) - 1) *
                image_ratio * scale;
            double y_coordinate = (1 - 2 * (height_index + 0.5) /
                                           (static_cast<double>(camera_options.screen_height))) *
                                  scale;
            auto world_ray = GenerateRay(camera_options, move_matrix, x_coordinate, y_coordinate);
            auto normals = RayIntersectWithSceneKNormal(world_ray, scene);
            RGB pixel;
            pixel.r = static_cast<int>(255 * normals[0]);
            pixel.g = static_cast<int>(255 * normals[1]);
            pixel.b = static_cast<int>(255 * normals[2]);
            image.SetPixel(pixel, height_index, width_index);
        }
    }
    return image;
}

Image Render(const std::string& filename, const CameraOptions& camera_options,
             const RenderOptions& render_options) {
    double max_dist = -1.0;
    Scene scene = ReadScene(filename);
    if (render_options.mode == RenderMode::kDepth) {
        return KDepthRender(scene, max_dist, camera_options);
    } else {
        return KNormalRender(scene, camera_options);
    }
}