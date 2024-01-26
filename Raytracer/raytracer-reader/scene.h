#pragma once

#include <material.h>
#include <vector.h>
#include <object.h>
#include <light.h>

#include <vector>
#include <map>
#include <string>
#include <fstream>

inline std::vector<double> ReadNumbers(const std::string& str) {
    std::vector<double> result;
    size_t index = 0;
    while (index < str.size()) {
        if ((str[index] == '-') || (isdigit(str[index]))) {
            std::string number;
            number += str[index];
            ++index;
            while ((index < str.size()) && (!isspace(str[index])) && (str[index] != '/')) {
                number += str[index];
                ++index;
            }
            result.push_back(std::stod(number));
        } else {
            ++index;
        }
    }
    return result;
}

inline std::array<double, 3> ReadThreeNumbers(const std::string& str) {
    auto res = ReadNumbers(str);
    std::array<double, 3> arr;
    arr[0] = res[0];
    arr[1] = res[1];
    arr[2] = res[2];
    return arr;
}

inline std::string ReadMaterialField(const std::string& str) {
    size_t index = 0;
    while (!isalpha(str[index])) {
        ++index;
    }
    std::string field;
    field += str[index];
    while ((index < str.size()) && (str[index + 1] != ' ')) {
        ++index;
        field += str[index];
    }
    return field;
}

inline char FirstChar(const std::string& str) {
    size_t index = 0;
    while (isspace(str[index])) {
        ++index;
    }
    return str[index];
}

class Scene {
public:
    Scene(){};
    const std::vector<Object>& GetObjects() const {
        return objects_;
    }
    void AddObject(const Object& object) {
        objects_.push_back(object);
    }
    const std::vector<SphereObject>& GetSphereObjects() const {
        return sphere_objects_;
    }
    void AddSphereObjects(const SphereObject& sphere) {
        sphere_objects_.push_back(sphere);
    }
    const std::vector<Light>& GetLights() const {
        return lights_;
    }
    void AddLight(const Light& light) {
        lights_.push_back(light);
    }
    const std::map<std::string, Material>& GetMaterials() const {
        return materials_;
    }
    void AddMaterials(const std::map<std::string, Material>& new_materials) {
        materials_ = new_materials;
    }
    const Material* GetMaterialPointer(const std::string& name) {
        return &materials_[name];
    }

private:
    std::vector<Object> objects_;
    std::vector<SphereObject> sphere_objects_;
    std::vector<Light> lights_;
    std::map<std::string, Material> materials_;
};

inline Sphere CreateSphere(std::vector<double> coordinates) {
    double radius = coordinates[3];
    Vector vector;
    vector[0] = coordinates[0];
    vector[1] = coordinates[1];
    vector[2] = coordinates[2];
    Sphere sphere(vector, radius);
    return sphere;
}
inline Light CreateLight(std::vector<double> coordinates) {
    Vector vector;
    vector[0] = coordinates[0];
    vector[1] = coordinates[1];
    vector[2] = coordinates[2];
    Vector rgb;
    rgb[0] = coordinates[3];
    rgb[1] = coordinates[4];
    rgb[2] = coordinates[5];
    Light light(vector, rgb);
    return light;
}

inline std::map<std::string, Material> ReadMaterials(std::string_view filename) {
    std::map<std::string, Material> result;
    std::fstream fin(static_cast<std::string>(filename));
    std::string file_line;
    std::string curr_name;
    while (std::getline(fin, file_line)) {
        if ((file_line.empty()) || (file_line[0] == '#') || (FirstChar(file_line) == '#')) {
            continue;
        }
        std::string str = ReadMaterialField(file_line);
        std::string mtl;

        if (str == "newmtl") {
            std::string object_name;
            size_t index = 7;
            while ((index < file_line.size()) && (file_line[index] != ' ')) {
                object_name += file_line[index];
                ++index;
            }
            curr_name = object_name;
            result[curr_name].name = curr_name;
        }
        if ((!curr_name.empty()) && (str != "newmtl")) {
            if ((str[0] == 'K') && (str[1] == 'a')) {
                result[curr_name].ambient_color = ReadThreeNumbers(file_line);
            }
            if ((str[0] == 'K') && (str[1] == 'd')) {
                result[curr_name].diffuse_color = ReadThreeNumbers(file_line);
            }
            if ((str[0] == 'K') && (str[1] == 's')) {
                result[curr_name].specular_color = ReadThreeNumbers(file_line);
            }
            if ((str[0] == 'K') && (str[1] == 'e')) {
                result[curr_name].intensity = ReadThreeNumbers(file_line);
            }
            if ((str[0] == 'N') && (str[1] == 's')) {
                result[curr_name].specular_exponent = ReadNumbers(file_line)[0];
            }
            if ((str[0] == 'N') && (str[1] == 'i')) {
                result[curr_name].refraction_index = ReadNumbers(file_line)[0];
            }
            if ((str[0] == 'a') && (str[1] == 'l')) {
                result[curr_name].albedo = ReadThreeNumbers(file_line);
            }
        }
    }
    return result;
}
inline std::string FileName(const std::string& filename) {
    auto index = filename.end();
    for (; index != filename.begin(); --index) {
        if (*index == '/') {
            break;
        }
    }
    std::string result;
    for (auto ind = filename.begin(); ind != index; ++ind) {
        result += *ind;
    }
    return result;
}

inline std::string ParsName(std::string& str) {
    size_t index = 7;
    std::string result;
    for (; index < str.size(); ++index) {
        result += str[index];
    }
    return result;
}

std::vector<std::pair<int, int>> ParsF(const std::string& file_line) {
    std::vector<std::pair<int, int>> result;
    size_t index = 1;
    while (!isspace(file_line[index])) {
        ++index;
    }
    while ((index < file_line.size())) {
        std::string vertex_and_normal;
        while (std::isspace(file_line[index])) {
            ++index;
        }

        while ((index < file_line.size()) && (!isspace(file_line[index]))) {
            vertex_and_normal += file_line[index];
            ++index;
        }
        std::string vertex_index;
        std::string normal_index;
        size_t ind = 0;
        while ((ind < vertex_and_normal.size()) && (vertex_and_normal[ind] != '/')) {
            if (isspace(vertex_and_normal[ind])) {
                ++ind;
            }
            vertex_index += vertex_and_normal[ind];
            ++ind;
        }
        ++ind;
        while ((ind < vertex_and_normal.size()) && (vertex_and_normal[ind] != '/')) {
            ++ind;
        }
        ++ind;
        while (ind < vertex_and_normal.size()) {
            normal_index += vertex_and_normal[ind];
            ++ind;
        }

        if ((normal_index.empty()) && (!vertex_index.empty())) {
            result.push_back({std::stoi(vertex_index), 0});
        } else if (!vertex_index.empty()) {
            result.push_back({std::stoi(vertex_index), std::stoi(normal_index)});
        }
        while ((index < file_line.size()) && (!isspace(file_line[index]))) {
            ++index;
        }
    }
    return result;
}

void AddObject(std::vector<Object>& new_objects, const std::string& file_line,
               std::vector<Vector>& vertexes, std::vector<Vector>& normals, Scene& scene,
               std::string name) {
    auto vertex_and_normals = ParsF(file_line);
    Vector main_vertex, main_normal;
    main_vertex = (vertex_and_normals[0].first > 0)
                      ? vertexes[vertex_and_normals[0].first - 1]
                      : vertexes[vertexes.size() + vertex_and_normals[0].first];
    if (vertex_and_normals[0].second != 0) {
        main_normal = (vertex_and_normals[0].second > 0)
                          ? normals[vertex_and_normals[0].second - 1]
                          : normals[normals.size() + vertex_and_normals[0].second];
    }
    size_t index = 1;
    while (index + 1 < vertex_and_normals.size()) {
        Vector normal_first, normal_second;
        Vector vertex_first = (vertex_and_normals[index].first > 0)
                                  ? vertexes[vertex_and_normals[index].first - 1]
                                  : vertexes[vertexes.size() + vertex_and_normals[index].first];
        if (vertex_and_normals[index].second != 0) {
            normal_first = (vertex_and_normals[index].second > 0)
                               ? normals[vertex_and_normals[index].second - 1]
                               : normals[normals.size() + vertex_and_normals[index].second];
        }
        Vector vertex_second =
            (vertex_and_normals[index + 1].first > 0)
                ? vertexes[vertex_and_normals[index + 1].first - 1]
                : vertexes[vertexes.size() + vertex_and_normals[index + 1].first];
        if (vertex_and_normals[index + 1].second != 0) {
            normal_second = (vertex_and_normals[index + 1].second > 0)
                                ? normals[vertex_and_normals[index + 1].second - 1]
                                : normals[normals.size() + vertex_and_normals[index + 1].second];
        }
        Object object;
        object.material = scene.GetMaterialPointer(name);
        Triangle triangle({main_vertex, vertex_first, vertex_second});
        object.polygon = triangle;
        object.normals = {main_normal, normal_first, normal_second};
        new_objects.push_back(object);
        index += 1;
    }
}
inline Scene ReadScene(const std::string& filename) {
    std::string path_name = FileName(filename);
    std::vector<Vector> vertexes;
    std::vector<Vector> normals;
    std::ifstream fin(static_cast<std::string>((filename)));
    std::string file_line;
    std::string curr_name;
    Scene scene;
    while (std::getline(fin, file_line)) {
        auto materials = scene.GetMaterials();
        if ((file_line[0] == 'v') && (file_line[1] != 'n') && (file_line[1] != 't')) {
            auto res = ReadThreeNumbers(file_line);
            Vector vector = res;
            vertexes.push_back(vector);
            continue;
        }
        if ((file_line[0] == 'v') && (file_line[1] == 'n')) {
            auto res = ReadThreeNumbers(file_line);
            Vector vector = res;
            normals.push_back(vector);
            continue;
        }
        if ((file_line.empty()) || (file_line[0] == '#') || (file_line[0] == 'g') ||
            (file_line[0] == 's') || (file_line[0] == 'o')) {
            continue;
        }
        if (file_line[0] == 'P') {
            scene.AddLight(CreateLight(ReadNumbers(file_line)));
            continue;
        }
        std::string mtl;
        mtl = ReadMaterialField(file_line);
        if (mtl == "mtllib") {
            auto name = ParsName(file_line);
            scene.AddMaterials(ReadMaterials(path_name + "/" + name));
        }
        if (mtl == "usemtl") {
            curr_name = ParsName(file_line);
        }
        if (FirstChar(file_line) == 'S') {  // сферический объект
            scene.AddSphereObjects(SphereObject(scene.GetMaterialPointer(curr_name),
                                                CreateSphere(ReadNumbers(file_line))));
        }
        if (FirstChar(file_line) == 'f') {
            std::vector<Object> new_objects;
            AddObject(new_objects, file_line, vertexes, normals, scene, curr_name);
            for (const auto& item : new_objects) {
                scene.AddObject(item);
            }
        }
    }
    return scene;
}
