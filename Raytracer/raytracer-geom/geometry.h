#include <vector.h>
#include <sphere.h>
#include <intersection.h>
#include <triangle.h>
#include <ray.h>
#include <cmath>
#include <optional>

const double kEps = 0.000001;

std::optional<Intersection> GetIntersection(const Ray& ray, const Sphere& sphere) {
    Vector oc = ray.GetOrigin() - sphere.GetCenter();
    double a = DotProduct(ray.GetDirection(), ray.GetDirection());
    double b = 2.0 * DotProduct(oc, ray.GetDirection());
    double c = DotProduct(oc, oc) - sphere.GetRadius() * sphere.GetRadius();
    double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return std::nullopt;  // Нет решения, луч не пересекает сферу
    }

    // Находим корни уравнения
    double sqrtDiscriminant = sqrt(discriminant);
    double t1 = (-b - sqrtDiscriminant) / (2.0 * a);
    double t2 = (-b + sqrtDiscriminant) / (2.0 * a);

    // Находим ближайшее положительное t
    double t = (t1 < kEps && t2 < kEps) ? std::numeric_limits<double>::max() :
               (t1 < kEps) ? t2 : (t2 < kEps) ? t1 : std::min(t1, t2);

    if (t >= std::numeric_limits<double>::max()) {
        return std::nullopt; // Нет подходящих решений
    }

    // Вычисляем точку пересечения и нормаль в этой точке
    Vector position = ray.GetOrigin() + t * ray.GetDirection();
    Vector normal = (position - sphere.GetCenter()).Normalize();

    // Корректируем направление нормали, если оно направлено в сторону луча
    if (DotProduct(ray.GetDirection(), normal) > 0) {
        normal = -normal;
    }

    return Intersection(position, normal, t);
}

std::optional<Intersection> GetIntersection(const Ray& ray, const Triangle& triangle) {
    Vector vertex0 = triangle.GetVertex(0);
    Vector vertex1 = triangle.GetVertex(1);
    Vector vertex2 = triangle.GetVertex(2);
    Vector edge1, edge2, h, s, q;
    double a, f, u, v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = CrossProduct(ray.GetDirection(), edge2);
    a = DotProduct(edge1, h);
    if (a > -kEps && a < kEps) {
        return {};
    }
    f = 1.0 / a;
    s = ray.GetOrigin() - vertex0;
    u = f * DotProduct(s, h);
    if (u < 0.0 || u > 1.0) {
        return {};
    }
    q = CrossProduct(s, edge1);
    v = f * DotProduct(ray.GetDirection(), q);
    if (v < 0.0 || u + v > 1.0) {
        return {};
    }
    double t = f * DotProduct(edge2, q);
    if (t > kEps) {
        Vector position = ray.GetOrigin() + t * ray.GetDirection();  // точка
        double dist = t;
        Vector norm = CrossProduct(triangle.GetVertex(1) - triangle.GetVertex(0),
                                   triangle.GetVertex(2) - triangle.GetVertex(0));
        norm.Normalize();
        if (DotProduct(norm, -1 * ray.GetDirection()) <= 0.0) {
            norm = -1 * norm;
        }
        return Intersection(position, norm, dist);
    } else {
        return {};
    }
}

std::optional<Vector> Refract(const Vector& ray, const Vector& normal, double eta) {
    double cos = DotProduct(normal, -1 * ray) / Length(normal) / Length(ray);
    double acos = std::acos(cos);
    double asin = std::asin(1.0 / eta);

    if (acos > asin) {
        return {};
    }

    Vector refract = eta * ray + (eta * cos - sqrt(1 - eta * eta * (1 - cos * cos))) * normal;
    refract.Normalize();
    return refract;
}
Vector Reflect(const Vector& ray, const Vector& normal) {
    Vector reflect = ray + (-2 * DotProduct(normal, ray)) * normal;
    reflect.Normalize();
    return reflect;
}
Vector GetBarycentricCoords(const Triangle& triangle, const Vector& point) {
    // 0 == A, 1 == B, 2 == C
    Vector res;
    double area = triangle.Area();
    Vector pa = triangle.GetVertex(0) - point;
    Vector pc = triangle.GetVertex(2) - point;
    Vector pb = triangle.GetVertex(1) - point;
    res[0] = Length(CrossProduct(pc, pb)) / 2 / area;
    res[1] = Length(CrossProduct(pa, pc)) / 2 / area;
    res[2] = Length(CrossProduct(pb, pa)) / 2 / area;
    return res;
}
