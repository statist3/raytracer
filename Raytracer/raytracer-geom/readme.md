The following functions are implemented in `geometry.h`:

* `GetIntersection`, which finds the intersection of a ray and a sphere or a ray and a triangle. Read that the triangle is non-degenerate.
* `Reflect(ray,normal)`, taking into account the reflection from the `ray` ray at a location normal to the surface, where it is equal to `normal`.
* `Refract(ray,normal,eta)`, which counts the ray refracted to `ray` (if any). Here `normal` is normal at the point where the ray hits another medium, and `eta` is equal to $`\frac{n_1}{n_2}`$, where $`n_1, n_2`$ are refractive indices the medium from which the beam comes and the medium into which it goes, respectively. In the Reflection and Refraction functions, you can determine that the ray and vector are normalized, and the normal is oriented in the opposite direction of the ray.
* `GetBarycentricCoords`, which calculates barycentric coordinate points relative to a given triangle (guaranteed that the point lies inside or on the border of the triangle).
