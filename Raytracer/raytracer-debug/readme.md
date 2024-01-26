# Raytracer P3
Simplified raytracer operating modes have been implemented, which helped to catch the main errors when working with the camera, ray tracing and geometry implementation.

In `RenderMode::kDepth` mode, when tracing a ray, the resulting pixel color will be the distance to the nearest object that was hit by the ray. Suppose that when a ray is emitted from the camera through the next pixel, this ray hits an object at a distance $`d`$. Then the resulting value for this pixel will be the triple $`(d, d, d)`$.

In `RenderMode::kNormal` mode, when tracing a ray, the resulting color will be the value of the normal at the point where the ray intersects the nearest object. In this case, the normal should be directed in the direction opposite to the beam. Since the normal vector is normalized, all coordinates will be in the range [-1, 1].
