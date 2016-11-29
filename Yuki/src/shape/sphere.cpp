/* -----------------------------
* File   : sphere.cpp
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#include <sphere.h>

namespace Yuki {
    Sphere::Sphere(const shared_ptr<Transform> o2w, const shared_ptr<Transform> w2o, bool ro,
        Float rad, Float z0, Float z1, Float pm)
        : Shape(o2w, w2o, ro) {
        radius = rad;
        z_min = clamp(std::min(z0, z1), -radius, radius);
        z_max = clamp(std::max(z0, z1), -radius, radius);
        theta_min = (Float)std::acos(clamp(z_min / radius, -1.f, 1.f));
        theta_max = (Float)std::acos(clamp(z_max / radius, -1.f, 1.f));
        phi_max = radians(clamp(pm, 0.f, 360.f));
    }

    Sphere::Sphere(const Sphere &sp) : 
        Shape(sp),
        radius(sp.radius), phi_max(sp.phi_max),
        z_min(sp.z_min), z_max(sp.z_max), 
        theta_min(sp.theta_min), theta_max(sp.theta_max) {}

    BBox Sphere::object_bound() const {
        return BBox(Point(-radius, -radius, z_min),
            Point( radius,  radius, z_max));
    }

    bool Sphere::intersect(const Ray &r, Float *t_hit, Intersection *isect) const {
        Float phi;
        Point p_hit;
        // Transform _Ray_ to object space
        Vector o_err, d_err;
        Ray ray = (*world_to_object)(r, &o_err, &d_err);

        // Compute quadratic sphere coefficients

        // Initialize _EFloat_ ray coordinate values
        EFloat ox(ray.o.x, o_err.x), oy(ray.o.y, o_err.y), oz(ray.o.z, o_err.z);
        EFloat dx(ray.d.x, d_err.x), dy(ray.d.y, d_err.y), dz(ray.d.z, d_err.z);
        EFloat a = dx * dx + dy * dy + dz * dz;
        EFloat b = 2 * (dx * ox + dy * oy + dz * oz);
        EFloat c = ox * ox + oy * oy + oz * oz - EFloat(radius) * EFloat(radius);

        // Solve quadratic equation for _t_ values
        EFloat t0, t1;
        if (!quadratic(a, b, c, &t0, &t1)) return false;

        // Check quadric shape _t0_ and _t1_ for nearest intersection
        if (t0.upper_bound() > ray.max_t || t1.lower_bound() <= 0) return false;
        EFloat t_shape_hit = t0;
        if (t_shape_hit.lower_bound() <= 0) {
            t_shape_hit = t1;
            if (t_shape_hit.upper_bound() > ray.max_t) return false;
        }

        // Compute sphere hit position and $\phi$
        p_hit = ray((Float)t_shape_hit);

        // Refine sphere intersection point
        p_hit *= radius / distance(p_hit, Point(0, 0, 0));
        if (p_hit.x == 0 && p_hit.y == 0) p_hit.x = 1e-5f * radius;
        phi = std::atan2(p_hit.y, p_hit.x);
        if (phi < 0) phi += 2 * M_PI;

        // Test sphere intersection against clipping parameters
        if ((z_min > -radius && p_hit.z < z_min) || (z_max < radius && p_hit.z > z_max) ||
            phi > phi_max) {
            if (t_shape_hit == t1) return false;
            if (t1.upper_bound() > ray.max_t) return false;
            t_shape_hit = t1;
            // Compute sphere hit position and $\phi$
            p_hit = ray((Float)t_shape_hit);

            // Refine sphere intersection point
            p_hit *= radius / distance(p_hit, Point(0, 0, 0));
            if (p_hit.x == 0 && p_hit.y == 0) p_hit.x = 1e-5f * radius;
            phi = std::atan2(p_hit.y, p_hit.x);
            if (phi < 0) phi += 2 * M_PI;
            if ((z_min > -radius && p_hit.z < z_min) ||
                (z_max < radius && p_hit.z > z_max) || phi > phi_max)
                return false;
        }

        // Compute error bounds for sphere intersection
        Vector p_error = epsilon_gamma(5) * abs(p_hit);

        // Initialize _SurfaceInteraction_ from parametric information
        Normal n = (p_hit - Point()).normalized();
        *isect = (*object_to_world)(Intersection(p_hit, p_error, n, -ray.d, ray.time));
        isect->color    = this->color;
        isect->emission = this->emission;
        isect->refl_t   = this->reflect_type;

        // Update _tHit_ for quadric intersection
        *t_hit = (Float)t_shape_hit;
        return true;
    }

    bool Sphere::intersect_p(const Ray &r) const {
        Float phi;
        Point p_hit;
        // Transform _Ray_ to object space
        Vector o_err, d_err;
        Ray ray = (*world_to_object)(r, &o_err, &d_err);

        // Compute quadratic sphere coefficients

        // Initialize _EFloat_ ray coordinate values
        EFloat ox(ray.o.x, o_err.x), oy(ray.o.y, o_err.y), oz(ray.o.z, o_err.z);
        EFloat dx(ray.d.x, d_err.x), dy(ray.d.y, d_err.y), dz(ray.d.z, d_err.z);
        EFloat a = dx * dx + dy * dy + dz * dz;
        EFloat b = 2 * (dx * ox + dy * oy + dz * oz);
        EFloat c = ox * ox + oy * oy + oz * oz - EFloat(radius) * EFloat(radius);

        // Solve quadratic equation for _t_ values
        EFloat t0, t1;
        if (!quadratic(a, b, c, &t0, &t1)) return false;

        // Check quadric shape _t0_ and _t1_ for nearest intersection
        if (t0.upper_bound() > ray.max_t || t1.lower_bound() <= 0) return false;
        EFloat t_shape_hit = t0;
        if (t_shape_hit.lower_bound() <= 0) {
            t_shape_hit = t1;
            if (t_shape_hit.upper_bound() > ray.max_t) return false;
        }

        // Compute sphere hit position and $\phi$
        p_hit = ray((Float)t_shape_hit);

        // Refine sphere intersection point
        p_hit *= radius / distance(p_hit, Point(0, 0, 0));
        if (p_hit.x == 0 && p_hit.y == 0) p_hit.x = 1e-5f * radius;
        phi = std::atan2(p_hit.y, p_hit.x);
        if (phi < 0) phi += 2 * M_PI;

        // Test sphere intersection against clipping parameters
        if ((z_min > -radius && p_hit.z < z_min) || (z_max < radius && p_hit.z > z_max) ||
            phi > phi_max) {
            if (t_shape_hit == t1) return false;
            if (t1.upper_bound() > ray.max_t) return false;
            t_shape_hit = t1;
            // Compute sphere hit position and $\phi$
            p_hit = ray((Float)t_shape_hit);

            // Refine sphere intersection point
            p_hit *= radius / distance(p_hit, Point(0, 0, 0));
            if (p_hit.x == 0 && p_hit.y == 0) p_hit.x = 1e-5f * radius;
            phi = std::atan2(p_hit.y, p_hit.x);
            if (phi < 0) phi += 2 * M_PI;
            if ((z_min > -radius && p_hit.z < z_min) ||
                (z_max < radius && p_hit.z > z_max) || phi > phi_max)
                return false;
        }

        return true;
    }

    Float Sphere::area() const {
        return phi_max * radius * (z_max - z_min);
    }
}