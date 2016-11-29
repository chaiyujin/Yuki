#pragma once
#ifndef __KY_INTERSECTION_H__
#define __KY_INTERSECTION_H__

#include <core.h>

namespace Yuki {
    class Intersection {
    public:
        Point p;
        Float time;
        Vector p_error;
        Vector wo;
        Normal n;
        vec3<Float> color;
        vec3<Float> emission;
        int         refl_t;

        // method
        Intersection() : time(0) {}
        Intersection(const Point &p, const Vector &p_error,
                     const Normal &n, const Vector &wo,
                     Float time)
            : p(p), time(time), p_error(p_error),
            wo(wo.normalized()), n(n.normalized()) {}

        Ray spawn_ray(const Vector &d) const {
            Point o = offset_ray_origin(p, p_error, n, d);
            return Ray(o, d, Infinity, time);
        }

    };
}

#endif // !__KuroYuki_Core_Intersection_H__
