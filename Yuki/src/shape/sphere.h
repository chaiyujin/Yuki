/* -----------------------------
* File   : sphere.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_SPHERE_H__
#define __YUKI_SPHERE_H__ 

#include <efloat.h>
#include <shape.h>

namespace Yuki {
    class Sphere : public Shape {
    protected:
        Float radius;
        Float phi_max;
        Float z_min, z_max;
        Float theta_min, theta_max;
    public:
        Sphere(const shared_ptr<Transform> o2w, const shared_ptr<Transform> w2o, bool ro,
               Float rad, Float z0, Float z1, Float pm);
        Sphere(const Sphere &sp);

        BBox object_bound() const;
        bool intersect(const Ray &r, Float *t_hit, Intersection *isect) const;
        bool intersect_p(const Ray &r) const;
        Float area() const;
    };

}

#endif // !__YUKI_SPHERE_H___
