/* -----------------------------
* File   : shape.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_SHAPE_H__
#define __YUKI_SHAPE_H__

#include <log.h>
#include <core.h>
#include <geometry.h>
#include <transform.h>
#include <vector>
#include <intersection.h>
using std::vector;

namespace Yuki {
    class Shape {
    public:
        const shared_ptr<Transform> object_to_world, world_to_object;
        const bool reverse_orientation, transform_swaps_handedness;
        const uint32_t shape_id;
        static uint32_t next_shape_id;

        // temp
        vec3<Float> emission;
        vec3<Float> color;
        int         reflect_type;

        Shape(const shared_ptr<Transform> o2w, const shared_ptr<Transform> w2o, bool ro):
            object_to_world(o2w), world_to_object(w2o), reverse_orientation(ro),
            transform_swaps_handedness(o2w->swaps_handedness()),
            shape_id(next_shape_id++) {}
        Shape(const Shape &s) :
            object_to_world(s.object_to_world),
            world_to_object(s.world_to_object),
            reverse_orientation(s.reverse_orientation),
            transform_swaps_handedness(s.transform_swaps_handedness),
            shape_id(s.shape_id) {}
        virtual ~Shape() {}

        // interface
        virtual BBox object_bound() const = 0;
        BBox world_bound() const { return (*object_to_world)(object_bound()); }
        bool can_intersect() const { return true; }

        virtual void refine(vector<shared_ptr<Shape> > &refined) const {
            LOG::error("Not implemented Shape::refine().");
            return;
        }

        virtual bool intersect(const Ray &r, Float *t_hit, Intersection *isect) const {
            LOG::error("Not implemented Shape::intersect().");
            return false;
        }

        virtual bool intersect_p(const Ray &ray) const {
            LOG::error("Not implemented Shape::intersectP().");
            return false;
        }

        virtual Float area() const {
            LOG::error("Not implemented Shape::area().");
            return 0.f;
        }
    };

    class Sphere;
    /*class Cylinder;
    class Disk;
    class Paraboloid;
    class Hyperboloid;
    class Cone;
    class Triangle;
    class TriangleMesh;*/
}

#endif