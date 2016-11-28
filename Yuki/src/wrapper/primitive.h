#pragma once
#ifndef __KuroYuki_Wrapper_Primitive_H__
#define __KuroYuki_Wrapper_Primitive_H__

#include <core.h>
#include <geometry.h>
#include <shape.h>
#include <vector>
#include <memory>
#include <string>
using std::string;
using std::vector;
using std::shared_ptr;

namespace Yuki {
    class Primitive  {
    protected:

    public:
        static  uint32_t    next_primitive_id; 
        const   uint32_t    primitive_id;

        Primitive() : primitive_id(next_primitive_id++) {}
        virtual BBox world_bound() const = 0;
        virtual bool can_intersect() const {
            return true;
        }
        virtual bool intersect(const Ray &r, Intersection *in) const = 0;
        virtual bool intersect_p(const Ray &r) const = 0;
        


    };

    class GeometricPrimitive : public Primitive {
    private:
        shared_ptr<Shape> shape;
    public:
        GeometricPrimitive(const shared_ptr<Shape> &s);
        BBox world_bound() const;
        bool intersect(const Ray &r, Intersection *isect) const;
        bool intersect_p(const Ray &r) const;
        bool can_intersect() const;

    };

    shared_ptr<Primitive> make_shape(string type, Float *args);
}

#endif // !__KuroYuki_Wrapper_Primitive_H__
