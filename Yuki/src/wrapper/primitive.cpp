#include <primitive.h>
#include <intersection.h>
#include <sphere.h>

namespace Yuki {
    uint32_t Primitive::next_primitive_id = 1;

    /* GeometricPrimitive */

    GeometricPrimitive::GeometricPrimitive(const shared_ptr<Shape> &s
        /*const Reference<Material> &m,
        AreaLight *a*/) {
        shape = s;
        /*
        material = m;
        area_light = a;
        */
    }

    BBox GeometricPrimitive::world_bound() const {
        return shape->world_bound();
    }

    bool GeometricPrimitive::intersect(const Ray &r, Intersection *isect) const {
        Float t_hit, ray_epsilon;
        if (!shape->intersect(r, &t_hit, isect)) 
            return false;
        
        r.max_t = t_hit;

        return true;
    }

    bool GeometricPrimitive::intersect_p(const Ray &r) const {
        return shape->intersect_p(r);
    }

    bool GeometricPrimitive::can_intersect() const {
        return shape->can_intersect();
    }


    shared_ptr<Primitive> make_shape(string type, Float *args) {
        shared_ptr<Primitive> ret;
        if (type == "sphere") {
            Float radius = args[0];
            Point center(args[1], args[2], args[3]);
            shared_ptr<Transform> object_to_world(new Transform(Transform::translate(Vector(center))));
            shared_ptr<Transform> world_to_object(new Transform(inverse(*object_to_world)));
            shared_ptr<Shape> sphere(new Sphere(object_to_world,  world_to_object, false, radius, -radius, radius, 360.f));
            sphere->emission = vec3<Float>(args[4], args[5], args[6]);
            sphere->color    = vec3<Float>(args[7], args[8], args[9]);
            sphere->reflect_type = args[10];
            GeometricPrimitive *p = new GeometricPrimitive(sphere);
            ret = shared_ptr<Primitive>(p);
        }
        return ret;
    }
}