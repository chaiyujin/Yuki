/* -----------------------------
* File   : geometry.cpp
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#include <geometry.h>

namespace Yuki {
	Vector::Vector(const Normal &n) : vec3(n.x, n.y, n.z) {}
	Vector::Vector(const Point  &p) : vec3(p.x, p.y, p.z) {}

	BBox union_BBox(const BBox &b, const Point &p) {
		BBox ret = b;
		ret.p_min.x = std::min(b.p_min.x, p.x);
		ret.p_min.y = std::min(b.p_min.y, p.y);
		ret.p_min.z = std::min(b.p_min.z, p.z);
		ret.p_max.x = std::max(b.p_max.x, p.x);
		ret.p_max.y = std::max(b.p_max.y, p.y);
		ret.p_max.z = std::max(b.p_max.z, p.z);
		return ret;
	}

	BBox union_BBox(const BBox &b, const BBox &b2) {
		BBox ret = b;
		ret.p_min.x = std::min(b.p_min.x, b2.p_min.x);
		ret.p_min.y = std::min(b.p_min.y, b2.p_min.y);
		ret.p_min.z = std::min(b.p_min.z, b2.p_min.z);
		ret.p_max.x = std::max(b.p_max.x, b2.p_max.x);
		ret.p_max.y = std::max(b.p_max.y, b2.p_max.y);
		ret.p_max.z = std::max(b.p_max.z, b2.p_max.z);
		return ret;
	}

	BBox intersect_BBox(const BBox &b1, const BBox &b2) {
		return BBox(
			Point(std::max(b1.p_min.x, b2.p_min.x),
				  std::max(b1.p_min.y, b2.p_min.y),
				  std::max(b1.p_min.z, b2.p_min.z)),
			Point(std::min(b1.p_max.x, b2.p_max.x),
				  std::min(b1.p_max.y, b2.p_max.y),
				  std::min(b1.p_max.z, b2.p_max.z)));
	}

	// pbrt-v3 implement
	bool BBox::intersect_p(const Ray &ray, Float *hit_t0, Float *hit_t1) const {
		Float t0 = 0, t1 = ray.max_t;
		for (int i = 0; i < 3; ++i) {
			// Update interval for _i_th bounding box slab
			Float inv_ray_dir = 1 / ray.d[i];
			Float t_near = (p_min[i] - ray.o[i]) * inv_ray_dir;
			Float t_far  = (p_max[i] - ray.o[i]) * inv_ray_dir;

			// Update parametric interval from slab intersection $t$ values
			if (t_near > t_far) std::swap(t_near, t_far);

			// Update _t_far_ to ensure robust ray--bounds intersection
			t_far *= 1 + 2 * epsilon_gamma(3);
			t0 = t_near > t0 ? t_near : t0;
			t1 = t_far  < t1 ? t_far  : t1;
			if (t0 > t1) return false;
		}
		if (hit_t0) *hit_t0 = t0;
		if (hit_t1) *hit_t1 = t1;
		return true;
	}
}