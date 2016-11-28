/* -----------------------------
* File   : geometry.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_GEOMETRY_H__
#define __YUKI_GEOMETRY_H__

#include <core.h>
#include <vec.h>

namespace Yuki {
	class Vector;
	class Normal;
	class Point;

	class Vector : public vec3<Float> {
	public:
		Vector() {}
		Vector(Float a, Float b, Float c) :vec3(a, b, c) {}
		Vector(const vec3<Float> &v) : vec3(v.x, v.y, v.z) {}
		Vector(const Vector &v) : vec3(v.x, v.y, v.z) {}
		explicit Vector(const Normal &n);
		explicit Vector(const Point  &p);
		~Vector() {}

#ifdef DEBUG
		friend ostream &operator<<(ostream &out, const Vector &v) {
			out << (vec3&)(v);
			return out;
		}
#endif
	};


	/* Point */
	class Point : public vec3<Float> {
	public:
		Point() {}
		Point(Float a, Float b, Float c) : vec3(a, b, c) {}
		Point(const vec3<Float> &v) : vec3(v) {}
		Point(const Point &p) : vec3(p.x, p.y, p.z) {}
		~Point() {}

#ifdef DEBUG
		friend ostream &operator<<(ostream &out, const Point &v) {
			out << (vec3&)(v);
			return out;
		}
#endif
	};

	inline Float distance(const Point &p1, const Point &p2) {
		return (p1 - p2).length();
	}
	inline Float distance_squared(const Point &p1, const Point &p2) {
		return (p1 - p2).length_squared();
	}

	/* Normal */
	class Normal : public vec3<Float> {
	public:
		Normal() {}
		Normal(Float a, Float b, Float c) : vec3(a, b, c) {}
		Normal(const vec3 &v) : vec3(v) {}
		Normal(const Normal &n) : vec3(n.x, n.y, n.z) {}
		explicit Normal(const Vector& v) : vec3(v.x, v.y, v.z) {}
		~Normal() {}

	};

	inline Normal face_forward(const Normal &n, const Vector &v) {
		if ((n * v) < 0.f) return -n; else return n;
	}

	class Ray {
	public:
		Point o;
		Vector d;
		mutable Float max_t;
		Float time;

        Ray() : max_t(Infinity), time(0.f) {}
        Ray(const Point &o, const Vector &d, Float t_max = Infinity, Float time = 0.f)
            : o(o), d(d), max_t(t_max), time(time) {}

		Point operator()(Float t) const {
			return o + d * t;
		}
	};

	class RayDifferential : public Ray {
	public:
		// members
		bool has_differentials;
		Point rx_origin, ry_origin;
		Vector rx_direction, ry_direction;
		// constructors
		RayDifferential() { has_differentials = false; }
		RayDifferential(const Point &org, const Vector &dir,
			Float end = INFINITY, Float t = 0.f)
			: Ray(org, dir, end, t) {
			has_differentials = false;
		}
		explicit RayDifferential(const Ray & ray) : Ray(ray) {
			has_differentials = false;
		}
		void scale_differnetials(Float s) {
			rx_origin = o + (rx_origin - o) * s;
			ry_origin = o + (ry_origin - o) * s;
			rx_direction = d + (rx_direction - d) * s;
			ry_direction = d + (ry_direction - d) * s;
		}
	};

	class BBox {
	public:
		Point p_min, p_max;
		BBox() {
			p_min = Point(INFINITY, INFINITY, INFINITY);
			p_max = Point(-INFINITY, -INFINITY, -INFINITY);
		}
		BBox(const Point &p) : p_min(p), p_max(p) {}
		BBox(const Point &p1, const Point &p2) {
			p_min = Point(std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z));
			p_max = Point(std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z));
		}
		bool overlaps(const BBox &b) const {
			bool x = (p_max.x >= b.p_min.x) && (p_min.x <= b.p_max.x);
			bool y = (p_max.y >= b.p_min.y) && (p_min.y <= b.p_max.y);
			bool z = (p_max.z >= b.p_min.z) && (p_min.z <= b.p_max.z);
			return (x && y && z);
		}
		bool inside(const Point &pt) const {
			return (pt.x >= p_min.x && pt.x <= p_max.x &&
				pt.y >= p_min.y && pt.y <= p_max.y &&
				pt.z >= p_min.z && pt.z <= p_max.z);
		}
		void expand(Float delta) {
			Vector vector_delta(delta, delta, delta);
			p_min -= vector_delta;
			p_max += vector_delta;
		}
		Float surface_area() const {
			Vector d = p_max - p_min;
			return 2.f * (d.x * d.y + d.x * d.z + d.y * d.z);
		}
		Float volume() const {
			Vector d = p_max - p_min;
			return d.x * d.y * d.z;
		}
		// the index of axis with max extent
		int maximum_extent() const {
			Vector diag = p_max - p_min;
			if (diag.x > diag.y && diag.x > diag.z)
				return 0;
			else if (diag.y > diag.z)
				return 1;
			else
				return 2;
		}
		const Point &operator[](int i) const {
			CHECK((i >= 0 && i <= 1));
			if (i == 0) return p_min;
			else return p_max;
		}
		Point &operator[](int i) {
			CHECK((i >= 0 && i <= 1));
			if (i == 0) return p_min;
			else return p_max;
		}
		Point lerp(Float tx, Float ty, Float tz) const {
			return Point(Yuki::lerp(tx, p_min.x, p_max.x),
						 Yuki::lerp(ty, p_min.y, p_max.y),
						 Yuki::lerp(tz, p_min.z, p_max.z));
		}
		Vector offset(const Point &p) const {
			return Vector(
				(p.x - p_min.x) / (p_max.x - p_min.x),
				(p.y - p_min.y) / (p_max.y - p_min.y),
				(p.z - p_min.z) / (p_max.z - p_min.z));
		}
		// get the bouding shpere of the bbox
		void bounding_sphere(Point *c, Float *rad) const {
			*c = 0.5f * p_min + 0.5f * p_max;
			*rad = inside(*c) ? distance(*c, p_max) : 0.f;
		}

		bool intersect_p(const Ray &r, Float *hit_t0, Float *hit_t1) const;
	};

	inline Float distance_squared(const Point &p, const BBox &b) {
		Float dx = std::max({ Float(0), b.p_min.x - p.x, p.x - b.p_max.x });
		Float dy = std::max({ Float(0), b.p_min.y - p.y, p.y - b.p_max.y });
		Float dz = std::max({ Float(0), b.p_min.z - p.z, p.z - b.p_max.z });
		return dx * dx + dy * dy + dz * dz;
	}

	inline Float distance(const Point &p, const BBox &b) {
		return std::sqrt(distance_squared(p, b));
	}

	BBox union_BBox(const BBox &b, const Point &p);
	BBox union_BBox(const BBox &b1, const BBox &b2);
	BBox intersect_BBox(const BBox &b1, const BBox &b2);

    // pbrt-v3 implement
    inline Point offset_ray_origin(const Point &p, const Vector &p_error,
                                   const Normal &n, const Vector &w) {
        Float d = (abs(n) * p_error);

        // We have tons of precision; for now bump up the offset a bunch just
        // to be extra sure that we start on the right side of the surface
        // (In case of any bugs in the epsilons code...)
        if (sizeof(Float) >= sizeof(double)) {
            d *= 1024.;
        }
        Vector offset = d * Vector(n);
        if ((w * n) < 0) offset = -offset;
        Point po = p + offset;
        // Round offset point _po_ away from _p_
        for (int i = 0; i < 3; ++i) {
            if (offset[i] > 0)
                po[i] = next_float_up(po[i]);
            else if (offset[i] < 0)
                po[i] = next_float_down(po[i]);
        }
        return po;
    }
}

#endif  // !__YUKI_GEOMETRY_H__
