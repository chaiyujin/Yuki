/* -----------------------------
* File   : vec.h
* Author : Yuki Chai
* Created: 2016.11.28
* Project: Yuki
*/
#pragma once
#ifndef __YUKI_VEC_H__
#define __YUKI_VEC_H__

#include <core.h>
#include <log.h>

namespace Yuki {
	template <class T>
	class vec2 {
	public:
		/* members */
		T x, y;

		/* constructors */

		vec2() : x(0), y(0) {}
		vec2(T a, T b) : x(a), y(b) {}
		vec2(const vec2 & v) : x(v.x), y(v.y) {}
		virtual ~vec2() {}

		/* operators */

		vec2 operator-() const {
			return vec2(-x, -y);
		}
		const vec2 &operator=(const vec2 &v) {
			x = v.x;
			y = v.y;
			return *this;
		}
		bool operator==(const vec2 &v) {
			return equal_zero(x - v.x) &&
				equal_zero(y - v.y);
		}
		bool operator!=(const vec2 &v) {
			return !equal_zero(x - v.x) ||
				!equal_zero(y - v.y);
		}
		vec2 operator+(const vec2 &v) const {
			return vec2(x + v.x, y + v.y);
		}
		vec2 operator-(const vec2 &v) const {
			return vec2(x - v.x, y - v.y);
		}
		vec2 operator*(T f) const {
			return vec2(x * f, y * f);
		}
		vec2 operator/(T f) const {
			CHECK(f != 0);
			T inv = (T)1.0 / f;
			return vec2(x * inv, y * inv);
		}
		vec2 &operator+=(const vec2 &v) {
			x += v.x; y += v.y;
			return *this;
		}
		vec2 &operator-=(const vec2 &v) {
			x -= v.x; y -= v.y;
			return *this;
		}
		vec2 &operator*=(T f) {
			x *= f; y *= f;
			return *this;
		}
		vec2 &operator/=(T f) {
			CHECK(f != 0);
			T inv = (T)1.0 / f;
			x *= inv; y *= inv;
			return *this;
		}

		/* dot and cross */
		// dot
		T operator*(const vec2 &v) const {
			return x * v.x +
				y * v.y;
		}
		// cross
		T operator^(const vec2 &v) const {
			return x * v.y - y * v.x;
		}

		// normalize
		vec2 normalized() const {
			return (*this) / (*this).length();
		}
		void normalize() {
			(*this) /= (*this).length();
		}
		/* Get from [] */
		T &operator[](int index) {
			CHECK((index >= 0 && index <= 1));
			if (index == 0) return x;
			else return y;
		}
		const T &operator[](int index) const {
			CHECK((index >= 0 && index <= 1));
			if (index == 0) return x;
			else return y;
		}

		Float length_squared() const {
			return x * x + y * y;
		}
		Float length() const {
			return sqrt(length_squared());
		}

		/* print method */
#ifdef DEBUG
		template <class T>
		friend ostream &operator<<(ostream &out, const vec2<T> &v) {
			out << "[" << v.x << ", " << v.y << "] ";
			return out;
		}
#endif
	};

	template <class T>
	class vec3 {
	public:
		/* members */
		T x, y, z;

		/* constructors */

		vec3() : x(0), y(0), z(0) {}
		vec3(T a, T b, T c) : x(a), y(b), z(c) {}
		vec3(const vec3 &v) : x(v.x), y(v.y), z(v.z) {
			//printf("vec3(copy)\n");
		}
		virtual ~vec3() {}

		/* operators */

		vec3 operator-() const {
			return vec3(-x, -y, -z);
		}
		const vec3 &operator=(const vec3 &v) {
			x = v.x;
			y = v.y;
			z = v.z;
			return *this;
		}
		bool operator==(const vec3 &v) {
			return equal_zero(x - v.x) &&
				equal_zero(y - v.y) &&
				equal_zero(z - v.z);
		}
		bool operator!=(const vec3 &v) {
			return !equal_zero(x - v.x) ||
				!equal_zero(y - v.y) ||
				!equal_zero(z - v.z);
		}
		vec3 operator+(const vec3 &v) const {
			return vec3(x + v.x, y + v.y, z + v.z);
		}
		vec3 operator-(const vec3 &v) const {
			return vec3(x - v.x, y - v.y, z - v.z);
		}
		vec3 operator*(T f) const {
			return vec3(x * f, y * f, z * f);
		}
		vec3 operator/(T f) const {
			CHECK(f != 0);
			T inv = (T)1.0 / f;
			return vec3(x * inv, y * inv, z * inv);
		}
		vec3 &operator+=(const vec3 &v) {
			x += v.x; y += v.y; z += v.z;
			return *this;
		}
		vec3 &operator-=(const vec3 &v) {
			x -= v.x; y -= v.y; z -= v.z;
			return *this;
		}
		vec3 &operator*=(T f) {
			x *= f; y *= f; z *= f;
			return *this;
		}
		vec3 &operator/=(T f) {
			CHECK(f != 0);
			T inv = (T)1.0 / f;
			x *= inv; y *= inv; z *= inv;
			return *this;
		}

		/* dot and cross */
		// dot
		T operator*(const vec3 &v) const {
			return x * v.x +
				y * v.y +
				z * v.z;
		}
		// cross
		vec3 operator^(const vec3 &v) const {
			return vec3(
				(y * v.z) - (z * v.y),
				(z * v.x) - (x * v.z),
				(x * v.y) - (y * v.x)
			);
		}
		vec3 operator&(const vec3 &v) const {
			return vec3(
				x * v.x,
				y * v.y,
				z * v.z
			);
		}

		// normalize
		vec3 normalized() const {
			return (*this) / (*this).length();
		}
		void normalize() {
			(*this) /= (*this).length();
		}

		/* Get from [] */
		T &operator[](int index) {
			CHECK((index >= 0 && index <= 2));
			if (index == 0) return x;
			else if (index == 1) return y;
			else return z;
		}
		const T &operator[](int index) const {
			CHECK((index >= 0 && index <= 2));
			if (index == 0) return x;
			else if (index == 1) return y;
			else return z;
		}

		Float length_squared() const {
			return (Float)(x * x + y * y + z * z);
		}
		Float length() const {
			return (Float)(sqrt(length_squared()));
		}

		/* print method */
#ifdef DEBUG
		template <class T>
		friend ostream &operator<<(ostream &out, const vec3<T> &v) {
			out << "[" << v.x << ", " << v.y << ", " << v.z << "] ";
			return out;
		}
#endif
	};
}

#endif  // !__YUKI_VEC_H__