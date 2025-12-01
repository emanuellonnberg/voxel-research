#pragma once

#include <cmath>
#include <iostream>
#include <functional>

/**
 * Vector3 - 3D vector for positions, directions, and basic linear algebra
 *
 * Used throughout the voxel system for:
 * - Voxel positions in world space
 * - Directions (normals, rays)
 * - Forces and displacements (structural analysis)
 * - Physics velocities and accelerations
 */
class Vector3 {
public:
    float x, y, z;

    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

    // Assignment
    Vector3& operator=(const Vector3& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    // Arithmetic operators
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    Vector3 operator/(float scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    Vector3 operator-() const {
        return Vector3(-x, -y, -z);
    }

    // Compound assignment operators
    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    // Comparison operators
    bool operator==(const Vector3& other) const {
        const float epsilon = 1e-6f;
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }

    bool operator!=(const Vector3& other) const {
        return !(*this == other);
    }

    // Vector operations
    float Dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3 Cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    float Length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    float LengthSquared() const {
        return x * x + y * y + z * z;
    }

    Vector3 Normalized() const {
        float len = Length();
        if (len < 1e-6f) return Vector3(0, 0, 0);
        return *this / len;
    }

    void Normalize() {
        float len = Length();
        if (len > 1e-6f) {
            *this /= len;
        }
    }

    float Distance(const Vector3& other) const {
        return (*this - other).Length();
    }

    float DistanceSquared(const Vector3& other) const {
        return (*this - other).LengthSquared();
    }

    // Utility functions
    static Vector3 Zero() { return Vector3(0, 0, 0); }
    static Vector3 One() { return Vector3(1, 1, 1); }
    static Vector3 Up() { return Vector3(0, 1, 0); }
    static Vector3 Down() { return Vector3(0, -1, 0); }
    static Vector3 Left() { return Vector3(-1, 0, 0); }
    static Vector3 Right() { return Vector3(1, 0, 0); }
    static Vector3 Forward() { return Vector3(0, 0, 1); }
    static Vector3 Back() { return Vector3(0, 0, -1); }

    // Linear interpolation
    static Vector3 Lerp(const Vector3& a, const Vector3& b, float t) {
        return a + (b - a) * t;
    }

    // Component-wise min/max
    static Vector3 Min(const Vector3& a, const Vector3& b) {
        return Vector3(
            std::min(a.x, b.x),
            std::min(a.y, b.y),
            std::min(a.z, b.z)
        );
    }

    static Vector3 Max(const Vector3& a, const Vector3& b) {
        return Vector3(
            std::max(a.x, b.x),
            std::max(a.y, b.y),
            std::max(a.z, b.z)
        );
    }

    // For use as unordered_map key
    struct Hash {
        size_t operator()(const Vector3& v) const {
            // Quantize to voxel grid (5cm precision)
            const float grid_size = 0.05f;
            int ix = static_cast<int>(std::round(v.x / grid_size));
            int iy = static_cast<int>(std::round(v.y / grid_size));
            int iz = static_cast<int>(std::round(v.z / grid_size));

            // Hash combine
            size_t h1 = std::hash<int>{}(ix);
            size_t h2 = std::hash<int>{}(iy);
            size_t h3 = std::hash<int>{}(iz);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    // String representation
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

// Global operator for scalar * vector
inline Vector3 operator*(float scalar, const Vector3& vec) {
    return vec * scalar;
}

// Common vector operations as standalone functions
inline float Dot(const Vector3& a, const Vector3& b) {
    return a.Dot(b);
}

inline Vector3 Cross(const Vector3& a, const Vector3& b) {
    return a.Cross(b);
}

inline Vector3 Normalize(const Vector3& v) {
    return v.Normalized();
}

inline float Distance(const Vector3& a, const Vector3& b) {
    return a.Distance(b);
}

inline float DistanceSquared(const Vector3& a, const Vector3& b) {
    return a.DistanceSquared(b);
}
