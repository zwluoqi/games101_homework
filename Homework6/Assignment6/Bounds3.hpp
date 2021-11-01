//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>
#define INF 11451419.0f

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline float hitSquare(const Ray& ray,const Vector3f& p1,const Vector3f& p2,const Vector3f& p3,const Vector3f& p4) const;

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};


//check ray hit rect

inline float Bounds3::hitSquare(const Ray& ray,const Vector3f& p1,const Vector3f& p2,const Vector3f& p3,const Vector3f& p4) const {
    Vector3f S = ray.origin;        //ray ori
    Vector3f d = ray.direction;         //ray dir
    Vector3f N = normalize(crossProduct(p2 - p1, p3 - p1));
    if (dotProduct(N, d) > 0.0f) N = -N;   //get normal

    //check ray dir pingxing rect
    if (fabs(dotProduct(N, d)) < 0.00001f) return INF;

    //distance
    float t = (dotProduct(N, p1) - dotProduct(S, N)) / dotProduct(d, N);
    if (t < 0.0005f) return INF;    //check ray in rect back

    //get cross poing
    Vector3f P = S + d * t;

    //check point in rect
    float minx = std::min(std::min(p1.x, p2.x), std::min(p3.x, p4.x)) - 0.0005f;
    float miny = std::min(std::min(p1.y, p2.y), std::min(p3.y, p4.y)) - 0.0005f;
    float minz = std::min(std::min(p1.z, p2.z), std::min(p3.z, p4.z)) - 0.0005f;

    float maxx = std::max(std::max(p1.x, p2.x), std::max(p3.x, p4.x)) + 0.0005f;
    float maxy = std::max(std::max(p1.y, p2.y), std::max(p3.y, p4.y)) + 0.0005f;
    float maxz = std::max(std::max(p1.z, p2.z), std::max(p3.z, p4.z)) + 0.0005f;
    if (minx <= P.x && P.x <= maxx && miny <= P.y && P.y <= maxy && minz <= P.z && P.z <= maxz) {
        return t;
    }
    return INF;
}

inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    auto dia = Diagonal();
    auto i1 = pMin + Vector3f(dia.x,0,0);
    auto i2 = pMin + Vector3f(0,dia.y,0);
    auto i3 = pMin + Vector3f(0,0,dia.z);

    auto a1 = pMax - Vector3f(dia.x,0,0);
    auto a2 = pMax - Vector3f(0,dia.y,0);
    auto a3 = pMax - Vector3f(0,0,dia.z);

    float hit_distance = INF;
    hit_distance = std::min(hit_distance,this->hitSquare(ray,pMin,i1,i2,a3));
    hit_distance = std::min(hit_distance,this->hitSquare(ray,pMin,i1,a2,i3));
    hit_distance = std::min(hit_distance,this->hitSquare(ray,pMin,a1,i2,i3));

    hit_distance = std::min(hit_distance,this->hitSquare(ray,pMax,a1,a2,i3));
    hit_distance = std::min(hit_distance,this->hitSquare(ray,pMax,a1,i2,a3));
    hit_distance = std::min(hit_distance,this->hitSquare(ray,pMax,i1,a2,a3));
    
    return hit_distance<INF;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
