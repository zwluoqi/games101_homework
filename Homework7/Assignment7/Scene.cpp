//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f Scene::shade(const Vector3f& p, const  Vector3f& wo,  Material* m, const  Vector3f& N,int depth) const {
    Intersection inter;
    float pdf_light;
    //cibtruvytuib firn the light source,
    sampleLight(inter, pdf_light);
    auto x = inter.coords;
    auto NN = inter.normal;
    auto emit = inter.emit;
    auto dir = (x - p);
    auto ws = dir.normalized();
    auto distance = dir.norm();
    auto dirSqrtDistance = distance * distance;
    Vector3f L_dir = 0.0;
    auto refInt = bvh->Intersect(Ray(p, ws, distance));

    if (!refInt.happened) {
        //emit is the light value
        auto NDotI = std::max(0.0f, dotProduct(ws, N));
        auto NNDotI = std::max(0.0f, dotProduct(-ws,NN));
        auto fr = m->eval(wo, ws, N);
        auto k = (dirSqrtDistance) * (pdf_light + 0.000001);
        L_dir = emit* fr * NDotI * NNDotI / k;
    }

    ////contributoin from other reflectors,
    Vector3f L_indir = 0.0;
    auto wi = m->sample(wo, N);
    auto ksi = get_random_float();
    if (ksi < RussianRoulette) {
        float pdf_hemi = m->pdf(wo, wi, N);
        //if (pdf_hemi > 0) 
        {
            auto otherInt = bvh->Intersect(Ray(p, wi));
            if (otherInt.happened && !otherInt.obj->hasEmit()) {
                auto fr = m->eval(wo, wi, N);
                auto NdotI = std::max(0.0f, dotProduct(wi, N));
                auto indir = fr * NdotI / pdf_hemi / RussianRoulette;
                auto shadeVal = shade(otherInt.coords, -wi, otherInt.m, otherInt.normal, depth + 1);
                L_indir = shadeVal * indir;
            }
        }
    }


    return L_dir + L_indir;
    //auto ws = inter.uv;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > this->maxDepth) {
        return Vector3f(0.0, 0.0, 0.0);
    }
    Intersection intersection = Scene::intersect(ray);
    Material* m = intersection.m;
    Object* hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;

    if (intersection.happened) {
        if (m->hasEmission())
        {
            return m->getEmission();
        }
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        hitColor = shade(hitPoint, -ray.direction, m, N, 1);
       
    }

    return hitColor;
}