//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray& ray,
    const std::vector<Object*>& objects,
    float& tNear, uint32_t& index, Object** hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // Implement Path Tracing Algorithm here
    auto hitObj = intersect(ray);
    if (!hitObj.happened)
    {
        return Vector3f(0.0f); // Background color
    }
    if (hitObj.m->hasEmission())
    {
        return hitObj.m->getEmission(); // Directly return emission color for light sources
    }
    // 直接光照
    Vector3f L_dir = Vector3f(0.0f);
    Intersection lightPos;
    float pdf = 0.0f;
    sampleLight(lightPos, pdf);
    Vector3f obj2Light = lightPos.coords - hitObj.coords;
    Vector3f obj2LightDir = normalize(obj2Light);
    auto t = intersect(Ray(hitObj.coords, obj2LightDir));
    // 检查是否有遮挡
    if (t.distance - obj2Light.norm() > -0.001)
    {
        Vector3f f_r = hitObj.m->eval(ray.direction, obj2LightDir, hitObj.normal);
        float r2 = dotProduct(obj2Light, obj2Light);
        float cosTheta = std::max(0.0f, dotProduct(hitObj.normal, obj2LightDir));
        float cosThetaLight = std::max(0.0f, dotProduct(lightPos.normal, -obj2LightDir));
        L_dir = lightPos.emit * f_r * cosTheta * cosThetaLight / r2 / pdf;
    }
    // 间接光照
    Vector3f L_indir = Vector3f(0.0f);
    if (get_random_float() < RussianRoulette)
    {
        Vector3f dir2NextObj = hitObj.m->sample(ray.direction, hitObj.normal).normalized();
        float pdf = hitObj.m->pdf(ray.direction, dir2NextObj, hitObj.normal);
        if (pdf > EPSILON)
        {
            Intersection nextObj = intersect(Ray(hitObj.coords, dir2NextObj));
            if (nextObj.happened && !nextObj.m->hasEmission())
            {
                Vector3f f_r = hitObj.m->eval(ray.direction, dir2NextObj, hitObj.normal);
                L_indir = castRay(Ray(hitObj.coords, dir2NextObj), depth + 1) * f_r * dotProduct(
                    dir2NextObj, hitObj.normal) / pdf / RussianRoulette;
            }
        }
    }
    return L_dir + L_indir;
}