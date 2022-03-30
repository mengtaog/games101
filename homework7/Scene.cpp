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

// Implementation of Path Tracing

Vector3f Scene::castRay(const Ray &ray, int depth) const 
{
    //std::cout<<"CALL castRay"<<std::endl;
    // TO DO Implement Path Tracing Algorithm here
    Vector3f color(0.0f, 0.0f, 0.0f);
    Intersection intersection = intersect(ray);
    if (false == intersection.happened) return color;
    else if (intersection.m->hasEmission()) return intersection.m->m_emission;
    Vector3f wo = ray.direction;
    color = Shade(intersection, wo);
    return color;
    
}

Vector3f Scene::Shade(const Intersection& intersection, const Vector3f& wo) const
{
    //std::cout<<"ENTER"<<std::endl;
    Vector3f p = intersection.coords;//obj pos
    Vector3f N = intersection.normal;//obj normal
    Vector3f color(0.0f, 0.0f, 0.0f);
    float pdf;
    Intersection inter;
    sampleLight(inter, pdf);
    Vector3f x = inter.coords;//light pos
    float xpDis = (x-p).norm();
    Vector3f ws = (x - p).normalized();//light dir
    Vector3f NN = inter.normal;//light normal
    Vector3f emit = inter.emit;
    Ray px(p, ws); 
    Intersection shelter = intersect(px);
    
    Vector3f L_dir(0.0f, 0.0f, 0.0f);
    //if (*(shelter.obj) == *(inter.obj))
    //if (shelter.happened) std::cout<<(shelter.coords - inter.coords).norm()<<std::endl;
    //if (shelter.happened && (shelter.coords - inter.coords).norm() < 0.01f)
    if (shelter.distance - xpDis >= -0.005f)
    {
        //std::cout<<"emit: "<<emit<<", f_r: "<<intersection.m->eval(wo, ws, N)<<std::endl;
        L_dir = emit * intersection.m->eval(wo, ws, N) * dotProduct(ws, N) 
        * dotProduct(-ws, NN) / dotProduct(x-p, x-p) / pdf;
        //std::cout<<"L_dir: "<<L_dir<<std::endl;
    }
    
    Vector3f L_indir(0.0f, 0.0f, 0.0f);
    float rando = (float)rand()/RAND_MAX;
    //std::cout<<rando<<std::endl;
    if (rando< RussianRoulette)
    {
        Vector3f wi = intersection.m->sample(wo, N).normalized();
        Ray r(p, wi);
        Intersection hit = intersect(r);
        //std::cout<<"__1___"<<std::endl;
        if (hit.happened && !hit.obj->hasEmit())
        {
            //std::cout<<"___2__"<<std::endl;
            L_indir = Shade(hit, wi) * intersection.m->eval(wo, wi, N) * dotProduct(wi, N)
            /intersection.m->pdf(wo, wi, N) / RussianRoulette;
            //std::cout<<"L_indir: "<<L_indir<<std::endl;
        }
    }
    //std::cout<<"RETURN"<<std::endl;
    //std::cout<<"L_dir + L_indir: "<<L_dir+L_indir<<std::endl;
    return L_dir + L_indir;
}