#include <log.h>
#include <core.h>
#include <efloat.h>
#include <random.h>
#include <vec.h>
#include <yuki_memory.h>
#include <geometry.h>
#include <intersection.h>
#include <iostream>
#include <vector>
#include <memory>
#include <primitive.h>
using namespace std;
using namespace Yuki;


vector<shared_ptr<Primitive> > prims;
Float params[][11] = {
    {600, 50,681.6-.27,81.6, 12,12,12, 0,0,0,0},
    {1e5, 1e5+1 ,40.8,81.6, 0,0,0,.75, .25, .25,0},
    {1e5,-1e5+99,40.8,81.6, 0,0,0,.25, .25, .75,0},
    {1e5, 50,    40.8, 1e5, 0,0,0,.25, .75, .25,0},
    {1e5, 50,40.8,-1e5+170, 0,0,0,  0,   0,   0,0},
    {1e5, 50,1e5,81.6,      0,0,0,.75, .75, .75,0},
    {1e5, 50,-1e5+81.6,81.6,0,0,0,.75, .75, .75,0},
    {16.5,27,16.5,47,       0,0,0,.999,.999,.999,1},
    {16.5,73,16.5,78,       0,0,0,.999,.999,.999,2}
};
inline int toInt(double x){ return int(pow(clamp(x, 0, 1),1/2.2)*255+.5); }

vec3<Float> radiance(const Ray &r, int depth, Random &rand) {
    Intersection insect;
    Vector position;
    double t;
    bool intersect_ = false;
    for (uint32_t i = 0; i < prims.size(); ++i) {
        if (prims[i]->intersect_p(r) && prims[i]->intersect(r, &insect)) {
            position.x = params[i][1];
            position.y = params[i][2];
            position.z = params[i][3];
            intersect_ = true;
        }
    }
    if (!intersect_) return vec3<Float>();
    t = r.max_t;
    Point x = r(t);
    Vector n = (x - position).normalized(),
        nl = (n * r.d < 0) ? n : -n;
    vec3<Float> f = insect.color;
    double p = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z; // max refl
    if (++depth>5) {
        if (rand.random() < p) 
            f=f*(1/p);
        else return insect.emission;
    } //R.R.
    if (depth > 7) return insect.emission;
    if (insect.refl_t == 0) {// diff
        double r1 = 2 * M_PI * rand.random(),
            r2 = rand.random(),
            r2s = sqrt(r2);
        Vector w = nl, u = ((fabs(w.x) > .1? Vector(0,1,0):Vector(1, 0, 0)) ^ w).normalized(),
            v = w ^ u;
        Vector d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        return insect.emission + (f & radiance(insect.spawn_ray(d), depth, rand));
    }
    else if (insect.refl_t == 1) {// spec
        return insect.emission + (f & radiance(insect.spawn_ray(r.d - n * 2 * (n * r.d)), depth, rand));
    }
    // reflect
    Ray refl_ray(x, r.d - n * 2 * (n * r.d));
    bool into = (n * nl) > 0;
    double nc = 1, nt = 1.5, nnt = into?nc/nt:nt/nc, ddn=r.d * nl, cos2t;
    if ((cos2t=1-nnt*nnt*(1-ddn*ddn))<0)    // Total internal reflection 
        return insect.emission + (f & radiance(refl_ray,depth,rand)); 
    vec3<Float> tdir = (r.d*nnt - n*((into?1:-1)*(ddn*nnt+sqrt(cos2t)))).normalized(); 
    double a=nt-nc, b=nt+nc, R0=a*a/(b*b), c = 1-(into?-ddn:(tdir * n)); 
    double Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re,P=.25+.5*Re,RP=Re/P,TP=Tr/(1-P);
    Float xx = rand.random();
    return insect.emission + (f & (depth>2 ? (xx<P ?   // Russian roulette 
        radiance(refl_ray,depth,rand)*RP:radiance(insect.spawn_ray(tdir),depth,rand)*TP) : 
        radiance(refl_ray,depth,rand)*Re+radiance(insect.spawn_ray(tdir),depth,rand)*Tr)); 
}

int main() {

    EFloat temp;
    for (int i = 0; i < 9; ++i) {
        prims.push_back(make_shape("sphere", params[i]));
    }

    int w = 320, h = 240, samps = 1024;
    Ray cam(Point(50, 52, 295.6), Vector(0, -0.042612, -1).normalized());
    Vector cx(w * .5135/h, 0, 0);
    Vector cy = (cx ^ cam.d).normalized() * .5135;
    vec3<Float> r;
    vec3<Float> *c = new vec3<Float>[w * h];
#pragma omp parallel for schedule(dynamic, 1) private(r)       // OpenMP
    for (int y = 0; y < h; y++) {
        fprintf(stderr,"\rRendering (%d spp) %5.2f%%",samps*4,100.*y/(h-1));
        for (unsigned short x = 0; x < w; x++) {
            Random rand;
            for (int sy = 0, i = (h - y - 1) * w + x; sy < 2; sy++) {
                for (int sx = 0; sx < 2; sx++, r = vec3<Float>()) {
                    for (int s = 0; s < samps; s++) {
                        double r1 = 2 * rand.random(), dx = r1<1 ? sqrt(r1)-1: 1-sqrt(2-r1);
                        double r2 = 2 * rand.random(), dy = r2<1 ? sqrt(r2)-1: 1-sqrt(2-r2);
                        Vector d = cx*( ( (sx+.5 + dx)/2 + x)/w - .5) + 
                            cy*( ( (sy+.5 + dy)/2 + y)/h - .5) + cam.d;
                        Point p = cam.o + d * 140;
                        d.normalize();
                        vec3<Float> res = radiance(Ray(Point(p.x, p.y, p.z), Vector(d.x, d.y, d.z)), 0, rand) * (1. / samps);
                        r = r + vec3<Float>(res.x, res.y, res.z);
                    }
                    c[i] = c[i] + vec3<Float>(clamp(r.x, 0, 1), clamp(r.y, 0, 1), clamp(r.z, 0, 1)) * .25;
                }
            }
        }
    }

    FILE *f;
    fopen_s(&f, "image.ppm", "w");         // Write image to PPM file. 
    fprintf(f, "P3\n%d %d\n%d\n", w, h, 255); 
    for (int i=0; i<w*h; i++) 
        fprintf(f,"%d %d %d ", toInt(c[i].x), toInt(c[i].y), toInt(c[i].z));    


    system("pause");
    return 0;
}
