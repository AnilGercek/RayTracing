#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "Vect.h"

#include <ctime>
#include <iomanip>
typedef unsigned char RGB[3];
unsigned char *rayTracing(const parser::Scene&, parser::Camera);
Vect calculateCoorPixel(int, int, float, float, Vect, Vect, Vect);
float determinant(Vect, Vect, Vect);
float rayTriangleIntersection(Vect,Vect,Vect,Vect,Vect);
parser::Vec3i color_clipping(int, int, int);
float raySphereIntersection(Vect,Vect,Vect,float);
Vect computeColor(Vect,parser::PointLight,Vect,parser::Material,Vect);
Vect ray_tracing(parser::Scene,Vect,Vect,int,Vect);
int main(int argc, char *argv[])
{
    clock_t begin = clock();
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.

    int camera_count = scene.cameras.size();

    for (int i = 0; i < camera_count; i++)
    {
        int width = scene.cameras[i].image_width;
        int height = scene.cameras[i].image_height;

        parser::Camera camera = scene.cameras[i];
        unsigned char *im = rayTracing(scene, camera);

        std::string output_name = scene.cameras[i].image_name;
        const char *out = const_cast<char *>(output_name.c_str());

        write_ppm(out, im, width, height);
    }
    clock_t end = clock();
    double elapse = double(end-begin)/CLOCKS_PER_SEC;
    std::cout << "\nElapsed Time: "<<elapse << std::endl;
}

unsigned char *rayTracing(const parser::Scene &scene, parser::Camera camera)
{
    unsigned char *image = new unsigned char[camera.image_width * camera.image_height * 3];
    Vect e(camera.position.x, camera.position.y, camera.position.z);
    Vect gaze(camera.gaze.x, camera.gaze.y, camera.gaze.z);
    Vect v(camera.up.x, camera.up.y, camera.up.z);
    float left = camera.near_plane.x;
    float right = camera.near_plane.y;
    float bottom = camera.near_plane.z;
    float top = camera.near_plane.w;
    Vect u = v.crossProduct(gaze.vectorNegative());
    gaze = gaze.vectorMult(camera.near_distance);
    Vect m = gaze.vectorAdd(e);
    Vect temp = m.vectorAdd(u.vectorMult(left));
    Vect q = temp.vectorAdd(v.vectorMult(top));
    float rl = right - left; // r-l
    float tb = top - bottom; // t-b
    int nx = camera.image_width;
    int ny = camera.image_height;
    float rlnx = rl / nx; // (r-l)/nx
    float tbny = tb / ny; // (t-b)/ny
    for (int j = 0; j < ny; j++)
        {
            for (int i = 0; i < nx; i++)
            {
                Vect colors(0,0,0);
                Vect s = calculateCoorPixel(i, j, rlnx, tbny, u, v, q);
                Vect d = s.vectorSubtract(e);
                // do a new function d ve e gönderelim, scene, d 
                colors = ray_tracing(scene,e,d,1,Vect(0,0,0));
                
                //
                parser::Vec3i clippedColors = color_clipping((int)colors.x, (int)colors.y, (int)colors.z);
                image[(j * nx + i) * 3] = clippedColors.x;
                image[(j * nx + i) * 3 + 1] = clippedColors.y;
                image[(j * nx + i) * 3 + 2] = clippedColors.z;
            }
            std::cout << "\r" << "row: " << std::setw(3) << j+1 << "/" << ny << std::flush;
        }
    return image;
}
Vect ray_tracing(parser::Scene scene,Vect e, Vect d, int recursionCount,Vect recursionColor){
    /////////////////
    float t_sphere, t_triangle, t_mesh, t_min_sphere, t_min_triangle, t_min_mesh;
    Vect colors(0,0,0);
    int sphere_count = scene.spheres.size();
    int triangle_count = scene.triangles.size();
    int mesh_count = scene.meshes.size();

    bool isMirror = true;
    parser::Material hitMaterial;
    Vect hitNormal;

    const std::vector<parser::Vec3f> *vertex_data = &scene.vertex_data;
    const std::vector<parser::Sphere> *spheres = &scene.spheres;
    const std::vector<parser::Triangle> *triangles = &scene.triangles;
    const std::vector<parser::Mesh> *meshes = &scene.meshes;
    std::vector<parser::Face> faces;
    const std::vector<parser::PointLight> *pointLights = &scene.point_lights;
    const std::vector<parser::Vec3f> *vertices = &scene.vertex_data;
    const std::vector<parser::Material> *materials = &scene.materials;
    int light_count = pointLights->size();

    parser::Vec3f amb_intensity = scene.ambient_light;
    Vect ambient_intensity(amb_intensity.x, amb_intensity.y, amb_intensity.z);
    Vect normal_sphere,normal_triangle,normal_mesh;
    parser::Material material_sphere,material_triangle,material_mesh;
    Vect hitPosition;
    int hitIdMesh,hitIdSphere,hitIdTriangle,hitIdFace;
    /////////////////
    t_min_sphere = INFINITY;
    t_min_triangle = INFINITY;
    t_min_mesh = INFINITY;
    //
    for (int index = 0; index < sphere_count; index++){
        Vect center(vertices->at(spheres->at(index).center_vertex_id - 1).x,vertices->at(spheres->at(index).center_vertex_id - 1).y,vertices->at(spheres->at(index).center_vertex_id - 1).z);
        t_sphere = raySphereIntersection(d,e, center, spheres->at(index).radius);
        if (t_sphere < t_min_sphere)
        {
            t_min_sphere = t_sphere;
            hitPosition = e.vectorAdd(d.vectorMult(t_sphere));
            normal_sphere = hitPosition.vectorSubtract(center).vectorMult(1.f/spheres->at(index).radius);
            material_sphere = materials->at(spheres->at(index).material_id-1);
            hitIdSphere = index;
        }
    }

    // triangle intersection
    for (int index = 0; index < triangle_count; index++){
        Vect point_A(vertex_data->at(triangles->at(index).indices.v0_id - 1).x, vertex_data->at(triangles->at(index).indices.v0_id - 1).y, vertex_data->at(triangles->at(index).indices.v0_id - 1).z);
        Vect point_B(vertex_data->at(triangles->at(index).indices.v1_id - 1).x, vertex_data->at(triangles->at(index).indices.v1_id - 1).y, vertex_data->at(triangles->at(index).indices.v1_id - 1).z);
        Vect point_C(vertex_data->at(triangles->at(index).indices.v2_id - 1).x, vertex_data->at(triangles->at(index).indices.v2_id - 1).y, vertex_data->at(triangles->at(index).indices.v2_id - 1).z);
        Vect ba = point_B.vectorSubtract(point_A);
        Vect ca = point_C.vectorSubtract(point_A);
        t_triangle = rayTriangleIntersection(point_A, point_B, point_C, e, d);
        if (t_triangle < t_min_triangle)
        {
            t_min_triangle = t_triangle;
            normal_triangle = ba.crossProduct(ca).vectorMult(1/ba.crossProduct(ca).magnitude());
            material_triangle = materials->at(triangles->at(index).material_id-1);
            hitIdTriangle = index;
        }
    }

    // mesh intersection
    for (int index = 0; index < mesh_count; index++){
                faces = meshes->at(index).faces;
                int face_count = faces.size();

                for (int face = 0; face < face_count; face++)
                {
                    Vect point_A(vertex_data->at(faces.at(face).v0_id - 1).x, vertex_data->at(faces.at(face).v0_id - 1).y, vertex_data->at(faces.at(face).v0_id - 1).z);
                    Vect point_B(vertex_data->at(faces.at(face).v1_id - 1).x, vertex_data->at(faces.at(face).v1_id - 1).y, vertex_data->at(faces.at(face).v1_id - 1).z);
                    Vect point_C(vertex_data->at(faces.at(face).v2_id - 1).x, vertex_data->at(faces.at(face).v2_id - 1).y, vertex_data->at(faces.at(face).v2_id - 1).z);
                    Vect ba = point_B.vectorSubtract(point_A);
                    Vect ca = point_C.vectorSubtract(point_A);

                    t_mesh = rayTriangleIntersection(point_A, point_B, point_C, e, d);

                    if (t_mesh < t_min_mesh)
                    {
                        t_min_mesh = t_mesh;
                        normal_mesh = ba.crossProduct(ca).vectorMult(1/ba.crossProduct(ca).magnitude());
                        material_mesh = materials->at(meshes->at(index).material_id-1);
                        hitIdMesh = index;
                        hitIdFace = face;
                    }
                }
    }
    float t_min = std::min(t_min_mesh, std::min(t_min_sphere, t_min_triangle));
    
    if (t_min == INFINITY ){
        colors.x = 0;
        colors.y = 0;
        colors.z = 0;
    }
    else {
        //do the shadow thing. if no shadow, compute color
        hitPosition = e.vectorAdd(d.vectorMult(t_min));
        if(t_min == t_min_mesh){
            colors = colors.vectorAdd(Vect(ambient_intensity.x * material_mesh.ambient.x,ambient_intensity.y * material_mesh.ambient.y,ambient_intensity.z * material_mesh.ambient.z));    
            for(int l=0;l<light_count;l++){
                float t;
                bool shadow=false;
                Vect lightPosition(pointLights->at(l).position.x,pointLights->at(l).position.y,pointLights->at(l).position.z);
                Vect shadow_d = lightPosition.vectorSubtract(hitPosition).normalize();
                for (int index = 0; index < sphere_count; index++){
                    Vect center(vertices->at(spheres->at(index).center_vertex_id - 1).x,vertices->at(spheres->at(index).center_vertex_id - 1).y,vertices->at(spheres->at(index).center_vertex_id - 1).z);
                    t = raySphereIntersection(shadow_d,hitPosition.vectorAdd(normal_mesh.vectorMult(scene.shadow_ray_epsilon)), center, spheres->at(index).radius);
                    if (t!=INFINITY && (shadow_d.vectorMult(t).magnitude() < hitPosition.distance(lightPosition)))
                    {
                        shadow = true;
                        break;
                    }
                }
                if(shadow==true)continue;
                for (int index = 0; index < mesh_count; index++){
                    faces = meshes->at(index).faces;
                    int face_count = faces.size();

                    for (int face = 0; face < face_count; face++){
                        if(index == hitIdMesh && face == hitIdFace)continue;
                                    
                        Vect point_A(vertex_data->at(faces.at(face).v0_id - 1).x, vertex_data->at(faces.at(face).v0_id - 1).y, vertex_data->at(faces.at(face).v0_id - 1).z);
                        Vect point_B(vertex_data->at(faces.at(face).v1_id - 1).x, vertex_data->at(faces.at(face).v1_id - 1).y, vertex_data->at(faces.at(face).v1_id - 1).z);
                        Vect point_C(vertex_data->at(faces.at(face).v2_id - 1).x, vertex_data->at(faces.at(face).v2_id - 1).y, vertex_data->at(faces.at(face).v2_id - 1).z);
                        Vect ba = point_B.vectorSubtract(point_A);
                        Vect ca = point_C.vectorSubtract(point_A);

                        t = rayTriangleIntersection(point_A, point_B, point_C, hitPosition.vectorAdd(normal_mesh.vectorMult(scene.shadow_ray_epsilon)), shadow_d);

                        if (t!=INFINITY && (shadow_d.vectorMult(t).magnitude() < hitPosition.distance(lightPosition))){
                            shadow = true;
                            break;
                        }
                    }
                    if(shadow==true){
                        break;
                    }
                }
                if(shadow==true)continue;
                colors = colors.vectorAdd(computeColor(hitPosition,pointLights->at(l),normal_mesh,material_mesh,e));
                hitMaterial = material_mesh;
                hitNormal = normal_mesh;
            }
        }
        else if(t_min == t_min_sphere){
            colors = colors.vectorAdd(Vect(ambient_intensity.x * material_sphere.ambient.x,ambient_intensity.y * material_sphere.ambient.y,ambient_intensity.z * material_sphere.ambient.z));
            for(int l=0;l<light_count;l++){
                // SHADOW
                Vect lightPosition(pointLights->at(l).position.x,pointLights->at(l).position.y,pointLights->at(l).position.z);
                Vect shadow_d = lightPosition.vectorSubtract(hitPosition).normalize();
                float t;
                bool shadow=false;
                for (int index = 0; index < sphere_count; index++){
                    if(index == hitIdSphere)continue;
                    Vect center(vertices->at(spheres->at(index).center_vertex_id - 1).x,vertices->at(spheres->at(index).center_vertex_id - 1).y,vertices->at(spheres->at(index).center_vertex_id - 1).z);
                    t = raySphereIntersection(shadow_d,hitPosition.vectorAdd(normal_sphere.vectorMult(scene.shadow_ray_epsilon)), center, spheres->at(index).radius);
                    if (t!=INFINITY && (shadow_d.vectorMult(t).magnitude() < hitPosition.distance(lightPosition)))
                    {
                        shadow = true;
                        break;
                    }
                }
                if(shadow==true)continue;
                for (int index = 0; index < mesh_count; index++){
                    faces = meshes->at(index).faces;
                    int face_count = faces.size();

                    for (int face = 0; face < face_count; face++){
                        Vect point_A(vertex_data->at(faces.at(face).v0_id - 1).x, vertex_data->at(faces.at(face).v0_id - 1).y, vertex_data->at(faces.at(face).v0_id - 1).z);
                        Vect point_B(vertex_data->at(faces.at(face).v1_id - 1).x, vertex_data->at(faces.at(face).v1_id - 1).y, vertex_data->at(faces.at(face).v1_id - 1).z);
                        Vect point_C(vertex_data->at(faces.at(face).v2_id - 1).x, vertex_data->at(faces.at(face).v2_id - 1).y, vertex_data->at(faces.at(face).v2_id - 1).z);
                        Vect ba = point_B.vectorSubtract(point_A);
                        Vect ca = point_C.vectorSubtract(point_A);
                        t = rayTriangleIntersection(point_A, point_B, point_C, hitPosition.vectorAdd(normal_sphere.vectorMult(scene.shadow_ray_epsilon)), shadow_d);

                        if (t!=INFINITY && (shadow_d.vectorMult(t).magnitude() < hitPosition.distance(lightPosition))){
                            shadow =true;
                            break;
                        }
                    }
                    if(shadow==true)break;
                }
                            
                if(shadow==true)continue;
                colors = colors.vectorAdd(computeColor(hitPosition,pointLights->at(l),normal_sphere,material_sphere,e));
                hitNormal = normal_sphere;
                hitMaterial = material_sphere;
            }
        } 
        else{
            colors = colors.vectorAdd(Vect(ambient_intensity.x * material_triangle.ambient.x,ambient_intensity.y * material_triangle.ambient.y,ambient_intensity.z * material_triangle.ambient.z));
            for(int l=0;l<light_count;l++){
                colors = colors.vectorAdd(computeColor(hitPosition,pointLights->at(l),normal_triangle,material_triangle,e));
            }
            hitNormal = normal_triangle;
            hitMaterial = material_triangle;
        }
    }
    Vect ref_mirror(hitMaterial.mirror.x,hitMaterial.mirror.y,hitMaterial.mirror.z);
    //if(ref_mirror.magnitude() == 0 ){
        return colors;
    //}
    if(recursionCount == 0)return colors;
    colors.x *= ref_mirror.x;
    colors.y *= ref_mirror.y;
    colors.z *= ref_mirror.z;
    Vect _w0 = d.vectorNegative();
    float nw0 = hitNormal.dotProduct(d);
    return ray_tracing(scene,hitPosition,_w0.vectorAdd(hitNormal.vectorMult(nw0*2)),recursionCount-1,recursionColor.vectorAdd(colors));
}
Vect calculateCoorPixel(int i, int j, float rlnx, float tbny, Vect u, Vect v, Vect q){
    float su = rlnx * (i + 0.5); // (r-l)(i+0.5)/nx
    float sv = tbny * (j + 0.5); // (t-b)(j+0.5)/ny //
    Vect s = u.vectorMult(su).vectorAdd(v.vectorMult(-1 * sv)).vectorAdd(q);
    return s;
}

parser::Vec3i color_clipping(int red, int green, int blue){
    parser::Vec3i ret;

    ret.x = red;
    ret.y = green;
    ret.z = blue;
    if (red > 255)
    {
        ret.x = 255;
    }
    if (green > 255)
    {
        ret.y = 255;
    }
    if (blue > 255)
    {
        ret.z = 255;
    }
    return ret;
}

float raySphereIntersection(Vect d,Vect e,Vect center,float radius){
    float dd = d.dotProduct(d);
    Vect oc = e.vectorSubtract(center);
    float disc = pow(d.dotProduct(oc), 2) - dd * (oc.dotProduct(oc) - radius * radius);
    float t1 = (-1 * (d.dotProduct(oc)) + sqrt(disc)) / dd;
    float t2 = (-1 * (d.dotProduct(oc)) - sqrt(disc)) / dd;

    float t = std::min(t1, t2);
    if (disc <= 0)
    {
        return INFINITY;
    }
    else if (t1 >= 0 && t2 >= 0)
    {
        return t;
    }
    else if (t1 < 0 && t2 >= 0)
        return t2;
    else if (t2 < 0 && t1 >= 0)
        return t1;
    else
    {
        return INFINITY;
    }
}
Vect computeColor(Vect p,parser::PointLight light,Vect normal,parser::Material material,Vect cameraPosition){
    // Diffuse Shading
    Vect lightPosition(light.position.x,light.position.y,light.position.z);
    Vect lightIntensity(light.intensity.x,light.intensity.y,light.intensity.z);
    Vect w_i = lightPosition.vectorSubtract(p);

    Vect ret;
    // material coefficients
    parser::Vec3f diffuse_reflectance = material.diffuse;
    parser::Vec3f ambient_reflectance = material.ambient;
    parser::Vec3f specular_reflectance = material.specular;
    float phong_exponent = material.phong_exponent;

    Vect L_d; // diffuse shading
    Vect L_s; // specular shading

    float cos_theta = ((w_i.dotProduct(normal))) / (w_i.magnitude() * normal.magnitude());

    float distance = p.distance(lightPosition); //distance b/w light source and hit point

    Vect incoming_radiance = lightIntensity.vectorMult(1 / pow(distance, 2)); // L_i = I / (r*r)
    float cos_theta_prime = std::max(0.f, cos_theta);

    // diffuse shading rgb values
    L_d.x = diffuse_reflectance.x * cos_theta_prime * incoming_radiance.x;
    L_d.y = diffuse_reflectance.y * cos_theta_prime * incoming_radiance.y;
    L_d.z = diffuse_reflectance.z * cos_theta_prime * incoming_radiance.z;

    // specular shading rgb values
    Vect w_o = cameraPosition.vectorSubtract(p);

    Vect w_o_normalized = w_o.normalize();
    Vect w_i_normalized = w_i.normalize();
    Vect h = (w_i_normalized.vectorAdd(w_o_normalized)).vectorMult(1.f / (w_i_normalized.vectorAdd(w_o_normalized)).magnitude()); // half vector (w_i + w_o) / ( || w_i + w_o || )
    
    float res = pow(normal.dotProduct(h), phong_exponent);
    float specular_cos_theta_prime = std::max(0.f,res );

    L_s.x = specular_reflectance.x * specular_cos_theta_prime * incoming_radiance.x;
    L_s.y = specular_reflectance.y * specular_cos_theta_prime * incoming_radiance.y;
    L_s.z = specular_reflectance.z * specular_cos_theta_prime * incoming_radiance.z;

    // total shading
    float out_red =   L_d.x + L_s.x;
    float out_green = L_d.y + L_s.y;
    float out_blue =  L_d.z + L_s.z;

    ret.x = out_red;
    ret.y = out_green;
    ret.z = out_blue;

    return ret;
}

float determinant(Vect v1, Vect v2, Vect v3){
    float a = v1.x;
    float b = v1.y;
    float c = v1.z;
    float d = v2.x;
    float e = v2.y;
    float f = v2.z;
    float g = v3.x;
    float h = v3.y;
    float i = v3.z;

    return a * (e * i - h * f) + b * (g * f - d * i) + c * (d * h - e * g);
}

// point A, point B, point C, camera position, w_0, ray, diffuse_reflectance, light_intensity, ambient_intensity, ambient_reflectance
float rayTriangleIntersection(Vect point_A, Vect point_B, Vect point_C, Vect o, Vect d){
    float A = determinant(point_A.vectorSubtract(point_B), point_A.vectorSubtract(point_C), d);
    Vect ao = point_A.vectorSubtract(o);
    Vect ac = point_A.vectorSubtract(point_C);
    Vect ab = point_A.vectorSubtract(point_B);
    float beta = determinant(ao, ac, d) / A;
    float gama = determinant(ab, ao, d) / A;

    float t = determinant(ab, ac, ao) / A;

    if (beta >= 0 && gama >= 0 && ((beta + gama) <= 1) && t>0)
    {
        return t;
    }
    return INFINITY;
}