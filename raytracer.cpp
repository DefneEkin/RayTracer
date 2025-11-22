#include <iostream>
#include "parser.h"
#include "ppm.h"

#define MAXX 2147483640

typedef unsigned char RGB[3];

using namespace std;
using namespace parser;

struct Ray
{
    Vec3f o, d;
    int depth;
    bool is_shadow_ray = false;

    Vec3f find_point(const float t) {
        return o + d * t;
    }
};

struct Record 
{
    int type; //0:sphere 1:triangle 2:mesh
    Sphere sphere;
    Triangle triangle;
    Mesh mesh;
    Face face;
    float distance;
    Vec3f intersect_point;
    Material mat;
    int index;
    int face_index;
    Vec3f n;
};

Vec3i clamp(Vec3i colors, int max=255) {
    return {
        min(max, colors.x), 
        min(max, colors.y), 
        min(max, colors.z) 
    };
}

Vec3f compute_color(Ray & r, const Scene &scene, const vector<Vec3f>& triangle_normals, const vector<vector<Vec3f>> & face_normals);

bool sphere_intersection(const Ray & r, const Sphere & sphere, const vector<Vec3f>& vertex_data, Record &record) {
    Vec3f c = vertex_data[sphere.center_vertex_id - 1];
    float discriminant = std::pow(r.d * (r.o - c), 2) - (r.d * r.d)*((r.o - c)*(r.o - c) - sphere.radius*sphere.radius);
    if (discriminant < 0) return false;

    float term1 = (-1)*r.d*(r.o - c);
    float term2 = std::sqrt(discriminant);
    float term3 = r.d * r.d;

    float t1 = (term1 + term2) / term3;
    float t2 = (term1 - term2) / term3;

    Vec3f point1 = r.o + t1 * r.d;
    Vec3f point2 = r.o + t2 * r.d;

    if (t1 < 0 && t2 < 0) return false;
    if (t1 < 0 || (t2 > 0 && t2 < t1)) {
        record.distance = magnitude(point2 - r.o);
        record.intersect_point = point2;
    }
    else {
        record.distance = magnitude(point1 - r.o);
        record.intersect_point = point1;
    }

    record.sphere = sphere;
     
    return true;
}

float determinant(const float matrix[3][3]) {
    return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
           matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
           matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

bool cramers(const float matrix[3][3], const float vec[3], float soln[3]) {
    float det = determinant(matrix);
    if (det == 0) return false;

    //replace columns and calculate determinants
    float matrix_x[3][3] = {
        {vec[0], matrix[0][1], matrix[0][2]},
        {vec[1], matrix[1][1], matrix[1][2]},
        {vec[2], matrix[2][1], matrix[2][2]}
    };
    soln[0] = determinant(matrix_x) / det;

    float matrix_y[3][3] = {
        {matrix[0][0], vec[0], matrix[0][2]},
        {matrix[1][0], vec[1], matrix[1][2]},
        {matrix[2][0], vec[2], matrix[2][2]}
    };
    soln[1] = determinant(matrix_y) / det;

    float matrix_z[3][3] = {
        {matrix[0][0], matrix[0][1], vec[0]},
        {matrix[1][0], matrix[1][1], vec[1]},
        {matrix[2][0], matrix[2][1], vec[2]}
    };
    soln[2] = determinant(matrix_z) / det;

    return true;
}

bool triangle_intersection(const Ray & r, const Vec3f indices[3], Record &record) {
    Vec3f a = indices[0];
    Vec3f b = indices[1];
    Vec3f c = indices[2];

    float matrix[3][3] = {
        {a.x - b.x, a.x - c.x, r.d.x},
        {a.y - b.y, a.y - c.y, r.d.y},
        {a.z - b.z, a.z - c.z, r.d.z}
    };

    float vec[3] = {a.x - r.o.x, a.y - r.o.y, a.z - r.o.z};

    float soln[3];
    if (cramers(matrix, vec, soln) == false) return false;

    //soln[0]: beta, soln[1]: gamma, soln[2]: t 

    const float epsilon = 0.00001f;

    if ((soln[2] > 0) && (soln[0] + soln[1] <= 1) && (soln[0] >= 0) && (soln[1] >= -epsilon)) {
            record.intersect_point = r.o + r.d * soln[2]; 
            record.distance = magnitude(r.o - record.intersect_point);
            return true;
    }

    return false;
}

bool closest_hit(const Ray & r, Record & hitRecord, const Scene & scene, 
              const vector<Vec3f>& triangle_normals = {}, const vector<vector<Vec3f>>& face_normals = {{}}) {
    bool intersected = false;
    float t_min = MAXX;
    Record record_min;

    for (int i=0; i < scene.spheres.size(); i++) {
        Record tmp_record;
        bool soln_exists = sphere_intersection(r, scene.spheres[i], scene.vertex_data, tmp_record);
        if (soln_exists && tmp_record.distance > scene.shadow_ray_epsilon) {
            intersected = true;
            if (tmp_record.distance < t_min) {
                record_min = tmp_record;
                t_min = tmp_record.distance;
                record_min.type = 0;
            }
        }     
    }

    for (int i=0; i < scene.triangles.size(); i++) {
        Record tmp_record;
        Triangle triangle = scene.triangles[i];
        Vec3f indices[3] = {
            scene.vertex_data[triangle.indices.v0_id - 1],
            scene.vertex_data[triangle.indices.v1_id - 1],
            scene.vertex_data[triangle.indices.v2_id - 1]
        };
        if (r.is_shadow_ray == false) {
            if (r.d * triangle_normals[i] > 0) continue;
        }
        bool soln_exists = triangle_intersection(r, indices, tmp_record);
        if (soln_exists && tmp_record.distance > scene.shadow_ray_epsilon) {
            tmp_record.triangle = triangle;
            tmp_record.index = i; //ith triangle
            intersected = true;
            if (tmp_record.distance < t_min) {
                record_min = tmp_record;
                t_min = tmp_record.distance;
                record_min.type = 1;
            }
        }
    }

    for (int i=0; i < scene.meshes.size(); i++) {
        Mesh mesh = scene.meshes[i];
        for (int j=0; j < mesh.faces.size(); j++) {
            Face face = mesh.faces[j];
            Record tmp_record;
            Vec3f indices[3] = {
                scene.vertex_data[face.v0_id - 1],
                scene.vertex_data[face.v1_id - 1],
                scene.vertex_data[face.v2_id - 1]
            };
            if (r.is_shadow_ray == false) {
                if (r.d * face_normals[i][j] > 0) continue;
            }
            bool soln_exists = triangle_intersection(r, indices, tmp_record);
            if (soln_exists && tmp_record.distance > scene.shadow_ray_epsilon) {
                tmp_record.mesh = mesh;
                tmp_record.face = face;
                tmp_record.index = i; //ith mesh
                tmp_record.face_index = j; //jth face of that mesh
                intersected = true;
                if (tmp_record.distance < t_min) {
                    record_min = tmp_record;
                    t_min = tmp_record.distance;
                    record_min.type = 2;
                }
            }
        }
    }

    hitRecord = record_min;

    return intersected;
}

Material find_material(Record & hitRecord, const vector<Material> & materials) {
    switch (hitRecord.type)
    {
    case 0:
        hitRecord.mat = materials[hitRecord.sphere.material_id - 1];
        break;
    case 1:
        hitRecord.mat = materials[hitRecord.triangle.material_id - 1];
        break;
    case 2:
        hitRecord.mat = materials[hitRecord.mesh.material_id - 1];
    }

    return hitRecord.mat;
}


bool in_shadow(const Record & hitRecord, const PointLight & light, const Ray & shadow_ray, const Scene & scene, float dist) {

    Record record;
    bool object_found = closest_hit(shadow_ray, record, scene);
    if (object_found == false) return false; //no object between point and light source
    if (record.distance < dist) return true;
    else return false;
}

void add_normal(Record & hitRecord, const vector<Vec3f>& triangle_normals, const vector<vector<Vec3f>>& face_normals, const vector<Vec3f> & vertex_data) {
    switch (hitRecord.type)
    {
    case 0: 
        {
        Vec3f center = vertex_data[hitRecord.sphere.center_vertex_id - 1];
        hitRecord.n = (hitRecord.intersect_point - center) / magnitude(hitRecord.intersect_point - center);
        return;
        }
    case 1:
        {
        hitRecord.n = triangle_normals[hitRecord.index];
        return;
        }
    case 2:
        {
        hitRecord.n = face_normals[hitRecord.index][hitRecord.face_index];
        return;
        }
    }

    
}

Vec3f diffuse_term(const Record & hitRecord, const PointLight & light, const Ray & shadow_ray, float dist) {
    float cos_term = max(0.0f, shadow_ray.d * hitRecord.n);
    Vec3f irradience = light.intensity / pow(dist,2);

    return elementwise_mult(hitRecord.mat.diffuse, cos_term * irradience);
}

Vec3f specular_term(const Record & hitRecord, const PointLight & light, const Ray & shadow_ray, const Ray & camera_ray, float dist) {
    Ray wo;
    wo.o = shadow_ray.o;
    wo.d = (-1) * camera_ray.d / magnitude((-1) * camera_ray.d);

    Vec3f h = (shadow_ray.d + wo.d) / magnitude(shadow_ray.d + wo.d);
    float cos_term = max(0.0f, hitRecord.n * h);
    cos_term = pow(cos_term, hitRecord.mat.phong_exponent);
    Vec3f irradience = light.intensity / pow(dist,2);

    return elementwise_mult(hitRecord.mat.specular, cos_term * irradience);
}

void reflect(Ray & reflection_ray, const Ray & camera_ray, Vec3f normal) {
    reflection_ray.d = camera_ray.d + 2 * normal * (normal * ((-1)*camera_ray.d));
    reflection_ray.d = reflection_ray.d / magnitude(reflection_ray.d);
}

Vec3f apply_shading(const Ray & r, Record & hitRecord, const Scene & scene, 
                    const vector<Vec3f>& triangle_normals, const vector<vector<Vec3f>>& face_normals) {

    Material material = find_material(hitRecord, scene.materials);
    add_normal(hitRecord, triangle_normals, face_normals, scene.vertex_data);

    Vec3f color;
    Vec3f ambient_coeff = material.ambient;
    color = {
        ambient_coeff.x * scene.ambient_light.x, 
        ambient_coeff.y * scene.ambient_light.y,
        ambient_coeff.z * scene.ambient_light.z
    };
    
    if (material.is_mirror) {
        Ray reflection_ray;
        reflection_ray.o = hitRecord.intersect_point; //assignment?
        reflect(reflection_ray, r, hitRecord.n);
        reflection_ray.depth = r.depth + 1;
        Vec3f mirror = elementwise_mult(hitRecord.mat.mirror, compute_color(reflection_ray, scene, triangle_normals, face_normals));
        color = color + mirror;
    }

    for (int i=0; i < scene.point_lights.size(); i++) {
        PointLight light = scene.point_lights[i];

        float dist = magnitude(light.position - hitRecord.intersect_point);

        Ray shadow_ray;
        shadow_ray.o = hitRecord.intersect_point + scene.shadow_ray_epsilon * hitRecord.n;
        shadow_ray.d = (light.position - hitRecord.intersect_point) / dist;
        shadow_ray.is_shadow_ray = true;

        if (in_shadow(hitRecord, light, shadow_ray, scene, dist) == false) {
            Vec3f diffuse = diffuse_term(hitRecord, light, shadow_ray, dist);
            Vec3f specular = specular_term(hitRecord, light, shadow_ray, r, dist);
            color = color + diffuse + specular;
        }
    }
    
    return {color.x, color.y, color.z};
}

Vec3f compute_color(Ray & r, const Scene &scene, const vector<Vec3f>& triangle_normals, const vector<vector<Vec3f>> & face_normals) {

    if (r.depth > scene.max_recursion_depth) {

        return {0,0,0};
    }

    Record hitRecord;
    
    if (closest_hit(r, hitRecord, scene, triangle_normals, face_normals)) {
        return apply_shading(r, hitRecord, scene, triangle_normals, face_normals);
    }
    else if (r.depth==0) {
        return {(float)scene.background_color.x, (float)scene.background_color.y, (float)scene.background_color.z};
    }
    else return {0,0,0};
}

void compute_normals(vector<Vec3f>& triangle_normals, vector<vector<Vec3f>>& face_normals, Scene & scene) {

    vector<Triangle> triangles = scene.triangles;
    for (int i=0; i < triangles.size(); i++) {
        Face vertex_ids = triangles[i].indices; //int v0_id, v1_id, v2_id
        Vec3f v0 = scene.vertex_data[vertex_ids.v0_id - 1];
        Vec3f v1 = scene.vertex_data[vertex_ids.v1_id - 1];
        Vec3f v2 = scene.vertex_data[vertex_ids.v2_id - 1];
        Vec3f cross_product_result = cross_product(v1 - v0, v2 - v0);
        Vec3f normal_vec = cross_product_result / magnitude(cross_product_result);
        triangle_normals.push_back(normal_vec);
    }

    vector<Mesh> meshes = scene.meshes;
    for (int i=0; i < meshes.size(); i++) {
        Mesh mesh = meshes[i];

        for (int j=0; j < mesh.faces.size(); j++) {
            Face vertex_ids = mesh.faces[j];
            Vec3f v0 = scene.vertex_data[vertex_ids.v0_id - 1];
            Vec3f v1 = scene.vertex_data[vertex_ids.v1_id - 1];
            Vec3f v2 = scene.vertex_data[vertex_ids.v2_id - 1];
            Vec3f cross_product_result = cross_product(v1 - v0, v2 - v0);
            Vec3f normal_vec = cross_product_result / magnitude(cross_product_result);
            face_normals[i].push_back(normal_vec);
        }

        
    }
}

int main(int argc, char* argv[])
{
    //materials ve vertex_data'ya index 1'den başlayarak erişiliyor!
    
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    vector<Vec3f> triangle_normals;
    vector<vector<Vec3f>> face_normals;

    for (int i=0; i < scene.meshes.size(); i++) {
        vector<Vec3f> vec;
        face_normals.push_back(vec);
    }

    compute_normals(triangle_normals, face_normals, scene);

    for (int camera_id=0; camera_id < scene.cameras.size(); camera_id++) {

        Camera camera = scene.cameras[camera_id];
        int width = camera.image_width;
        int height = camera.image_height;

        unsigned char* image = new unsigned char [width * height * 3];

        float width_of_pixel = (camera.near_plane.y - camera.near_plane.x) / width; // (right-left)/width
        float heigth_of_pixel = (camera.near_plane.w - camera.near_plane.z) / height; // (top-bottom)/height

        Vec3f u = cross_product(camera.up, camera.gaze * (-1)); //u = v * (-w)
        Vec3f middle = camera.position + camera.gaze * camera.near_distance;
        Vec3f q = middle + u * camera.near_plane.x + camera.up * camera.near_plane.w; //q = middle + u * left + v * top 

        int n=0;

        for (int i=0; i < height; i++) {
            for (int j=0; j < width; j++) {

                Ray r;
                r.o = camera.position;
                r.depth = 0;

                float s_u = (j+0.5) * width_of_pixel;
                float s_v = (i+0.5) * heigth_of_pixel;

                Vec3f s = q + u * s_u - camera.up * s_v; //s = q + u*s_u + v*s_v

                r.d = (s - camera.position)/magnitude(s - camera.position);

                Vec3f result = compute_color(r, scene, triangle_normals, face_normals);
                Vec3i rounded_result = {round(result.x), round(result.y), round(result.z)};

                Vec3i color = clamp(rounded_result, 255);
                image[n++] = color.x; //r
                image[n++] = color.y; //g
                image[n++] = color.z; //b
            }
        }

        write_ppm(camera.image_name.c_str(), image, width, height);
    }

}
