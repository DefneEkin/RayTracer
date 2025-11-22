#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>
#include <cmath>


namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;

        Vec3f operator+(const Vec3f& other) const {
            return {x + other.x, y + other.y, z + other.z}; //benim
        }

        Vec3f operator-(const Vec3f& other) const {
            return {x - other.x, y - other.y, z - other.z}; //benim
        }

        Vec3f operator*(const float val) const {
            return {x * val, y * val, z * val}; //benim
        }

        float operator*(const Vec3f& other) const {
            return x * other.x + y * other.y + z * other.z; 
        }

    };

    inline Vec3f elementwise_mult(const Vec3f& a, const Vec3f& b) {
        return {a.x*b.x, a.y*b.y, a.z*b.z};
    }

    inline Vec3f cross_product(const Vec3f& a, const Vec3f& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    inline float magnitude(const Vec3f& a) {
        return sqrt(pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2));
    }

    inline Vec3f operator*(float val, const Vec3f& vec) {
        return {vec.x * val, vec.y * val, vec.z * val};
    }

    inline Vec3f operator/(const Vec3f& vec, float val) {
        return {vec.x / val, vec.y / val, vec.z / val};
    }


    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane; //left, rigth, bottom, top
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
        Vec3f normal;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
        Vec3f normal;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string &filepath);
    };
}

#endif
