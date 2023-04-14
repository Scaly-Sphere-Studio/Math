#include "Math.hpp"
static constexpr float  THREE_POINT_ALIGNEMENT_TOLERANCE = 0.023f;

namespace SSS::Math {

    glm::mat3 rotation_matrix2D_ccw(double angle)
    {
        return glm::mat3(glm::vec3(glm::cos(angle), -glm::sin(angle), 0),
            glm::vec3(glm::sin(angle), glm::cos(angle), 0),
            glm::vec3(0, 0, 1));
    }

    glm::mat3 rotation_matrix2D_cw(double angle)
    {
        return glm::mat3(glm::vec3(glm::cos(angle), glm::sin(angle), 0),
            glm::vec3(-glm::sin(angle), glm::cos(angle), 0),
            glm::vec3(0, 0, 1));
    }

    glm::vec3 direction_vector2D(glm::vec3 v1, glm::vec3 v2)
    {
        glm::vec2 a(v1.x, v1.y);
        glm::vec2 b(v2.x, v2.y);
        return glm::vec3(glm::normalize(b - a), v1.z);
    }


    glm::vec3 ortho_vector(glm::vec3& v1, glm::vec3& v2)
    {
        return glm::vec3(-direction_vector2D(v1, v2).y, direction_vector2D(v1, v2).x, v1.z);
    }

    float intersect_factor(glm::vec3& p1, glm::vec3& p2, glm::vec3& p3, glm::vec3& p4)
    {
        float numerator = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
        float denumerator = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);

        if (glm::abs(denumerator) < 1e-6) {
            //Return 1 when the two vectors are parallel
            //As this function is used only in a 3 points case, the two vectors are always connected
            //When the denumerator is equal to 0, it indicate that the two lines are parallel
            return 1;
        }
        return numerator / denumerator;
    }

    glm::vec3 intersection_point(glm::vec3& p1, glm::vec3& p2, glm::vec3& p3, glm::vec3& p4)
    {
        float factor = intersect_factor(p1, p2, p3, p4);

        float px = p1.x + factor * (p2.x - p1.x);
        float py = p1.y + factor * (p2.y - p1.y);

        return glm::vec3(px, py, p1.z);
    }

    float incidence_angle(glm::vec3 v1, glm::vec3 v2)
    {
        return glm::acos(std::clamp(glm::dot(v1, v2) / (glm::length(v1) * glm::length(v2)), -1.0f, 1.0f));
    }

    glm::vec3 bezier_func(float t, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d) {
        return glm::vec3(std::powf((1.f - t), 3.f)) * a +
            glm::vec3(3.f * std::powf((1.f - t), 2.f) * t) * b +
            glm::vec3(3.f * (1.f - t) * std::powf(t, 2.f)) * c +
            glm::vec3(std::powf(t, 3.f)) * d;
    }

    bool three_point_colinear_test(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
        //NORMALIZED 3 POINT ALIGNEMENT TEST
        glm::vec2 ab = SSS::Math::direction_vector2D(a, b);
        glm::vec2 ac = SSS::Math::direction_vector2D(a, c);
        return std::abs(glm::determinant(glm::mat2(ab, ac))) > THREE_POINT_ALIGNEMENT_TOLERANCE;
    }

    void bezier_recurs(std::vector<std::pair<float, glm::vec3>>& v, const std::pair<float, glm::vec3> pa, const std::pair<float, glm::vec3> pb,
        glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d) {

        float t = (pa.first + pb.first) / 2.0f;
        std::pair<float, glm::vec3> pm = std::make_pair(t, bezier_func(t, a, b, c, d));

        if (three_point_colinear_test(pa.second, pm.second, pb.second)) {
            v.emplace_back(pm);
            bezier_recurs(v, pa, pm, a, b, c, d);
            bezier_recurs(v, pm, pb, a, b, c, d);
        }
    }
}