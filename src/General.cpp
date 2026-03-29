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


    glm::vec3 ortho_vector(glm::vec3 v1, glm::vec3 v2)
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


    // BEZIER

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

    /* Bezier intersection */
    std::array<float, 4> BezierCoeffs(const float P0, const float P1, const float P2, const float P3)
    {
        std::array<float, 4> Z;
        Z[0] = -P0 + 3.0f * P1 + -3.0f * P2 + P3;
        Z[1] = 3.0f * P0 - 6.0f * P1 + 3.0f * P2;
        Z[2] = -3.0f * P0 + 3.0f * P1;
        Z[3] = P0;

        return Z;
    }


    std::array<float, 3> CubicRoots(const float a, const float b,  const float c, const float d)
    {

        float A = b / a;
        float B = c / a;
        float C = d / a;

        float Im;

        float Q = (3.f * B - std::pow(A, 2.f)) / 9.f;
        float R = (9.f * A * B - 27.f * C - 2.f * std::pow(A, 3.f)) / 54.f;
        float D = std::pow(Q, 3.f) + std::pow(R, 2.f);    // polynomial discriminant

        std::array<float, 3U> t;

        if (D >= 0)                                 // complex or duplicate roots POI
        {
            float S = SSS::Math::signum(R + std::sqrt(D)) * std::pow(std::abs(R + std::sqrt(D)), (1.0f / 3.0f));
            float T = SSS::Math::signum(R - std::sqrt(D)) * std::pow(std::abs(R - std::sqrt(D)), (1.0f / 3.0f));

            t[0] = -A / 3.0f + (S + T);                         // real root
            t[1] = -A / 3.0f - (S + T) / 2.0f;                  // real part of complex root
            t[2] = -A / 3.0f - (S + T) / 2.0f;                  // real part of complex root
            Im = std::abs(std::sqrt(3.0f) * (S - T) / 2.0f);    // complex part of root pair   

            //discard complex roots//
            if (Im != 0) {
                t[1] = -1.f;
                t[2] = -1.f;
            }

        }
        else                                          // distinct real roots
        {
            float th = std::acos(R / std::sqrt(-std::pow(Q, 3.f)));

            t[0] = 2.0f * std::sqrt(-Q) * std::cos(th / 3.0f) - A / 3.0f;
            t[1] = 2.0f * std::sqrt(-Q) * std::cos((th + 2.0f * glm::pi<float>()) / 3.0f) - A / 3.0f;
            t[2] = 2.0f * std::sqrt(-Q) * std::cos((th + 4.0f * glm::pi<float>()) / 3.0f) - A / 3.0f;
            Im = 0.0f;
        }

        /*discard out of spec roots*/
        for (size_t i = 0; i < t.size(); i++) {
            if (t[i] < 0 || t[i] > 1.0) {
                t[i] = -1;
            }
        }


        return t;
    }

    //px and py are the coordinates of the start, first tangent, second tangent, end in that order. length = 4
    //lx and ly are the start then end coordinates of the stright line. length = 2
    bool cubic_bezier_segment_intersection(const glm::vec3 b_a, const glm::vec3 b_b, const glm::vec3 b_c, const glm::vec3 b_d,
        const glm::vec3 s_a, const glm::vec3 s_b) 
    {
        glm::vec2 X;

        float A = s_b.y - s_a.y;      //A=y2-y1
        float B = s_a.x - s_b.x;      //B=x1-x2
        float C = s_a.x * (s_a.y - s_b.y) + s_a.y * (s_b.x - s_a.x);  //C=x1*(y1-y2)+y1*(x2-x1)

        std::array<float, 4> bx = BezierCoeffs(b_a.x, b_b.x, b_c.x, b_d.x);
        std::array<float, 4> by = BezierCoeffs(b_a.y, b_b.y, b_c.y, b_d.y);

        std::array<float, 4> P;
        P[0] = A * bx[0] + B * by[0];       /*t^3*/
        P[1] = A * bx[1] + B * by[1];       /*t^2*/
        P[2] = A * bx[2] + B * by[2];       /*t*/
        P[3] = A * bx[3] + B * by[3] + C;   /*1*/

        std::array<float, 3> r = CubicRoots(P[0], P[1], P[2], P[3]);

        /*verify the roots are in bounds of the linear segment*/
        for (size_t i = 0; i < r.size(); i++) {
            float t = r[i];

            X[0] = bx[0] * t * t * t + bx[1] * t * t + bx[2] * t + bx[3];
            X[1] = by[0] * t * t * t + by[1] * t * t + by[2] * t + by[3];

            /*above is intersection point assuming infinitely long line segment,
              make sure we are also in bounds of the line*/
            float s;
            if ((s_b.x - s_a.x) != 0) { s = (X[0] - s_a.x) / (s_b.x - s_a.x); }          /*if not vertical line*/
            else { s = (X[1] - s_a.y) / (s_b.y - s_a.y); }

            /*in bounds?*/
            if (t > 0 && t < 1.0 && s > 0 && s < 1.0) {
                return true;
            }
        }
        return false;
    }
}