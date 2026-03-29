#pragma once

#include "General.hpp"
#include <vector>
#include <glm/gtc/quaternion.hpp>
#include "EaseFunctions.hpp"
#include <concepts>

namespace SSS::Math 
{
    using EasingFunction = std::function<float(float)>;

    template<typename T>
    struct GradientPoint 
    {
        float t      = 0.f;
        T value;
        EaseType easing = EaseType::Linear;
        EasingFunction customEasing;

        GradientPoint(const float _t, const T& v, EaseType e = EaseType::Linear)
            : t(_t), value(v), easing(e), customEasing(nullptr) {}

        GradientPoint(const float _t, const T& v, EasingFunction customFunc)
            : t(_t), value(v), easing(EaseType::Custom), customEasing(customFunc) {}

        template<typename T>
        bool operator<(const GradientPoint<T>& rhs) const {return this->t < rhs.t;}

        template<typename T>
        bool operator>(const GradientPoint<T>& rhs) const {return this->t > rhs.t;}
    };


    template<typename T>
    T interpolate(const T& a, const T& b, float t) {
        return glm::mix(a, b, t);
    }

    // Specialization for quaternions (use slerp)
    template<>
    inline glm::quat interpolate(const glm::quat& a, const glm::quat& b, float t) {
        return glm::slerp(a, b, t);
    }



    template<class T>
    class Gradient {
    public:

        void push(std::pair<float, const T&> p, EaseType easing = EaseType::Linear);
        void push(std::pair<float, const T&> p, EasingFunction easing);
        void push(const GradientPoint<T>& val);

        T max() const requires std::totally_ordered<T>;
        T min() const requires std::totally_ordered<T>;

        T evaluate(const float t) const;

        inline void clear() { v.clear(); };
        inline float getDuration() const { return v.empty() ? 0.0f : v.back().first; };

    private:
        float apply(const EaseType easing, const float t) const ;

        std::vector<std::pair<float, GradientPoint<T>>> v;
    };

    template<class T>
    inline void Gradient<T>::push(std::pair<float, const T&> p, EaseType easing)
    {
        GradientPoint pt(p.first, p.second, easing);
        v.emplace_back(std::make_pair(pt.t, pt));
        std::sort(v.begin(), v.end());
    }

    template<class T>
    inline void Gradient<T>::push(std::pair<float, const T&> p, EasingFunction easing)
    {
        GradientPoint pt(p.first, p.second, easing);
        v.emplace_back(std::make_pair(pt.t, pt));
        std::sort(v.begin(), v.end());
    }

    template<class T>
    inline void Gradient<T>::push(const GradientPoint<T>& val)
    {
        v.emplace_back(std::make_pair(val.t, val));
        std::sort(v.begin(), v.end());
    }




    template<class T>
    inline T Gradient<T>::max() const requires std::totally_ordered<T>
    {
        T tmp = v[0].second.value;
        for (uint32_t i = 1; i < v.size(); i++) {
            tmp = std::max(tmp, v[i].second.value);
        }
        return tmp;
    }

    template<class T>
        inline T Gradient<T>::min() const requires std::totally_ordered<T>
    {
        T tmp = v[0].second;
        for (uint32_t i = 1; i < v.size(); i++) {
            tmp = std::min(tmp, v[i].second.value);
        }
        return tmp;
    }

    template<class T>
    inline T Gradient<T>::evaluate(const float t) const
    {
        if (v.empty()) {
            return T{};
        }

        //If the vector is filled with only one composent bypass the calculation
        if (v.size() == 1 || t <= v.front().first) {
            return v.front().second.value;
        }

        if (t >= v.back().first) {
            return v.back().second.value;
        }


        ////Find between which pair the parameter t is falling
        //t = std::clamp(t, v[0].first, v[v.size() - 1].first);

        for (size_t i = 0; i < v.size() - 1; ++i) {
            if (t >= v[i].second.t && t <= v[i + 1].second.t) {
                const auto& k1 = v[i].second;
                const auto& k2 = v[i + 1].second;

                float duration = k2.t - k1.t;
                float p = 0.f;
                if(duration != 0)
                    p = (t - k1.t) / duration;

                // Apply easing
                if (k1.easing == EaseType::Custom && k1.customEasing) {
                    p = k1.customEasing(t);
                }
                else {
                    p = apply(k1.easing, t);
                }

                return interpolate(k1.value, k2.value, p);
            }
            //Then return the value for the corresponding t

        }

        return v.back().second.value;
    }

    template<class T>
    inline float Gradient<T>::apply(const EaseType easing, const float t) const
    {
        switch (easing)
        {
            case EaseType::Linear               :return t;
            case EaseType::SineEaseIn           :return easeInSine(t);
            case EaseType::SineEaseOut          :return easeOutSine(t);
            case EaseType::SineEaseInOut        :return easeInOutSine(t);
            case EaseType::QuadEaseIn           :return easeInQuad(t);
            case EaseType::QuadEaseOut          :return easeOutQuad(t);
            case EaseType::QuadEaseInOut        :return easeInOutQuad(t);
            case EaseType::CubicEaseIn          :return easeInCubic(t);
            case EaseType::CubicEaseOut         :return easeOutCubic(t);
            case EaseType::CubicEaseInOut       :return easeInOutCubic(t);
            case EaseType::QuartEaseIn          :return easeInQuart(t);
            case EaseType::QuartEaseOut         :return easeOutQuart(t);
            case EaseType::QuartEaseInOut       :return easeInOutQuart(t);
            case EaseType::QuintEaseIn          :return easeInQuint(t);
            case EaseType::QuintEaseOut         :return easeOutQuint(t);
            case EaseType::QuintEaseInOut       :return easeInOutQuint(t);
            case EaseType::ExpoEaseIn           :return easeInExpo(t);
            case EaseType::ExpoEaseOut          :return easeOutExpo(t);
            case EaseType::ExpoEaseInOut        :return easeInOutExpo(t);
            case EaseType::CircEaseIn           :return easeInCirc(t);
            case EaseType::CircEaseOut          :return easeOutCirc(t);
            case EaseType::CircEaseInOut        :return easeInOutCirc(t);
            case EaseType::BackEaseIn           :return easeInBack(t);
            case EaseType::BackEaseOut          :return easeOutBack(t);
            case EaseType::BackEaseInOut        :return easeInOutBack(t);
            case EaseType::ElasticEaseIn        :return easeInElastic(t);
            case EaseType::ElasticEaseOut       :return easeOutElastic(t);
            case EaseType::ElasticEaseInOut     :return easeInOutElastic(t);
            case EaseType::BounceEaseIn         :return easeInBounce(t);
            case EaseType::BounceEaseOut        :return easeOutBounce(t);
            case EaseType::BounceEaseInOut      :return easeInOutBounce(t);
        default:
            return t;
        }
    }
}