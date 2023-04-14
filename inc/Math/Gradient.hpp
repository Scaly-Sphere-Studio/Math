#pragma once

#include "General.hpp"
#include <vector>


namespace SSS::Math {

    template<class T>
    class Gradient {
    public:

        void push(std::pair<float, T>);

        T max();

        T evaluate(float t);

    private:
        std::vector<std::pair<float, T>> v;
    };

    template<class T>
    inline void Gradient<T>::push(std::pair<float, T> p)
    {
        v.emplace_back(p);
    }

    template<class T>
    inline T Gradient<T>::max()
    {
        T tmp = v[0].second;
        for (uint32_t i = 1; i < v.size(); i++) {
            if (tmp < v[i].second) {
                tmp = v[i].second;
            }
        }
        return tmp;
    }

    template<class T>
    inline T Gradient<T>::evaluate(float t)
    {
        //If the vector is filled with only one composent bypass the calculation
        if (v.size() == 1) {
            return v[0].second;
        }

        size_t index = 0;

        //Find between which pair the paramater t is falling
        t = std::clamp(t, v[0].first, v[v.size() - 1].first);
        for (size_t i = 0; i < v.size() - 1; i++) {
            if (t <= v[i + 1].first) {
                index = i;
                break;
            }
        }

        //Then return the value for the corresponding t
        return SSS::Math::lerp(v[index].second, v[index + 1].second, (t - v[index].first) / (v[index + 1].first - v[index].first));
    }

}