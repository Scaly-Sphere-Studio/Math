#pragma once

#include <SSS/Commons.hpp>
#include <glm/gtc/constants.hpp>

#ifdef SSS_MATH_EXPORTS
#define SSS_MATH_API __declspec(dllexport)
#else
#define SSS_MATH_API __declspec(dllimport)
#endif

namespace SSS::Math {
    enum class EaseType {
        Linear,
        Custom,

        // Sine ease functions
        SineEaseIn,
        SineEaseOut,
        SineEaseInOut,

        // Quadratic ease functions
        QuadEaseIn,
        QuadEaseOut,
        QuadEaseInOut,

        // Cubic ease functions
        CubicEaseIn,
        CubicEaseOut,
        CubicEaseInOut,

        // Quart ease functions
        QuartEaseIn,
        QuartEaseOut,
        QuartEaseInOut,

        // Quint ease functions
        QuintEaseIn,
        QuintEaseOut,
        QuintEaseInOut,

        // Exponential ease functions
        ExpoEaseIn,
        ExpoEaseOut,
        ExpoEaseInOut,

        // Circular ease functions
        CircEaseIn,
        CircEaseOut,
        CircEaseInOut,

        // Back ease functions
        BackEaseIn,
        BackEaseOut,
        BackEaseInOut,

        // Elastic ease functions
        ElasticEaseIn,
        ElasticEaseOut,
        ElasticEaseInOut,

        // Bouncy ease functions
        BounceEaseIn,
        BounceEaseOut,
        BounceEaseInOut,
    };


    //  --------------------- EASEIN SIN --------------------------
    //.                                                           .
    //.                                               :-          .
    //.                                             .=            .
    //.                                            -.             .
    //.                                          -:               .
    //.                                        .-                 .
    //.                                      .=                   .
    //.                                    .+                     .
    //.                                  .#                       .
    //.                                =%                         .
    //.                              #%                           .
    //.                            %=                             .
    //.                         #%                                .
    //.                      *%.                                  .
    //.                 .*@*                                      .
    //.         *@@@@#+.                                          .
    //.                                                           .
    SSS_MATH_API inline float easeInSine(const float x)
    {
        return 1.f - glm::cos((x * glm::pi<float>()) / 2.f);
    }

    //  --------------------- EASEOUT SINE ------------------------
   //.                                                            .
   //.                                                            .
   //.                                        :*@%+-:..           .
   //.                                    -@%.                    .
   //.                                 =@+                        .
   //.                              .@*                           .
   //.                            +@.                             .
   //.                          %#                                .
   //.                        %=                                  .
   //.                      *-                                    .
   //.                    +:                                      .
   //.                  -:                                        .
   //.                --                                          .
   //.              :-                                            .
   //.            .=                                              .
   //.           -:                                               .
   //.         .-                                                 .
   //.                                                            .
    SSS_MATH_API inline float easeOutSine(const float x)
    {
        return glm::sin((x * glm::pi<float>()) / 2.f);
    }

    //  ------------------- EASE IN OUT SIN ------------------------
    //.                                                            .
    //.                                           .-=:..           .
    //.                                        :=.                 .
    //.                                      =-                    .
    //.                                    -:                      .
    //.                                  -:                        .
    //.                                --                          .
    //.                              .=                            .
    //.                             =.                             .
    //.                           -:                               .
    //.                         :-                                 .
    //.                       .-                                   .
    //.                     .=                                     .
    //.                   :-                                       .
    //.                :-.                                         .
    //.          ===-.                                             .
    //.                                                            .
    SSS_MATH_API inline float easeInOutSine(const float x)
    {
        return -(glm::cos(glm::pi<float>() * x) - 1) / 2.f;
    }

    //  --------------------- EASEIN QUAD --------------------------
    //.                                                            .
    //.                                              -:            .
    //.                                             =              .
    //.                                           :-               .
    //.                                          =.                .
    //.                                        :-                  .
    //.                                       =                    .
    //.                                     +.                     .
    //.                                   *-                       .
    //.                                 %=                         .
    //.                              .@-                           .
    //.                            +@.                             .
    //.                         -@=                                .
    //.                      =@=                                   .
    //.                 :*@*                                       .
    //.        @@@@@%*-                                            .
    //.                                                            .
    SSS_MATH_API inline float easeInQuad(const float x)
    {
        return x * x;
    }

    //  --------------------- EASEOUT QUAD -------------------------
    //.                                                            .
    //.                                       =#@*=:.  .           .
    //.                                  :%@-                      .
    //.                               =@*                          .
    //.                            :@*                             .
    //.                          *%                                .
    //.                        @+                                  .
    //.                      #=                                    .
    //.                    *-                                      .
    //.                  --                                        .
    //.                .=                                          .
    //.               -.                                           .
    //.             :-                                             .
    //.            -.                                              .
    //.          .-                                                .
    //.         :.                                                 .
    SSS_MATH_API inline float easeOutQuad(const float x)
    {
        return 1 - (1 - x) * (1 - x);
    }

    //  ------------------- EASE IN OUT QUAD -----------------------
    //.                                                            .
    //.                                          .-=-..            .
    //.                                       :-.                  .
    //.                                     =.                     .
    //.                                   -.                       .
    //.                                 -:                         .
    //.                                =                           .
    //.                              -.                            .
    //.                             =                              .
    //.                           -.                               .
    //.                         .-                                 .
    //.                        -.                                  .
    //.                      -:                                    .
    //.                   .-:                                      .
    //.                .--                                         .
    //.         .===-:                                             .
    SSS_MATH_API inline float easeInOutQuad(const float x)
    {
        return x < 0.5 ? 2.f * x * x : 1 - glm::pow(-2.f * x + 2.f, 2.f) / 2.f;
    }

    //  --------------------- EASEIN CUBIC -------------------------
    //.                                                            .
    //.                                                =           .
    //.                                               =            .
    //.                                              =             .
    //.                                             =              .
    //.                                            =               .
    //.                                          .-                .
    //.                                         -:                 .
    //.                                        +                   .
    //.                                      ++                    .
    //.                                    :%                      .
    //.                                  :@                        .
    //.                                #%                          .
    //.                             *@:                            .
    //.                        -#@+                                .
    //.         *@@@@@@@@@%*=.                                     .
    //.                                                            .
    SSS_MATH_API inline float easeInCubic(const float x)
    {
        return x * x * x;
    }

    //  --------------------- EASE OUT CUBIC -----------------------
    //.                                                            .
    //.                                 .*%@%*-::....              .
    //.                             +@#:                           .
    //.                          +@-                               .
    //.                       .@+                                  .
    //.                      @=                                    .
    //.                    #+                                      .
    //.                  :*                                        .
    //.                 +.                                         .
    //.                =                                           .
    //.              .-                                            .
    //.             :-                                             .
    //.            ::                                              .
    //.           -:                                               .
    //.          ::                                                .
    //.         .:                                                 .
    //.                                                            .
    SSS_MATH_API inline float easeOutCubic(const float x)
    {
        return 1.f - glm::pow(1.f - x, 3.f);
    }

    //  ------------------- EASE IN OUT CUBIC ----------------------
    //.                                                            .
    //.                                        .-==:...            .
    //.                                     -=.                    .
    //.                                   -.                       .
    //.                                 .-                         .
    //.                                -.                          .
    //.                               =                            .
    //.                              -                             .
    //.                             -                              .
    //.                            =                               .
    //.                           =                                .
    //.                          =                                 .
    //.                        :-                                  .
    //.                      :-                                    .
    //.                  .--.                                      .
    //.          ====-::                                           .
    //.                                                            .
    SSS_MATH_API inline float easeInOutCubic(const float x)
    {
        return x < 0.5f ? 4.f * x * x * x : 1 - glm::pow(-2.f * x + 2.f, 3.f) / 2.f;
    }

    //  ---------------------- EASE IN QUART -----------------------
    //.                                                            .
    //.                                                 .-         .
    //.                                                 =          .
    //.                                                -.          .
    //.                                               ::           .
    //.                                              .-            .
    //.                                              -             .
    //.                                             =              .
    //.                                           .=               .
    //.                                          -=                .
    //.                                         #.                 .
    //.                                       +#                   .
    //.                                     +@                     .
    //.                                  .@+                       .
    //.                              +%@-                          .
    //.           @@@@@@@@@@@@@%#+=                                .
    //.                                                            .
    SSS_MATH_API inline float easeInQuart(const float x)
    {
        return x * x * x * x;
    }


    //  -------------------- EASE OUT QUART ------------------------
    //.                                                            .
    //.                             :*%@%*=:::.....                .
    //.                         *@+.                               .
    //.                      =@:                                   .
    //.                    +%                                      .
    //.                  .%                                        .
    //.                 +=                                         .
    //.                +                                           .
    //.               =                                            .
    //.              =                                             .
    //.             -                                              .
    //.            -.                                              .
    //.           ::                                               .
    //.          .-                                                .
    //.          -                                                 .
    //.         :.                                                 .
    //.                                                            .
    SSS_MATH_API inline float easeOutQuart(const float x)
    {
        return 1.f - glm::pow(1 - x, 4.f);
    }

    //  ------------------- EASE IN OUT QUART ----------------------
    //.                                                            .
    //.                                       :-=-:..              .
    //.                                   .=:                      .
    //.                                 .=                         .
    //.                                -.                          .
    //.                               =                            .
    //.                              -.                            .
    //.                             .-                             .
    //.                             =                              .
    //.                            -.                              .
    //.                           .-                               .
    //.                          .-                                .
    //.                         -:                                 .
    //.                       :-                                   .
    //.                   .--.                                     .
    //.         .====--::                                          .
    //.                                                            .
    SSS_MATH_API inline float easeInOutQuart(const float x)
    {
        return x < 0.5f ? 8.f * x * x * x * x : 1 - glm::pow(-2.f * x + 2.f, 4.f) / 2.f;
    }


    //  --------------------- EASE IN QUINT ------------------------
    //.                                                            .
    //.                                                =           .
    //.                                               -.           .
    //.                                               =            .
    //.                                              =             .
    //.                                             ::             .
    //.                                             -              .
    //.                                            =               .
    //.                                           =                .
    //.                                          +                 .
    //.                                         #                  .
    //.                                       :%                   .
    //.                                      @:                    .
    //.                                   =@.                      .
    //.                               *@@:                         .
    //.         *@@@@@@@@@@@@@@%#*=-                               .
    //.                                                            .
    SSS_MATH_API inline float easeInQuint(const float x)
    {
        return x * x * x * x * x;
    }


    //  --------------------- EASE OUT QUINT -----------------------
    //.                                                            .
    //.                           +%@@#=:.....                     .
    //.                       #@:                                  .
    //.                    :@.                                     .
    //.                   @                                        .
    //.                 =+                                         .
    //.                +:                                          .
    //.               =.                                           .
    //.              -.                                            .
    //.             ::                                             .
    //.             -                                              .
    //.            -                                               .
    //.           -.                                               .
    //.          .-                                                .
    //.          -                                                 .
    //.         .:                                                 .
    //.                                                            .

    SSS_MATH_API inline float easeOutQuint(const float x)
    {
        return 1.f - glm::pow(1.f - x, 5.f);
    }


    //  ------------------- EASE IN OUT QUINT ----------------------
    //.                                                           .
    //.                                      .-==:..              .
    //.                                   --.                     .
    //.                                 -.                        .
    //.                                =                          .
    //.                               =                           .
    //.                              -                            .
    //.                              -                            .
    //.                             -.                            .
    //.                             =                             .
    //.                            -.                             .
    //.                           ::                              .
    //.                          -:                               .
    //.                        :=                                 .
    //.                    .-=.                                   .
    //.          =====--:.                                        .
    //.                                                           .
    SSS_MATH_API inline float easeInOutQuint(const float x)
    {
        return x < 0.5f ? 16.f * x * x * x * x * x : 1 - glm::pow(-2.f * x + 2.f, 5.f) / 2.f;
    }

    //  ---------------------- EASE IN EXPO -----------------------
    //.                                                           .
    //.                                                 -         .
    //.                                                -.         .
    //.                                                =          .
    //.                                               :.          .
    //.                                               =           .
    //.                                              -            .
    //.                                             .-            .
    //.                                             =             .
    //.                                            +.             .
    //.                                           *               .
    //.                                          %                .
    //.                                        #*                 .
    //.                                      %*                   .
    //.                                 =%@+                      .
    //.          @@@@@@@@@@@@@@@@%%*=-.                           .
    //.                                                           .
    SSS_MATH_API inline float easeInExpo(const float x)
    {
        return x == 0 ? 0 : glm::pow(2.f, 10 * x - 10);
    }


    //  --------------------- EASE OUT EXPO -----------------------
    //.                                                           .
    //.                         +%@@%+-......                     .
    //.                     #%:                                   .
    //.                  -%.                                      .
    //.                 %.                                        .
    //.               :*                                          .
    //.              :=                                           .
    //.             .=                                            .
    //.             -                                             .
    //.            -.                                             .
    //.            -                                              .
    //.           -                                               .
    //.          .-                                               .
    //.          -                                                .
    //.         .-                                                .
    //.         :                                                 .
    //.                                                           .
    SSS_MATH_API inline float easeOutExpo(const float x)
    {
        return x == 1 ? 1 : 1 - glm::pow(2.f, -10 * x);
    }


    //  ------------------ EASE IN  OUT EXPO ----------------------
    //.                                                           .
    //.                                      --=-:.               .
    //.                                  -=.                      .
    //.                                :-                         .
    //.                               -                           .
    //.                              -.                           .
    //.                              -                            .
    //.                             :.                            .
    //.                             =                             .
    //.                            .:                             .
    //.                            -                              .
    //.                           ::                              .
    //.                          --                               .
    //.                        .=                                 .
    //.                    .--.                                   .
    //.         .===---:..                                        .
    //.                                                           .
    SSS_MATH_API inline float easeInOutExpo(const float x)
    {
        return x == 0
            ? 0
            : x == 1
            ? 1
            : x < 0.5 ? glm::pow(2.f, 20 * x - 10) / 2.f
            : (2.f - glm::pow(2.f, -20 * x + 10)) / 2.f;

    }

    //  ---------------------- EASE IN CIRC -----------------------
    //.                                                           .
    //.                                                -.         .
    //.                                                -          .
    //.                                                =          .
    //.                                               -.          .
    //.                                              .-           .
    //.                                             .-            .
    //.                                            :-             .
    //.                                           =.              .
    //.                                         +=                .
    //.                                       #*                  .
    //.                                    -@.                    .
    //.                                 +@-                       .
    //.                             =@%                           .
    //.                       .*%%-                               .
    //.         +####%%@@%+:.                                     .
    //.                                                           .
    SSS_MATH_API inline float easeInCirc(const float x)
    {
        return 1 - glm::sqrt(1 - glm::pow(x, 2.f));
    }


    //  --------------------- EASE OUT CIRC -----------------------
    //.                                                           .
    //.                                     -%@@@@#+==--.         .
    //.                              -%@#.                        .
    //.                          +@*.                             .
    //.                       *%.                                 .
    //.                    +%.                                    .
    //.                  #-                                       .
    //.                +:                                         .
    //.              :-                                           .
    //.             -                                             .
    //.            -                                              .
    //.           =                                               .
    //.          :.                                               .
    //.          =                                                .
    //.         .-                                                .
    //.         .:                                                .
    //.                                                           .
    SSS_MATH_API inline float easeOutCirc(const float x)
    {
        return glm::sqrt(1 - glm::pow(x - 1, 2.f));
    }

    //  -------------------- EASE IN OUT CIRC ---------------------
    //.                                                           .
    //.                                       .-===--::..         .
    //.                                   :=.                     .
    //.                                 -:                        .
    //.                                =                          .
    //.                               =                           .
    //.                              -                            .
    //.                              -                            .
    //.                             -.                            .
    //.                             =                             .
    //.                            :.                             .
    //.                           .-                              .
    //.                          .-                               .
    //.                         -.                                .
    //.                     .--.                                  .
    //.          ------=-:.                                       .
    //.                                                           .
    SSS_MATH_API inline float easeInOutCirc(const float x)
    {
        return x < 0.5
            ? (1 - glm::sqrt(1 - glm::pow(2.f * x, 2.f))) / 2.f
            : (glm::sqrt(1 - glm::pow(-2.f * x + 2.f, 2.f)) + 1) / 2.f;
    }


    //  ---------------------- EASE IN BACK ------------------------
    //.                                                            .
    //.                                                 =          .
    //.                                                -.          .
    //.                                               .-           .
    //.                                               =            .
    //.                                              -             .
    //.                                             .-             .
    //.                                             =              .
    //.                                            =               .
    //.                                           +                .
    //.                                          +.                .
    //.                                         #.                 .
    //.                                        %                   .
    //.                                      .@                    .
    //.                                     %=                     .
    //.          #*+-                     %#                       .
    //.                :+@@#=.        .*@-                         .
    SSS_MATH_API inline float easeInBack(const float x)
    {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1;

        return c3 * x * x * x - c1 * x * x;
    }

    //  ---------------------- EASE OUT BACK -----------------------
    //.                        .@%           :%@@#:                .
    //.                      +%                       .:           .
    //.                    .@                                      .
    //.                   %-                                       .
    //.                  #.                                        .
    //.                 #                                          .
    //.                +                                           .
    //.               =                                            .
    //.              -.                                            .
    //.             :.                                             .
    //.            .-                                              .
    //.            =                                               .
    //.           -.                                               .
    //.          .:                                                .
    //.          -                                                 .
    //.         :.                                                 .
    //.                                                            .
    SSS_MATH_API inline float easeOutBack(const float x)
    {
        const float c1 = 1.70158f;
        const float c3 = c1 + 1;

        return 1.f + c3 * glm::pow(x - 1.f, 3.f) + c1 * glm::pow(x - 1.f, 2.f);
    }

    //  ------------------- EASE IN OUT BACK -----------------------
    //.                                   .=        .=-            .
    //.                                  =.                        .
    //.                                 =                          .
    //.                                -                           .
    //.                               -.                           .
    //.                               =                            .
    //.                              =                             .
    //.                             .-                             .
    //.                             -                              .
    //.                            .:                              .
    //.                            =                               .
    //.                           :.                               .
    //.                          .-                                .
    //.                          =                                 .
    //.                         -                                  .
    //.         .:            .-                                   .
    //.           .--.      :-.                                    .
    SSS_MATH_API inline float easeInOutBack(const float x)
    {
        const float c1 = 1.70158f;
        const float c2 = c1 * 1.525f;

        return x < 0.5
            ? (glm::pow(2.f * x, 2.f) * ((c2 + 1.f) * 2.f * x - c2)) / 2.f
            : (glm::pow(2.f * x - 2.f, 2.f) * ((c2 + 1) * (x * 2.f - 2.f) + c2) + 2.f) / 2.f;
    }


    //  -------------------- EASE IN ELASTIC -----------------------
    //.                                                            . 
    //.                                                            .
    //.                                                            .
    //.                                      -                     .
    //.                                      =                     .
    //.                                      =                     .
    //.                                      :                     .
    //.                                     :.                     .
    //.                                     =                      .
    //.                                     =                      .
    //.                                     =                      .
    //.                              *:    .=                      .
    //.                            #+ +=   ::                      .
    //.               .:::    .%@@@    %   +                       .
    //.                                :#  %                       .
    //.                                 @  #                       .
    //.                                 .@*-                       .
    //.                                  .-                        .
    //.                                                            .
    SSS_MATH_API inline float easeInElastic(const float x) {
        const float c4 = (2.f * glm::pi<float>()) / 3;

        return x == 0
            ? 0
            : x == 1
            ? 1
            : -glm::pow(2.f, 10.f * x - 10.f) * glm::sin((x * 10.f - 10.75f) * c4);
    }


    //  -------------------- EASE OUT ELASTIC ----------------------
    //.                                                            .    
    //.             @*                                             .
    //.            %.%.                                            .
    //.            %  @                                            .
    //.            +  #:                                           .
    //.           =.   %   #* .%%##%@@@@@@%%@@+                    .
    //.           +    :*:@                                        .
    //.           =                                                .
    //.           -                                                .
    //.          .-                                                .
    //.          :.                                                .
    //.          -                                                 .
    //.          =                                                 .
    //.          -                                                 .
    //.          -                                                 .
    //.         ::                                                 .
    //.         :.                                                 .
    //.                                                            .
    //.                                                            .
    SSS_MATH_API inline float easeOutElastic(const float x)
    {
        float c4 = (2.f * glm::pi<float>()) / 3;
        return x == 0
            ? 0
            : x == 1
            ? 1
            : glm::pow(2.f, -10.f * x) * glm::sin((x * 10.f - 0.75f) * c4) + 1.f;
    }

    //  ------------------ EASE IN OUT ELASTIC ---------------------
    //.                                ---                         .
    //.                                -  =                        .
    //.                               -    .==.                    .
    //.                               =                            .
    //.                              .-                            .
    //.                              :.                            .
    //.                              =                             .
    //.                              -                             .
    //.                             ::                             .
    //.                             -                              .
    //.                             =                              .
    //.                            .:                              .
    //.                            -.                              .
    //.                            =                               .
    //.                           .-                               .
    //.                           :.                               .
    //.          ------::---:-:   =                                .
    //.                       .- .:                                .
    //.                         ::                                 .
    SSS_MATH_API inline float easeInOutElastic(const float x)
    {
        const float c5 = (2.f * glm::pi<float>()) / 4.5f;

        return x == 0
            ? 0
            : x == 1
            ? 1
            : x < 0.5f
            ? -(glm::pow(2.f, 20.f * x - 10.f) * glm::sin((20.f * x - 11.125f) * c5)) / 2.f
            : (glm::pow(2.f, -20.f * x + 10.f) * glm::sin((20.f * x - 11.125f) * c5)) / 2.f + 1;
    }


    //TODO VERIFICATION ON THIS EASE FUNCTIONS DUE TO MODIFICATION OF X VALUE


    //  -------------------- EASE OUT BOUNCE -----------------------
    //.                                                            .
    //.                                                            .
    //.                       @@            @#@   %@ ==            .
    //.                      =* @          @                       .
    //.                      @   %+      #*                        .
    //.                     *=     +@##@+                          .
    //.                     @                                      .
    //.                    #.                                      .
    //.                   .+                                       .
    //.                   +                                        .
    //.                  -.                                        .
    //.                 ::                                         .
    //.                .-                                          .
    //.               .-                                           .
    //.              -:                                            .
    //.             =                                              .
    //.         :--                                                .
    //.                                                            .
    //.                                                            .
    SSS_MATH_API inline float easeOutBounce(const float t)
    {
        const float n1 = 7.5625;
        const float d1 = 2.75f;

        float x = t;
        if (x < 1 / d1) {
            return n1 * x * x;
        }
        else if (x < 2.f / d1) {
            return n1 * (x -= 1.5f / d1) * x + 0.75f;
        }
        else if (x < 2.5f / d1) {
            return n1 * (x -= 2.25f / d1) * x + 0.9375f;
        }
        else {
            return n1 * (x -= 2.625f / d1) * x + 0.984375f;
        }

    }



    //  --------------------- EASE IN BOUNCE -----------------------
    //.                                                            .
    //.                                                            .
    //.                                               -=           .
    //.                                             -:             .
    //.                                            =               .
    //.                                           =                .
    //.                                          =                 .
    //.                                         -                  .
    //.                                        :-                  .
    //.                                        +                   .
    //.                                       #                    .
    //.                                      ==                    .
    //.                                      @                     .
    //.                         :@-  :@=    *.                     .
    //.                        @        @.  %                      .
    //.                 =*:  -#          +*#.                      .
    //.           %@@@%    .@*            =%                       .
    //.                                                            .
    //.                                                            .
    SSS_MATH_API inline float easeInBounce(const float x)
    {
        return 1.f - easeOutBounce(1.f - x);
    }


    //  ----------------- EASE IN OUT BOUNCE -----------------------
    //.                                                            .   
    //.                                                            .
    //.                                    =-    .--=-             .
    //.                                   -. =:.=:                 .
    //.                                   -                        .
    //.                                  =                         .
    //.                                 :.                         .
    //.                                ::                          .
    //.                               -:                           .
    //.                           .=-.                             .
    //.                          -                                 .
    //.                         -                                  .
    //.                        -.                                  .
    //.                       .:                                   .
    //.                  .    =                                    .
    //.                -.  -::.                                    .
    //.         .---.==     .=                                     .
    //.                                                            .
    //.                                                            .
    SSS_MATH_API inline float easeInOutBounce(const float x)
    {
        return x < 0.5
            ? (1 - easeOutBounce(1.f - 2.f * x)) / 2.f
            : (1 + easeOutBounce(2.f * x - 1)) / 2.f;
    }

}