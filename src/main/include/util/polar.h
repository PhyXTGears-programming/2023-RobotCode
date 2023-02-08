#pragma once

class Polar {
    public:
        Polar(float radian, float radius) : radian(radian), magnitude(magnitude) {}
        Polar(){
            radian = 0;
            magnitude = 0;
        }

        float radian;
        float magnitude;
};