#pragma once

class Polar {
    public:
        Polar(float radian, float radius) : radian(radian), magnitude(magnitude) {}
        Polar(){ // default constructor
            radian = 0;
            magnitude = 0;
        } // default constructor

        float radian;
        float magnitude;
};