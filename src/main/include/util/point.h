#pragma once

class Point {
    public:
        Point(float x, float y, float z) : x(x), y(y), z(z) {}
        Point(){ // default constructor
            x = 0;
            y = 0;
            z = 0;
        } // default constructor

        float x;
        float y;
        float z;
};