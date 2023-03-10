#pragma once

class Bezier {
    public:
        Bezier(float x_a, float y_a, float x_b, float y_b, float x_c, float y_c, float x_d, float y_d) : 
                x_a(x_a), y_a(y_a), x_b(x_b), y_b(y_b), x_c(x_c), y_c(y_c), x_d(x_d), y_d(y_d) {}

        Bezier(){
            x_a = 0;
            y_a = 0;
            x_b = 0;
            y_b = 0;
            x_c = 0;
            y_c = 0;
            x_d = 0;
            y_d = 0;
        }

        float x_a;
        float y_a;
        float x_b;
        float y_b;
        float x_c;
        float y_c;
        float x_d;
        float y_d;
};