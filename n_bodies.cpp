#include <vector>
#include <iostream>
#include <cmath>

struct TDvec {
    double x;
    double y;
};

TDvec operator+ (TDvec const& a, TDvec const& b) {
    return {a.x + b.x, a.y + b.y};
}

TDvec operator- (TDvec const& a, TDvec const& b) {
    return {a.x - b.x, a.y - b.y};
}

double norm (TDvec const& a) {
    return sqrt(a.x*a.x + a.y*a.y);
}

class Body {
    double m_;
    TDvec r_;
    TDvec v_;
    TDvec a_;
};
