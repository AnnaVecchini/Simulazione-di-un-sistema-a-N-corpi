#include <vector>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <numeric>

namespace constant {
    const double c_light{300000};
    const double G{6.67E-11};
    const double dt{0.001};
}

struct Vec2D {
    double x{};
    double y{};

    Vec2D& operator+=(Vec2D const& b) {
        x += b.x;
        y += b.y;
        return *this;
    }
};

Vec2D operator+(Vec2D a, Vec2D const& b) {
    a += b;
    return a;
}

Vec2D operator-(Vec2D const& a, Vec2D const& b) {
    return {a.x - b.x, a.y - b.y};
}

Vec2D operator*(Vec2D const& a, double h) {
    return {a.x*h,a.y*h};
}

Vec2D operator*(double h, Vec2D const& a) {
    return {a.x*h,a.y*h};
}

double norm2 (Vec2D const& a) {
    return a.x*a.x + a.y*a.y;
}

double norm (Vec2D const& a) {
    return sqrt(norm2(a));
}

class Body {
    double m_;
    Vec2D r_;
    Vec2D v_;
    Vec2D a_;

    void validate() {
        if (m_<=0.) {
            throw std::runtime_error{"mass must be strictly positive"};
        }
        if (norm(v_) > constant::c_light) {
            throw std::runtime_error{"can not exceed the speed of light"};
        }
    }
 
  public:
    Body(double mass, Vec2D pos, Vec2D vel) : m_{mass}, r_{pos}, v_{vel} {
        validate();
    }

    Body(double mass, double rx, double ry, double vx, double vy) : Body(mass, Vec2D{rx,ry}, Vec2D{vx,vy}) {}

    double get_m() const {return m_;}
    const Vec2D& get_r() const {return r_;}
    const Vec2D& get_v() const {return v_;}
    const Vec2D& get_a() const {return a_;}

    void set_a(Vec2D const& new_a) {a_= new_a;}

    void update_position(double dt) {
        r_ += v_*dt + 0.5*a_*dt*dt;
    }

    void update_velocity(Vec2D const& old_a, double dt) {
        v_ += 0.5*(a_+old_a)*dt;
        validate();
    }

    void increment_a(Vec2D const& add_a) {
        a_ += add_a;
    }
};

double calculate_kinetic_energy(Body const& b) {
        return 0.5*b.get_m()*norm2(b.get_v());
}

Vec2D calculate_momentum(Body const& b) {
    return b.get_m()*b.get_v();
}

double calculate_angular_momentum(Body const& b) {
    Vec2D r = b.get_r();
    Vec2D v = b.get_v();
    return b.get_m()*(r.x*v.y - r.y*v.x);
}

/*da fare: lettura condizioni iniziali da un file di dati e creazione del vettore di bodies
std::vector<Body>*/

class Simulation {
    std::vector<Body> bodies_;
    int n_steps_;
    const double dt_{0.001};
    const double eps_{1E-12};

    void validate() {
        if (bodies_.size()<2) {
            throw std::runtime_error{"not enough bodies to run a simulation"};
        }
        if(n_steps_<0) {
            throw std::runtime_error{"steps must be positive"};
        }
    }

    void calculate_acceleration() {
        std::size_t N = bodies_.size();
        double eps2 = eps_*eps_;
        for (auto &b : bodies_) {
            b.set_a({0.,0.});
        }
        for (std::size_t i{}; i<N-1; ++i) {
            auto & bi = bodies_[i];
            for (std::size_t j{i+1}; j<N; ++j) {
                auto & bj = bodies_[j];
                Vec2D diff = bj.get_r()-bi.get_r();
                double k = norm2(diff)+eps2;
                double factor = constant::G/(k*std::sqrt(k));
                bi.increment_a(factor*bj.get_m()*diff);
                bj.increment_a(-1.*factor*bi.get_m()*diff);
            }
        }
    }

    void step() {
        std::size_t N = bodies_.size();
        std::vector<Vec2D> old_accs(N);
        for (std::size_t i{}; i<N; ++i) {
            bodies_[i].update_position(dt_);
            old_accs[i]=bodies_[i].get_a();
        }
        calculate_acceleration();
        for (std::size_t i{}; i<N; ++i) {
            bodies_[i].update_velocity(old_accs[i],dt_);
        } 

    }

  public:
    Simulation(std::vector<Body> v, int n) : bodies_{std::move(v)}, n_steps_{n} {
        validate();
        calculate_acceleration(); //non sono sicuro!
    }

    const std::vector<Body>& get_bodies() const {return bodies_;}

    void run() {
        for (int i{}; i<n_steps_; ++i) {
        step();
        }
    }
};

double total_kinetic_energy(std::vector<Body> const& bodies) {
    return std::accumulate(bodies.begin(), bodies.end(),0.,
                        [](double acc, Body const& b){return acc + calculate_kinetic_energy(b);});
}

double total_potential_energy(std::vector<Body> const& bodies) {
    double U{};
    std::size_t N = bodies.size();
    for (std::size_t i{}; i<N-1; ++i) {
        auto const& bi = bodies[i];
        for (std::size_t j{i+1}; j<N; ++j) {
            auto const& bj= bodies[j];
            U -= constant::G*bi.get_m()*bj.get_m()/norm(bi.get_r()-bj.get_r());
        }
    }
    return U;
}

double mechanical_energy(std::vector<Body> const& bodies) {
    return total_kinetic_energy(bodies) + total_potential_energy(bodies);
}

Vec2D total_momentum(std::vector<Body> const& bodies) {
    return std::accumulate(bodies.begin(), bodies.end(), Vec2D{0.,0.}, 
                        [](Vec2D acc, Body const& b){return acc + calculate_momentum(b);});
}

double total_angular_momentum(std::vector<Body> const& bodies) {
    return std::accumulate(bodies.begin(), bodies.end(),0.,
                        [](double acc, Body const& b){return acc + calculate_angular_momentum(b);});
}

bool is_conserved(double i_value, double f_value, double tol = 0.01) {
    double abs_diff = std::abs(f_value-i_value);
    double abs_i = std::abs(i_value);
    if (abs_i < 1E-12) {
        return abs_diff < tol; 
    }
    return abs_diff < abs_i*tol;
}

bool is_conserved(Vec2D const& i, Vec2D const& f, double tol = 0.01) {
    double norm_diff = norm(f - i);
    double norm_i = norm(i);
    if (norm_i < 1E-12) {
        return norm_diff < tol;
    }
    return norm_diff < tol*norm_i;
}







int main() {
    Body b1{10.,{1.,2.},{2,3}};
    Body b2{4.,6.,3.,4.,6.};

    Body b3{5., b1.get_r()+b2.get_r(),b1.get_v()-b2.get_v()*2};
    std::cout << b3.get_r().x << b3.get_r().y << b3.get_a().x << b3.get_a().y << '\n';

}

/*VECCHIE FUNZIONI (implementazioni precedenti):

double total_kinetic_energy(std::vector<Body> const& bodies) {
    double K{};
    for (auto const& b : bodies) {
        K += calculate_kinetic_energy(b);
    }
    return K;
}

double total_potential_energy(std::vector<Body> const& bodies) {
    double U{};
    std::size_t N = bodies.size();
    for (std::size_t i{}; i<N-1; ++i) {
        auto const& bi = bodies[i];
        for (std::size_t j{i+1}; j<N; ++j) {
            auto const& bj= bodies[j];
            U -= constant::G*bi.get_m()*bj.get_m()/norm(bi.get_r()-bj.get_r());
        }
    }
    return U;
}

double mechanical_energy(std::vector<Body> const& bodies) {
    return total_kinetic_energy(bodies) + total_potential_energy(bodies);
}

Vec2D total_momentum(std::vector<Body> const& bodies) {
    Vec2D P{0.,0.};
    for (auto const& b : bodies) {
        P += calculate_momentum(b);
    }
    return P;
}

double total_angular_momentum(std::vector<Body> const& bodies) {
    double L{};
    for (auto const& b : bodies) {
        L += calculate_angular_momentum(b);
    }
    return L;
}

ENERGIA POTENZIALE CON ALGORITMI (troppo complicato e intrecciato):

double total_potential_energy(std::vector<Body> const& bodies) {
    double U{};
    for (auto it=bodies.begin(); it<bodies.end(); ++it) {
        U += std::accumulate(it+1, bodies.end(),0., 
                                [&](double acc){
                                    auto next_it=it+1;
                                    return acc + constant::G*(it->get_m())*(next_it->get_m())/norm(it->get_r()-next_it->get_r());});
    }
    return U;
}

double total_potential_energy(std::vector<Body> const& bodies) {
    auto i = bodies.begin();
    return std::accumulate(i, bodies.end()-1, 0.,
                        [&](double acc_i){
                            auto j = i+1;
                            return acc_i + std::accumulate(j, bodies.end(), 0.,
                                                [&](double acc_j){
                                                    return acc_j + constant::G*(i->get_m())*(j->get_m())/norm(i->get_r()-j->get_r());});
                                                });
}*/