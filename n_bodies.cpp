#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

const double G{6.67e-11};
const double ϵ{10e-12};
const double Δt{0.001};

struct TDvec {
  double x;
  double y;
};

TDvec operator+(TDvec const& a, TDvec const& b) {
  return {a.x + b.x, a.y + b.y};
}

TDvec operator-(TDvec const& a, TDvec const& b) {
  return {a.x - b.x, a.y - b.y};
}

TDvec operator*(TDvec const& a, double s) {
  return {a.x * s, a.y * s};
}

double norm(TDvec const& a) {
  return sqrt(a.x * a.x + a.y * a.y);
}

class Body {
 private:
  double m_;  
  TDvec r_;   
  TDvec v_;
  TDvec a_;

 public:
  Body(double m, TDvec r, TDvec v, TDvec a);

  // Getter: permettono di leggere i membri privati dall'esterno
  // della classe (es. da computeAccelerations o da main).
  // Sono const perché non modificano l'oggetto su cui sono chiamati.
  double m() const { return m_; }
  TDvec r() const { return r_; }
  TDvec v() const { return v_; }
  TDvec a() const { return a_; }

  // Setter: permette di modificare l'accelerazione dall'esterno.
  // Non è const, perché il suo scopo è proprio cambiare lo stato
  // dell'oggetto.
  void set_a(TDvec a) { a_ = a; }
};

// Definizione del costruttore, fuori dalla classe.
// La sintassi ": m_{m}, r_{r}, v_{v}, a_{a}" è la member
// initialization list
Body::Body(double m, TDvec r, TDvec v, TDvec a)
    : m_{m}, r_{r}, v_{v}, a_{a} {
  if (m_ <= 0) {
    throw std::invalid_argument("La massa deve essere positiva");
  }
  if (norm(v_) >= 299792458.) {
    throw std::invalid_argument("La velocita deve essere minore di quella della luce");
  }
}

// Prende bodies per riferimento (non const) perché deve
// modificare l'accelerazione di ciascun corpo tramite set_a().
void computeAccelerations(std::vector<Body>& bodies) {
  int N = bodies.size();

  for (int i{0}; i < N; ++i) {
    TDvec a_i{0, 0};  // accumulatore per l'accelerazione del corpo i

    for (int j{0}; j < N; ++j) {
      if (j == i) continue;  // un corpo non esercita forza su se stesso

      // r_i - r_j
      TDvec diff = bodies[i].r() - bodies[j].r();

      // |r_i - r_j|^2, calcolato a mano per evitare una sqrt inutile
      double dist2 = diff.x * diff.x + diff.y * diff.y;

      // (|r_i - r_j|^2 + eps^2)^(3/2)
      double denom = std::pow(dist2 + ϵ * ϵ, 1.5);

      // Accumula il contributo del corpo j, con il segno meno
      // previsto dalla formula
      a_i = a_i - diff * (G * bodies[j].m() / denom);
    }

    bodies[i].set_a(a_i);
  }
}
