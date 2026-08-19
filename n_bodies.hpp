#ifndef PF_N_BODIES_HPP
#define PF_N_BODIES_HPP

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pf {

// Costanti fisiche e parametri della simulazione
inline constexpr double G{6.67e-11};
inline constexpr double eps{10e-12};
inline constexpr double dt{0.001};
inline constexpr double c_light{299792458.};

// Istante di tempo corrente della simulazione (inline variable, C++17):
// un'unica variabile condivisa da tutte le translation unit che
// includono questo header.
inline double t{0.};

// Vettore bidimensionale: usato per posizione, velocita' e accelerazione
struct TDvec {
  double x;
  double y;
};

inline TDvec operator+(TDvec const& a, TDvec const& b) {
  return {a.x + b.x, a.y + b.y};
}

inline TDvec operator-(TDvec const& a, TDvec const& b) {
  return {a.x - b.x, a.y - b.y};
}

inline TDvec operator*(TDvec const& a, double s) {
  return {a.x * s, a.y * s};
}

inline TDvec operator*(double s, TDvec const& a) {
  return {s * a.x, s * a.y};
}

inline double norm(TDvec const& a) {
  return std::sqrt(a.x * a.x + a.y * a.y);
}

// Rappresenta un singolo corpo del sistema gravitazionale.
// La massa resta privata (senza setter) perche' non deve poter
// cambiare dopo la costruzione. Posizione, velocita' e accelerazione
// sono pubbliche: un getter/setter triviale non aggiungerebbe nulla.
class Body {
  double m_;

 public:
  TDvec r;
  TDvec v;
  TDvec a;

  Body(double mass, TDvec pos, TDvec vel, TDvec acc);

  double m() const { return m_; }
};

// Legge le condizioni iniziali dei corpi da un file di testo.
// Il file deve contenere, per ogni corpo, cinque numeri (separati da
// spazi o a capo, in un ordine qualsiasi rispetto alle righe):
// massa, posizione x, posizione y, velocita' x, velocita' y.
// L'accelerazione iniziale viene impostata a {0,0} e va poi calcolata
// con computeAccelerations prima del primo step.
// Lancia std::runtime_error se il file non puo' essere aperto.
std::vector<Body> readBodiesFromFile(std::string const& filename);

// Calcola l'accelerazione di ogni corpo dovuta all'attrazione
// gravitazionale di tutti gli altri, con softening
void computeAccelerations(std::vector<Body>& bodies);

// Esegue un singolo passo di integrazione Velocity Verlet.
// Richiede che bodies[i].a contenga gia' l'accelerazione corrente
// a(t) di ciascun corpo (va quindi chiamata computeAccelerations
// almeno una volta prima del primo step).
void step(std::vector<Body>& bodies);

// Calcola l'energia cinetica totale del sistema: somma di 1/2*m_i*v_i^2
double computeKineticEnergy(std::vector<Body> const& bodies);

// Calcola l'energia potenziale gravitazionale totale del sistema
// (somma sulle coppie i<j, per non contarle due volte)
double computePotentialEnergy(std::vector<Body> const& bodies);

// Calcola l'energia meccanica totale (cinetica + potenziale) del sistema
double computeEnergy(std::vector<Body> const& bodies);

// Verifica se l'energia attuale E e' rimasta entro una tolleranza
// relativa "tolerance" rispetto all'energia iniziale E0
bool isEnergyConserved(double E0, double E, double tolerance);

// Quantita' di moto totale del sistema: somma di m_i * v_i
TDvec computeMomentum(std::vector<Body> const& bodies);

// Momento angolare totale del sistema (in 2D ha una sola componente,
// perpendicolare al piano): somma di m_i*(x_i*vy_i - y_i*vx_i)
double computeAngularMomentum(std::vector<Body> const& bodies);

// Verifica se la quantita' di moto e' rimasta conservata entro una
// tolleranza relativa. La tolleranza e' relativa a una "scala tipica"
// del sistema (somma di m_i*|v_i|) e non al valore iniziale P0,
// perche' P0 puo' essere il vettore nullo (come nel caso Figure-8),
// nel qual caso una tolleranza relativa a P0 sarebbe sempre zero.
bool isMomentumConserved(std::vector<Body> const& bodies, TDvec const& P0,
                          double tolerance);

// Stessa idea di isMomentumConserved, applicata al momento angolare
bool isAngularMomentumConserved(std::vector<Body> const& bodies, double L0,
                                 double tolerance);

} 

#endif