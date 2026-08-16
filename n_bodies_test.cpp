#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "n_bodies.hpp"

#include <cstdio>
#include <fstream>

#include "doctest.h"

TEST_CASE("Costruction of a Body non valid") {
  pf::Body b{1., pf::TDvec{0., 0.}, pf::TDvec{0., 0.}, pf::TDvec{0., 0.}};
  CHECK(b.m() == 1.);
}

TEST_CASE("Body with mass not positive trows an exception") {
  CHECK_THROWS_AS((pf::Body{-1., pf::TDvec{0., 0.}, pf::TDvec{0., 0.},
                             pf::TDvec{0., 0.}}),
                  std::invalid_argument);
}

TEST_CASE("Body with velocity superluminale lancia un'eccezione") {
  CHECK_THROWS_AS((pf::Body{1., pf::TDvec{0., 0.}, pf::TDvec{3e8, 0.},
                             pf::TDvec{0., 0.}}),
                  std::invalid_argument);
}

TEST_CASE("Due corpi identici si attraggono con accelerazioni opposte") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::TDvec{0., 0.}, pf::TDvec{0., 0.}, pf::TDvec{0., 0.}},
      pf::Body{1., pf::TDvec{1., 0.}, pf::TDvec{0., 0.}, pf::TDvec{0., 0.}}};

  pf::computeAccelerations(bodies);

  // Il corpo 0 e' attratto verso il corpo 1 (accelerazione positiva su x)
  CHECK(bodies[0].a.x > 0.);
  // Per il terzo principio della dinamica le accelerazioni sono opposte
  // (a meno del rapporto delle masse, qui uguali)
  CHECK(bodies[0].a.x == doctest::Approx(-bodies[1].a.x));
}

TEST_CASE("isEnergyConserved rileva correttamente entro ed oltre la tolleranza") {
  CHECK(pf::isEnergyConserved(-100., -100.5, 0.01) == true);
  CHECK(pf::isEnergyConserved(-100., -120., 0.01) == false);
}

TEST_CASE("readBodiesFromFile legge correttamente un file valido") {
  std::string const filename{"test_input_tmp.txt"};
  {
    std::ofstream out{filename};
    out << "1. 0. 0. 0. 0.\n";
    out << "2. 1. 0. 0. 1.\n";
  }

  auto bodies = pf::readBodiesFromFile(filename);

  REQUIRE(bodies.size() == 2);
  CHECK(bodies[0].m() == doctest::Approx(1.));
  CHECK(bodies[1].m() == doctest::Approx(2.));
  CHECK(bodies[1].r.x == doctest::Approx(1.));
  CHECK(bodies[1].v.y == doctest::Approx(1.));

  std::remove(filename.c_str());
}

TEST_CASE("readBodiesFromFile lancia un'eccezione se il file non esiste") {
  CHECK_THROWS_AS(pf::readBodiesFromFile("file_che_non_esiste.txt"),
                  std::runtime_error);
}

TEST_CASE("computeMomentum calcola correttamente la quantita' di moto totale") {
  std::vector<pf::Body> bodies{
      pf::Body{2., pf::TDvec{0., 0.}, pf::TDvec{1., 0.}, pf::TDvec{0., 0.}},
      pf::Body{3., pf::TDvec{0., 0.}, pf::TDvec{0., 2.}, pf::TDvec{0., 0.}}};

  pf::TDvec P = pf::computeMomentum(bodies);
  CHECK(P.x == doctest::Approx(2.));  // 2*1 + 3*0
  CHECK(P.y == doctest::Approx(6.));  // 2*0 + 3*2
}

TEST_CASE("computeAngularMomentum calcola correttamente il momento angolare") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::TDvec{1., 0.}, pf::TDvec{0., 1.}, pf::TDvec{0., 0.}}};

  // L = m*(x*vy - y*vx) = 1*(1*1 - 0*0) = 1
  CHECK(pf::computeAngularMomentum(bodies) == doctest::Approx(1.));
}

TEST_CASE("Momento e momento angolare restano conservati durante la simulazione") {
  double const m{1. / pf::G};
  std::vector<pf::Body> bodies{
      pf::Body{m, pf::TDvec{-0.97000436, 0.24308753},
               pf::TDvec{0.4662036850, 0.4323657300}, pf::TDvec{0., 0.}},
      pf::Body{m, pf::TDvec{0.97000436, -0.24308753},
               pf::TDvec{0.4662036850, 0.4323657300}, pf::TDvec{0., 0.}},
      pf::Body{m, pf::TDvec{0., 0.}, pf::TDvec{-0.93240737, -0.86473146},
               pf::TDvec{0., 0.}}};

  pf::computeAccelerations(bodies);
  pf::TDvec const P0{pf::computeMomentum(bodies)};
  double const L0{pf::computeAngularMomentum(bodies)};

  for (int i{0}; i < 100; ++i) {
    pf::step(bodies);
  }

  CHECK(pf::isMomentumConserved(bodies, P0, 0.01) == true);
  CHECK(pf::isAngularMomentumConserved(bodies, L0, 0.01) == true);
}

TEST_CASE("step conserva approssimativamente l'energia su pochi passi") {
  double const m{1. / pf::G};
  std::vector<pf::Body> bodies{
      pf::Body{m, pf::TDvec{-0.97000436, 0.24308753},
               pf::TDvec{0.4662036850, 0.4323657300}, pf::TDvec{0., 0.}},
      pf::Body{m, pf::TDvec{0.97000436, -0.24308753},
               pf::TDvec{0.4662036850, 0.4323657300}, pf::TDvec{0., 0.}},
      pf::Body{m, pf::TDvec{0., 0.}, pf::TDvec{-0.93240737, -0.86473146},
               pf::TDvec{0., 0.}}};

  pf::computeAccelerations(bodies);
  double const E0{pf::computeEnergy(bodies)};

  for (int i{0}; i < 100; ++i) {
    pf::step(bodies);
  }

  double const E{pf::computeEnergy(bodies)};
  CHECK(pf::isEnergyConserved(E0, E, 0.01) == true);
}
