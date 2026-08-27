#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "n_bodies.hpp"

// ============================================================================
// 1. 2D ALGEBRA TESTS
// ============================================================================
TEST_CASE("Vec2D equality operator==") {
  pf::Vec2D u{3., 4.};
  pf::Vec2D v{1., -2.};

  CHECK(u == pf::Vec2D{3., 4.});
  CHECK_FALSE(u == v);
}

TEST_CASE("Vec2D addition operator+ and operator+=") {
  pf::Vec2D u{3., 4.};
  pf::Vec2D v{1., -2.};

  pf::Vec2D sum = u + v;
  CHECK(sum == pf::Vec2D{4., 2.});

  u += v;
  CHECK(u == pf::Vec2D{4., 2.});
}

TEST_CASE("Vec2D subtraction operator-") {
  pf::Vec2D u{3., 4.};
  pf::Vec2D v{1., -2.};

  pf::Vec2D diff = u - v;
  CHECK(diff == pf::Vec2D{2., 6.});
}

TEST_CASE("Vec2D scalar multiplication operator*") {
  pf::Vec2D u{3., 4.};

  pf::Vec2D scaled1 = u * 2.0;
  pf::Vec2D scaled2 = 2.0 * u;
  CHECK(scaled1 == pf::Vec2D{6., 8.});
  CHECK(scaled2 == pf::Vec2D{6., 8.});
}

TEST_CASE("Vec2D norm and norm2") {
  pf::Vec2D u{3., 4.};

  CHECK(pf::norm2(u) == doctest::Approx(25.0)); // 3^2 + 4^2
  CHECK(pf::norm(u) == doctest::Approx(5.0));   // sqrt(25)
}

// ============================================================================
// 2. CLASS BODY/ VALIDATION TESTS
// ============================================================================
TEST_CASE("Body construction with Vec2D") {
  pf::Body b{1., pf::Vec2D{1., -1.}, pf::Vec2D{2., 0.}};
  CHECK(b.get_m() == 1.);
  CHECK(b.get_r() == pf::Vec2D{1., -1.});
  CHECK(b.get_v() == pf::Vec2D{2., 0.});
  CHECK(b.get_a() == pf::Vec2D{0., 0.});
}

TEST_CASE("Body construction with scalars") {
  pf::Body b{1., 1., -1., 2., 0.};
  CHECK(b.get_m() == 1.);
  CHECK(b.get_r() == pf::Vec2D{1., -1.});
  CHECK(b.get_v() == pf::Vec2D{2., 0.});
  CHECK(b.get_a() == pf::Vec2D{0., 0.});
}

TEST_CASE("Body validation throws on negative mass") {
  CHECK_THROWS_AS((pf::Body{-1., pf::Vec2D{0., 0.}, pf::Vec2D{0., 0.}}),
                  std::runtime_error);
}

TEST_CASE("Body validation throws on superluminal speed") {
  CHECK_THROWS_AS((pf::Body{1., pf::Vec2D{0., 0.}, pf::Vec2D{3e8, 0.}}),
                  std::runtime_error);
}

TEST_CASE("Body kinematics update (Velocity-Verlet steps)") {
  pf::Body b{1., pf::Vec2D{0., 0.}, pf::Vec2D{10., 0.}};
  b.set_a(pf::Vec2D{2., 0.});
  double dt = 1.0;

  // r_new = r + v*dt + 0.5*a*dt^2 = (0,0) + (10,0)*1 + 0.5*(2,0)*1 = (11, 0)
  b.update_position(dt);
  CHECK(b.get_r() == pf::Vec2D{11., 0.});

  // v_new = v + 0.5*(a_old + a_new)*dt = (10,0) + 0.5*((2,0) + (4,0))*1 = (13,
  // 0)
  pf::Vec2D new_a{4., 0.};
  b.set_a(new_a);
  b.update_velocity(pf::Vec2D{2., 0.}, dt);
  CHECK(b.get_v() == pf::Vec2D{13., 0.});
}

// ============================================================================
// 3. ENERGY/ MOMENTUM/ ANGULAR MOMENTUM TESTS
// ============================================================================
TEST_CASE("Single body kinetic energy, momentum, and angular momentum") {
  pf::Body b1{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 1.}};

  CHECK(pf::calculate_kinetic_energy(b1) == doctest::Approx(0.5));
  CHECK(pf::calculate_momentum(b1) == pf::Vec2D{0., 1.});
  // L = m * (x*vy - y*vx) = 1 * (1*1 - 0*0) = 1
  CHECK(pf::calculate_angular_momentum(b1) == doctest::Approx(1.0));
}

TEST_CASE("Total energy equals kinetic plus potential energy") {
  pf::Body b1{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 1.}};
  pf::Body b2{2., pf::Vec2D{0., 0.}, pf::Vec2D{1., 0.}};
  std::vector<pf::Body> bodies{b1, b2};

  double const E{pf::total_energy(bodies)};
  CHECK(E == doctest::Approx(pf::total_kinetic_energy(bodies) +
                             pf::total_potential_energy(bodies)));
}

TEST_CASE("compute_totals aggregates all quantities correctly") {
  pf::Body b1{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 1.}};
  pf::Body b2{2., pf::Vec2D{0., 0.}, pf::Vec2D{1., 0.}};
  std::vector<pf::Body> bodies{b1, b2};

  pf::SystemTotals totals = pf::compute_totals(bodies);
  CHECK(totals.E == doctest::Approx(pf::total_energy(bodies)));
  CHECK(totals.P == pf::total_momentum(bodies));
  CHECK(totals.L == doctest::Approx(pf::total_angular_momentum(bodies)));
}

TEST_CASE("is_conserved tolerance logic for scalars and vectors") {
  CHECK(pf::is_conserved(100., 100.5) == true);
  CHECK(pf::is_conserved(100., 120.) == false);
  CHECK(pf::is_conserved(0., 1E-3) == true);
  CHECK(pf::is_conserved(0., 0.1) == false);

  CHECK(pf::is_conserved(pf::Vec2D{100., 150.}, pf::Vec2D{100.5, 150.5}) ==
        true);
  CHECK(pf::is_conserved(pf::Vec2D{100., 150.}, pf::Vec2D{105., 155.}) ==
        false);
  CHECK(pf::is_conserved(pf::Vec2D{0., 0.}, pf::Vec2D{1E-3, 1E-3}) == true);
  CHECK(pf::is_conserved(pf::Vec2D{0., 0.}, pf::Vec2D{0.5, 0.5}) == false);
}

// ============================================================================
// 4. FILE I/O TESTS
// ============================================================================
TEST_CASE("load_bodies_from_file functionality") {
  std::string const filename{"test_input_tmp.txt"};
  {
    std::ofstream out{filename};
    out << "1. 0. 0. 0. 0.\n";
    out << "2. 1. 0. 0. 1.\n";
  }

  auto bodies = pf::load_bodies_from_file(filename);

  REQUIRE(bodies.size() == 2);
  CHECK(bodies[0].get_m() == doctest::Approx(1.));
  CHECK(bodies[1].get_m() == doctest::Approx(2.));
  CHECK(bodies[1].get_r().x == doctest::Approx(1.));
  CHECK(bodies[1].get_v().y == doctest::Approx(1.));

  std::remove(filename.c_str());
}

TEST_CASE("load_bodies_from_file throws on missing file") {
  CHECK_THROWS_AS(pf::load_bodies_from_file("not_existing_file.txt"),
                  std::runtime_error);
}

// ============================================================================
// 5. SIMULATION/ CONSERVED QUANTITIES TEST
// ============================================================================
TEST_CASE("Simulation initialization throws if less than two bodies") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 1.}}};

  CHECK_THROWS_AS((pf::Simulation{bodies}), std::runtime_error);
}

TEST_CASE("Newton's Third Law (opposite accelerations)") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{0., 0.}, pf::Vec2D{0., 0.}},
      pf::Body{1., pf::Vec2D{1., 0.}, pf::Vec2D{0., 0.}}};

  pf::Simulation sim(std::move(bodies));

  CHECK(sim.get_bodies()[0].get_a().x > 0.);
  CHECK(sim.get_bodies()[0].get_a().x ==
        doctest::Approx(-sim.get_bodies()[1].get_a().x));
}

TEST_CASE("Energy, Momentum and Angular Momentum conserved at EVERY step") {
  std::vector<pf::Body> bodies{
      pf::Body{1., pf::Vec2D{-0.97000436, 0.24308753},
               pf::Vec2D{0.4662036850, 0.4323657300}},
      pf::Body{2., pf::Vec2D{0.97000436, -0.24308753},
               pf::Vec2D{0.4662036850, 0.4323657300}},
      pf::Body{3., pf::Vec2D{0., 0.}, pf::Vec2D{-0.93240737, -0.86473146}}};

  pf::SystemTotals initial = pf::compute_totals(bodies);
  pf::Simulation sim(std::move(bodies));

  for (int step = 0; step < 100; ++step) {
    sim.step();
    pf::SystemTotals current = pf::compute_totals(sim.get_bodies());

    CHECK_MESSAGE(pf::is_conserved(initial.E, current.E),
                  "Energy lost at step ", step + 1);
    CHECK_MESSAGE(pf::is_conserved(initial.P, current.P),
                  "Momentum lost at step ", step + 1);
    CHECK_MESSAGE(pf::is_conserved(initial.L, current.L),
                  "Angular momentum lost at step ", step + 1);
  }
}