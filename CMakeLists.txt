cmake_minimum_required(VERSION 3.28)
project(progettopf VERSION 0.1.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

# esporta il compilation database, utile per altri strumenti, a partire dall'editor
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++20 (volendo anche C++23), senza estensioni non-standard offerte dal compilatore usato
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS
      " -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
      " -Wshadow -Wimplicit-fallthrough -Wextra-semi -Wold-style-cast"
      " -fno-omit-frame-pointer")

# abilita asserzioni della standard library
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_ASSERTIONS")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  string(APPEND CMAKE_CXX_FLAGS " -D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_EXTENSIVE")
endif()

# abilita address sanitizer e undefined-behaviour sanitizer in Debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address,undefined")
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  string(APPEND CMAKE_CXX_FLAGS " -D_GLIBCXX_SANITIZE_STD_ALLOCATOR")
endif()
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined")

# richiede il componente graphics della libreria SFML (versione 2.6 in Ubuntu 24.04)
find_package(SFML 2.6 COMPONENTS graphics REQUIRED)

# eseguibile principale: la simulazione N-Body
add_executable(n_body main.cpp n_bodies.cpp)

# programma di prova per verificare il collegamento a SFML: apre solo una
# finestra vuota. Servira' da base quando integreremo la grafica in n_body.
add_executable(sfml_test sfml_test.cpp)
target_link_libraries(sfml_test PRIVATE sfml-graphics)

# copia il file delle condizioni iniziali accanto all'eseguibile n_body dopo
# la build, cosi' il percorso relativo usato in main.cpp per aprirlo funziona
# indipendentemente da dove viene lanciata la build (rilevante soprattutto
# con un generator multi-config come Ninja Multi-Config, che mette gli
# eseguibili in build/Debug/ o build/Release/, non in build/)
add_custom_command(TARGET n_body POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
          ${CMAKE_CURRENT_SOURCE_DIR}/initial_conditions.txt
          $<TARGET_FILE_DIR:n_body>
)

# il testing e' abilitato di default
# per disabilitarlo, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)

  # aggiungi l'eseguibile n_body.t
  add_executable(n_body.t n_bodies.test.cpp n_bodies.cpp)
  # aggiungi l'eseguibile n_body.t alla lista dei test
  add_test(NAME n_body.t COMMAND n_body.t)

endif()
