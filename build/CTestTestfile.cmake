# CMake generated Testfile for 
# Source directory: /home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi
# Build directory: /home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[n_body.t]=] "/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/build/Debug/n_body.t")
  set_tests_properties([=[n_body.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/CMakeLists.txt;60;add_test;/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[n_body.t]=] "/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/build/Release/n_body.t")
  set_tests_properties([=[n_body.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/CMakeLists.txt;60;add_test;/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[n_body.t]=] "/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/build/RelWithDebInfo/n_body.t")
  set_tests_properties([=[n_body.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/CMakeLists.txt;60;add_test;/home/micheleresca/n_bodies/Simulazione-di-un-sistema-a-N-corpi/CMakeLists.txt;0;")
else()
  add_test([=[n_body.t]=] NOT_AVAILABLE)
endif()
