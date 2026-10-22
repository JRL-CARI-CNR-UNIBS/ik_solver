^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ik_solver_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-22)
------------------
* Fix build type forced to Debug: CMAKE_BUILD_TYPE was hard-coded in CMakeLists.txt, overriding
  -DCMAKE_BUILD_TYPE. The code was optimized only when cnr_logger was installed as Release (it exports
  -Ofast to its consumers), otherwise it compiled at -O0. The package now defaults to Release
  (-O3 -DNDEBUG) and honours -DCMAKE_BUILD_TYPE. Rebuild with colcon build --cmake-clean-cache.
* Contributors: Federico Parma, Manuel Beschi, Nicola Pedrocchi, Roberto Fausti
