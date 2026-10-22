^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ik_solver_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-22)
------------------
* Fix build type: CMAKE_BUILD_TYPE was hard-coded to the non-existent "DebWithRelInfo", so no
  optimization flags were applied. The package now defaults to Release (-O3 -DNDEBUG) and honours
  -DCMAKE_BUILD_TYPE. Rebuild with colcon build --cmake-clean-cache.
* test_service_timing.py: add --parallelize {default,force,disable} to set the parallelize field of
  the IK array requests. Several modes can be given to compare them in one run; results and CSV
  have a new mode column. Previously the field was never set, so the node parameter decided
  (single-thread with all shipped configs).
* test_service_timing.py: add --threads to set num_threads of the IK array requests. Every
  combination of --parallelize and --threads is run; results and CSV have a new threads column.
* Contributors: Federico Parma, Manuel Beschi, Nicola Pedrocchi, Roberto Fausti
