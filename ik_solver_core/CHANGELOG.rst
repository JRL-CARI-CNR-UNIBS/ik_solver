^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ik_solver_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add IkSolverBase::setBounds(lb, ub): set the joint limits, clamped to the URDF limits; returns
  false and leaves the limits unchanged on wrong sizes or lower > upper. config() restores the
  limits of the parameters.
* config() can be called again (e.g. by reconfigure): the logger is initialized only the first time.
* Rename the second argument of computeIk() from T_world_base to T_base_world in the declaration
  and fix its documentation: it is the pose of the world in the base frame (the implementation
  already used it that way; no behaviour change).

0.1.0 (2026-10-22)
------------------
* Fix build type forced to Debug: CMAKE_BUILD_TYPE was hard-coded in CMakeLists.txt, overriding
  -DCMAKE_BUILD_TYPE. The code was optimized only when cnr_logger was installed as Release (it exports
  -Ofast to its consumers), otherwise it compiled at -O0. The package now defaults to Release
  (-O3 -DNDEBUG) and honours -DCMAKE_BUILD_TYPE. Rebuild with colcon build --cmake-clean-cache.
* Contributors: Federico Parma, Manuel Beschi, Nicola Pedrocchi, Roberto Fausti
