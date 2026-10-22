^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ik_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-22)
------------------
* Fix build type forced to Debug: CMAKE_BUILD_TYPE was hard-coded in CMakeLists.txt, overriding
  -DCMAKE_BUILD_TYPE. The code was optimized only when cnr_logger was installed as Release (it exports
  -Ofast to its consumers), otherwise it compiled at -O0. The package now defaults to Release
  (-O3 -DNDEBUG) and honours -DCMAKE_BUILD_TYPE. On an affected machine, get_ik_array with 1000
  targets went from ~11 s to sub-second. Rebuild with colcon build --cmake-clean-cache.
* Fix get_ik_array ignoring the update_recursively_seeds parameter: the raw request field was passed
  instead of the resolved value (request, falling back to the parameter), and the single-thread
  solver always propagated seeds. UPDATE_RECURSIVELY_SEEDS_DISABLE is now honoured.
* Rewrite computeIKArrayMT for lower overhead: workers take pose indices from an atomic counter and
  write results directly (no queue, promises or futures), progress is printed at most every 100 ms
  instead of once per pose, and only min(MAX_NUM_PARALLEL_IK_SOLVER, N) threads are started.
  Output is unchanged.
* Add num_threads to get_ik_array and get_task_redundant_ik_array requests to limit the number of
  worker threads used in parallel mode (0 = all, capped at MAX_NUM_PARALLEL_IK_SOLVER). The final
  progress line now reports the number of threads used. Requires ik_solver_msgs >= 0.1.0.
* A solver exception in a multi-thread IK array worker no longer terminates the node; the pose is
  reported as failed and logged.
* Contributors: Federico Parma, Manuel Beschi, Nicola Pedrocchi, Roberto Fausti
