cmake_minimum_required(VERSION 3.8)
project(ik_solver_core VERSION 0.1.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(IK_SOLVER_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(IK_SOLVER_YAML_CPP_TARGET yaml-cpp)
endif()

find_package(PkgConfig REQUIRED) # temporary for urdfdom
pkg_check_modules(urdfdom REQUIRED urdfdom IMPORTED_TARGET)

###########
## Build ##
###########

# Utilities
add_library(ik_solver_core_utilities SHARED
  src/${PROJECT_NAME}/internal/utils.cpp
  src/${PROJECT_NAME}/params.cpp
  include/${PROJECT_NAME}/logger.h
  include/${PROJECT_NAME}/params.h
  include/${PROJECT_NAME}/internal/SafeQueue.h
  include/${PROJECT_NAME}/internal/types.h
  include/${PROJECT_NAME}/internal/utils.h
  include/${PROJECT_NAME}/internal/wsq.h
  )

# Core
add_library(ik_solver_core INTERFACE
  include/${PROJECT_NAME}/ik_solver_base_class.h
  include/${PROJECT_NAME}/internal/ik_solver_base_class_impl.h
  )

# Utils
target_link_libraries(ik_solver_core_utilities PUBLIC
  Eigen3::Eigen
  ${IK_SOLVER_YAML_CPP_TARGET}
)
add_library(ik_solver_core::ik_solver_core_utilities ALIAS ik_solver_core_utilities)
# Core
target_link_libraries(ik_solver_core INTERFACE
  ik_solver_core::ik_solver_core_utilities
  PkgConfig::urdfdom
  Eigen3::Eigen
)
add_library(ik_solver_core::ik_solver_core ALIAS ik_solver_core)

target_include_directories(${PROJECT_NAME}_utilities PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

#############
## Install ##
#############
  set(CONFIG_DEST share/cmake/${PROJECT_NAME})
  set(EXPORT_DEST ${CONFIG_DEST})
  set(CONFIG_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include) # for config
  set(CONFIG_FILENAME "ik_solver_coreConfig")

  install(TARGETS ik_solver_core ik_solver_core_utilities
          EXPORT export_ik_solver_core
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)

  install(EXPORT export_ik_solver_core
          FILE export_ik_solver_core.cmake
          NAMESPACE ik_solver_core::
          DESTINATION ${EXPORT_DEST})

  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
          DESTINATION include)


  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})


  # Config
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    ${CONFIG_FILENAME}.cmake.in ${CONFIG_FILENAME}.cmake
    INSTALL_DESTINATION ${CONFIG_DEST}
  )
  write_basic_package_version_file(${CONFIG_FILENAME}Version.cmake
      VERSION ${PACKAGE_VERSION}
      COMPATIBILITY AnyNewerVersion)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_FILENAME}.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_FILENAME}Version.cmake"
          DESTINATION ${CONFIG_DEST}
  )


  # make uninstall
  # add_custom_target("uninstall" COMMENT "Uninstall installed files")
  # add_custom_command(
  #     TARGET "uninstall"
  #     POST_BUILD
  #     COMMENT "Uninstall files with install_manifest.txt"
  #     COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
  #             install_manifest.txt to be uninstalled!
  # )
