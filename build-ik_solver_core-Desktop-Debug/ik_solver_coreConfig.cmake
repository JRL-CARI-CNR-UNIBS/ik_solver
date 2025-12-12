
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ik_solver_coreConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

get_filename_component(ik_solver_core_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

include(CMakeFindDependencyMacro)
find_dependency(cnr_param       REQUIRED)
find_dependency(cnr_logger      REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED urdfdom IMPORTED_TARGET)
find_dependency(Eigen3          REQUIRED NO_MODULE)

# Temporary - cnr_logger
find_dependency(Boost           REQUIRED COMPONENTS date_time filesystem)

include("${ik_solver_core_CMAKE_DIR}/export_ik_solver_core.cmake")

set(ik_solver_core_FOUND TRUE)

set_and_check(ik_solver_core_INCLUDE_DIRS "/usr/local/include")
  
set(ik_solver_core_LIBRARIES ik_solver_core::ik_solver_core ik_solver_core::ik_solver_core_utilities)

check_required_components(ik_solver_core)
