#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(SOURCES
  urdf.cpp
)
set(HEADERS
  ../include/mc_rbdyn_urdf/api.h
  ../include/mc_rbdyn_urdf/urdf.h
)

add_library(mc_rbdyn_urdf SHARED ${SOURCES} ${HEADERS})
target_include_directories(mc_rbdyn_urdf PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include> $<INSTALL_INTERFACE:include>)
target_link_libraries(mc_rbdyn_urdf PUBLIC RBDyn::RBDyn PRIVATE TinyXML2::TinyXML2)
set_target_properties(mc_rbdyn_urdf PROPERTIES SOVERSION 1 VERSION 1.1.0)
target_compile_definitions(mc_rbdyn_urdf PRIVATE "-DMCRBDYNURDF_BUILDING")

install(FILES ${HEADERS} DESTINATION include/mc_rbdyn_urdf)
install(
  TARGETS mc_rbdyn_urdf
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "bin"
)
