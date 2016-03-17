set(SOURCES
  urdf.cpp
)
set(HEADERS
  ${CMAKE_SOURCE_DIR}/include/mc_rbdyn_urdf/api.h
  ${CMAKE_SOURCE_DIR}/include/mc_rbdyn_urdf/urdf.h
)

add_library(mc_rbdyn_urdf SHARED ${SOURCES} ${HEADERS})
PKG_CONFIG_USE_DEPENDENCY(mc_rbdyn_urdf SpaceVecAlg)
PKG_CONFIG_USE_DEPENDENCY(mc_rbdyn_urdf RBDyn)
PKG_CONFIG_USE_DEPENDENCY(mc_rbdyn_urdf tinyxml2)

set_target_properties(mc_rbdyn_urdf PROPERTIES SOVERSION 1 VERSION 1.1.0)
target_compile_definitions(mc_rbdyn_urdf PRIVATE "-DMCRBDYNURDF_BUILDING")

set(INSTALL_PATH include/mc_rbdyn_urdf)
install(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
install(TARGETS mc_rbdyn_urdf
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
