# set target name (will be the name of the executable)
set(PLUGIN_TARGET idssePlugin)

add_library(${PLUGIN_TARGET} SHARED
    idssePlugin.cpp
    idsse.cpp
)

target_include_directories(${PLUGIN_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PLUGIN_TARGET} PRIVATE Boost::boost ezC2X::ezC2X)

# enable the strict compiler warnings on the library
target_compile_options(${PLUGIN_TARGET} PRIVATE "${DEFAULT_STRICT_COMPILER_WARNINGS}")

# make the project version available when compiling
target_compile_definitions(${PLUGIN_TARGET} PRIVATE "PLUGIN_VERSION=${PROJECT_VERSION}")

# disable lib prefix and extensions
set_target_properties(${PLUGIN_TARGET} PROPERTIES
    PREFIX ""
    CXX_EXTENSIONS OFF
)

# install plugin
install(
    TARGETS ${PLUGIN_TARGET}
    EXPORT idsseTargets
    COMPONENT idsse
    DESTINATION ${PLUGIN_DIR}
)
