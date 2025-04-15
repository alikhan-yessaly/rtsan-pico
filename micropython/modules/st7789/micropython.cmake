# Define the module name and uppercase variant
set(MOD_NAME st7789)
string(TOUPPER ${MOD_NAME} MOD_NAME_UPPER)

# Create an INTERFACE library for our user C module
add_library(usermod_${MOD_NAME} INTERFACE)

# Add source files
target_sources(usermod_${MOD_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/st7789.c
    ${CMAKE_CURRENT_LIST_DIR}/mpfile.c
    ${CMAKE_CURRENT_LIST_DIR}/jpg/tjpgd565.c
    ${CMAKE_CURRENT_LIST_DIR}/png/pngle.c
    ${CMAKE_CURRENT_LIST_DIR}/png/miniz.c
)

# Add include directories
target_include_directories(usermod_${MOD_NAME} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/jpg
    ${CMAKE_CURRENT_LIST_DIR}/png
)

# Optional: Add compile-time macros if needed (example)
target_compile_definitions(usermod_${MOD_NAME} INTERFACE
    MODULE_${MOD_NAME_UPPER}_ENABLED=1
)

# Link this module to the global usermod target
target_link_libraries(usermod INTERFACE usermod_${MOD_NAME})
