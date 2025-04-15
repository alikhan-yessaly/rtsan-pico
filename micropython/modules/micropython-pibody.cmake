# Make sure the parent path is searchable for includes
include_directories(${CMAKE_CURRENT_LIST_DIR}/../../)

# Optional: Extend module search paths
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../")

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# === Include only what you need ===

# Your custom ST7789 driver module
include(st7789/micropython)

# If you had these later:
# include(picographics/micropython)
include(pwm/micropython)
include(servo/micropython)

# Optional compile flags or preprocessor defines
# target_compile_definitions(usermod_st7789 INTERFACE -DSOME_FLAG=1)

# Optional: disable C++ exceptions for size
# include(micropython-disable-exceptions)
