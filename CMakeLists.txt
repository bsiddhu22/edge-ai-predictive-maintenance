cmake_minimum_required(VERSION 3.16)
project(pdm_node C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_options(/W4 /D_CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra -Wshadow -Werror)
endif()

# ---- Firmware: portable C, static allocation, talks to hardware only via hal.h ----
add_library(pdm_fw STATIC
  firmware/src/accel.c
  firmware/src/acq.c
  firmware/src/app.c
  firmware/src/dsp.c
  firmware/src/health.c
  firmware/src/model_data.c
  firmware/src/nn.c
  firmware/src/perf.c
  firmware/src/telemetry.c
)
target_include_directories(pdm_fw PUBLIC firmware/include)
if(NOT MSVC)
  target_compile_options(pdm_fw PRIVATE -Wdouble-promotion)   # single-precision FPU math only
  target_link_libraries(pdm_fw PUBLIC m)
endif()

# ---- Simulation pieces shared by the SIL runner and the data generator ----
add_library(pdm_simcore STATIC sim/motor_model.c sim/adxl_emu.c)
target_include_directories(pdm_simcore PUBLIC sim firmware/include)
if(NOT MSVC)
  target_link_libraries(pdm_simcore PUBLIC m)
endif()

add_executable(pdm_sim sim/sim_main.c sim/sim_hal.c)
target_link_libraries(pdm_sim PRIVATE pdm_fw pdm_simcore)

add_executable(pdm_datagen sim/datagen.c)
target_link_libraries(pdm_datagen PRIVATE pdm_simcore)

# ---- Unit tests (incl. golden-vector parity with the Python pipeline) ----
enable_testing()
add_executable(unit_tests tests/unit/test_main.c tests/unit/test_hal.c)
target_link_libraries(unit_tests PRIVATE pdm_fw pdm_simcore)
target_compile_definitions(unit_tests PRIVATE GOLDEN_PATH="${CMAKE_SOURCE_DIR}/tests/golden/golden.bin")
add_test(NAME unit_tests COMMAND unit_tests)
