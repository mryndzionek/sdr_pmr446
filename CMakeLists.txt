cmake_minimum_required(VERSION 3.16.3)

project(sdr_pmr446)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")

include_directories(include local/include
                    dependencies/dlg/include)
link_directories(local/lib)

set(SRCS src/logging.c dependencies/dlg/src/dlg/dlg.c)
set(LIBS m dl pthread SoapySDR liquid rtaudio)

add_compile_options(-Wno-deprecated-declarations
                    -Wno-unused-but-set-variable
                    -Wno-unused-variable
                    -Wall -Werror -fPIC)
add_compile_definitions(DLG_LOG_LEVEL=dlg_level_info)

add_executable(sdr_pmr446 src/sdr_pmr446.c
                          ${SRCS})
target_link_libraries(sdr_pmr446 ${LIBS})

add_executable(dsd_in src/dsd_in.c
                      ${SRCS})
target_link_libraries(dsd_in ${LIBS})


