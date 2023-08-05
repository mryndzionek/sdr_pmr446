cmake_minimum_required(VERSION 3.16.3)

project(sdr_pmr446)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")

include_directories(include local/include
                    dependencies/dlg/include)
link_directories(local/lib)

set(SRCS src/logging.c src/shared.c
         dependencies/dlg/src/dlg/dlg.c)
set(LIBS m dl pthread SoapySDR liquid rtaudio)

add_compile_options(-Wno-deprecated-declarations
                    -Wall -Werror -fPIC)
add_compile_definitions(DLG_LOG_LEVEL=dlg_level_info)

add_executable(sdr_pmr446 src/sdr_pmr446.c
                          ${SRCS})
target_link_libraries(sdr_pmr446 ${LIBS})
target_compile_definitions(sdr_pmr446 PUBLIC APP_SDR_PMR446)

add_executable(dsd_in src/dsd_in.c
                      ${SRCS})
target_link_libraries(dsd_in ${LIBS})
target_compile_definitions(dsd_in PUBLIC APP_DSD_IN)


