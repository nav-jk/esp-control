include($ENV{IDF_PATH}/tools/cmake/version.cmake)


set(srcs "src/led_strip_api.c")
set(public_requires)


idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES ${public_requires})