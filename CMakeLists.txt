cmake_minimum_required(VERSION 3.20)

if(IDF_PROJECT)

    idf_component_register(
        SRCS src/bq25672.c
        INCLUDE_DIRS include
    )

else()

    project(BQ25672 
        LANGUAGES C 
        VERSION 0.1
        DESCRIPTION "A platform agnostic driver for the BQ25672 Battery Charger IC")

    add_library(${PROJECT_NAME} STATIC src/bq25672.c)
    target_include_directories(${PROJECT_NAME} PUBLIC include)

endif()