# When adding new source files to core, make sure to add those files to this list
set(CORE_SOURCES
    src/main.c
    src/lsm6dso.c
)

# When adding new include's directories to core, make sure to add those files to this list 
set(CORE_INCLUDES
    include/
)

target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${CORE_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CORE_INCLUDES})

# Add all items in $CORE_SOURCES to the list of warning-enabled files
list(TRANSFORM CORE_SOURCES PREPEND "${CMAKE_SOURCE_DIR}/core/")
list(APPEND STRICT_WARNINGS_WHITELIST ${CORE_SOURCES})
set(STRICT_WARNINGS_WHITELIST "${STRICT_WARNINGS_WHITELIST}" PARENT_SCOPE)