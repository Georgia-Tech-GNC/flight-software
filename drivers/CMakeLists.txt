# When adding new source files to core, make sure to add those files to this list
set(DRIVER_SOURCES

)

# When adding new include's directories to core, make sure to add those files to this list 
set(DRIVER_INCLUDES

)

target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${DRIVER_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${DRIVER_INCLUDES})

# Add all items in $CORE_SOURCES to the list of warning-enabled files
list(TRANSFORM DRIVER_SOURCES PREPEND "${CMAKE_SOURCE_DIR}/drivers/")
list(APPEND STRICT_WARNINGS_WHITELIST ${DRIVER_SOURCES})
set(STRICT_WARNINGS_WHITELIST "${STRICT_WARNINGS_WHITELIST}" PARENT_SCOPE)