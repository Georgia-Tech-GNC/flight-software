
########################################### Ground Station ###########################################
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ground-station)
target_link_libraries(${CMAKE_PROJECT_NAME} ground_station_core)