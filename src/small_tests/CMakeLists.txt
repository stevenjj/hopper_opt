#---------------------------------------------
# Test Interpolation
#---------------------------------------------
add_executable(test_interpolation test_interpolation.cpp)
target_link_libraries(test_interpolation  ${SJUtils})


add_subdirectory(test_val_model)