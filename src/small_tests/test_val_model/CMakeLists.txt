#---------------------------------------------
# Test Valkyrie Model
#---------------------------------------------
add_executable(test_val_model test_val_model.cpp)
target_link_libraries(test_val_model  Val_model ${SJUtils} ${SJurdf} ${SJrbdl})

