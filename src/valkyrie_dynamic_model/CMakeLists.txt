FILE(GLOB_RECURSE hpp_headers *.hpp)
FILE(GLOB_RECURSE h_headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(Val_model SHARED ${sources} ${hpp_headers} ${h_headers})
target_link_libraries(Val_model  ${SJUtils} ${SJurdf} ${SJrbdl})
