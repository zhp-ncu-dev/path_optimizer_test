cmake_minimum_required(VERSION 3.0)
project(path_optimizer_test)

set(CMAKE_CXX_STANDARD 17)
set(TARGET_NAME ${CMAKE_PROJECT_NAME}${SUFFIX_STR})

# find packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(IPOPT REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(PythonLibs 2.7)

# executable 
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})                                   # 当前路径是一个目录吗，是的话就加入到包含目录
        # message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)           # 获得当前目录下的所有文件，让如ALL_SUB列表中
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
        endif()
    endforeach()
endfunction()
include_sub_directories_recursively(${CMAKE_CURRENT_SOURCE_DIR})            # 对子目录递归调用，包含

file(GLOB_RECURSE SRC_FILES
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
add_executable(${TARGET_NAME} ${SRC_FILES})
target_include_directories(${TARGET_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} 
                      glog 
                      gflags 
                      ${IPOPT_LIBRARIES} 
                      OsqpEigen::OsqpEigen 
                      osqp::osqp
                      ${PYTHON_LIBRARIES}
                     )
