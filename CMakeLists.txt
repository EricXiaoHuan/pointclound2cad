cmake_minimum_required(VERSION 3.16)

project(PointCloud2CAD VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 设置编译选项
if(MSVC)
    add_compile_options(/W4 /WX /MP /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找依赖库
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters segmentation registration surface features)
find_package(CGAL 5.4 REQUIRED)
find_package(Qt5 5.15 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Assimp 5.1 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost 1.70 REQUIRED COMPONENTS filesystem system)

# 包含头文件目录
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${Qt5_INCLUDE_DIRS}
    ${Assimp_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src
)

# 链接库目录
link_directories(
    ${PCL_LIBRARY_DIRS}
    ${CGAL_LIBRARY_DIRS}
    ${Assimp_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
)

# 添加预定义宏
add_definitions(
    ${PCL_DEFINITIONS}
    ${CGAL_DEFINITIONS}
)

# 源文件目录
set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

# 收集源文件
file(GLOB_RECURSE SOURCES
    ${SOURCE_DIR}/*.cpp
    ${SOURCE_DIR}/*.h
    ${SOURCE_DIR}/*.hpp
)

# 收集UI文件
file(GLOB_RECURSE UI_FILES
    ${SOURCE_DIR}/*.ui
)

# 收集资源文件
file(GLOB_RECURSE RESOURCE_FILES
    ${SOURCE_DIR}/*.qrc
)

# 处理Qt UI文件
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# 处理Qt资源文件
qt5_add_resources(RESOURCE_OBJECTS ${RESOURCE_FILES})

# 创建可执行文件
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${UI_HEADERS}
    ${RESOURCE_OBJECTS}
)

# 链接依赖库
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${CGAL_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${Assimp_LIBRARIES}
    ${Boost_LIBRARIES}
)

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# 复制依赖库到输出目录
if(MSVC)
    # 复制PCL依赖库
    file(GLOB PCL_DLLS ${PCL_ROOT}/bin/*.dll)
    foreach(DLL ${PCL_DLLS})
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${DLL}
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
    endforeach()
    
    # 复制Qt依赖库
    foreach(DEPENDENCY Core Gui Widgets)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:Qt5::${DEPENDENCY}>
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
    endforeach()
endif()