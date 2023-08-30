cmake_minimum_required(VERSION 3.14)
project(findpeaks
	DESCRIPTION "Library to find peaks in 1D or 2D datasets using persistent homology"
	VERSION 0.0.1
)

set(CMAKE_CXX_STANDARD 20)

option(FINDPEAKS_COMPILE_EXAMPLE "Compile example code" OFF)



add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ADD_LIBRARY(UNION_FIND
	STATIC
	src/union_find.cpp
)

target_include_directories(UNION_FIND PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE UNION_FIND)

include(FetchContent)
FetchContent_Declare(delabella
	URL			https://github.com/msokalski/delabella/archive/df6bc55b3d78b09a43f0ae85d36499d7d9d783f8.tar.gz
	URL_HASH	SHA256=3128c73caafc081c3638acb72523cee3a5f9f810a9f048645b3f92e9c3a5e079
)
FetchContent_MakeAvailable(delabella)

target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_BINARY_DIR}/_deps/delabella-src/)

ADD_LIBRARY(DELA
	STATIC
	${CMAKE_BINARY_DIR}/_deps/delabella-src/delabella.cpp
)

target_link_libraries(${PROJECT_NAME} INTERFACE DELA)


IF(FINDPEAKS_COMPILE_EXAMPLE)

	find_library(NAMES hdf5 REQUIRED)
	include(ExternalProject)
	ExternalProject_Add(highfive
		URL			https://github.com/BlueBrain/HighFive/archive/refs/tags/v2.4.0.tar.gz
		URL_HASH	SHA256=ba0ed6d8e2e09e80849926f38c15a26cf4b80772084cea0555269a25fec02149
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
		INSTALL_COMMAND ""
	)
	add_definitions(-DHIGHFIVE_USE_BOOST=OFF)

	add_executable(findpeak_example example/main.cpp)
	target_include_directories(findpeak_example PRIVATE ${CMAKE_BINARY_DIR}/highfive-prefix/src/highfive/include)
	target_compile_options(findpeak_example PRIVATE -O3)
	target_link_libraries(findpeak_example findpeaks)

	target_link_libraries(findpeak_example  hdf5)
ENDIF()


add_executable(YOUR_TARGET example/main.cpp)
target_include_directories(YOUR_TARGET PRIVATE ${findpeaks_SOURCE_DIR}/include)
target_link_libraries(YOUR_TARGET findpeaks)





