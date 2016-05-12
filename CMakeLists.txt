cmake_minimum_required (VERSION 2.8)

cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0028 NEW)


project (feintrack)

FIND_PACKAGE(OpenCV 3.0 REQUIRED)


if (CMAKE_COMPILER_IS_GNUCXX)
#    find_package(Qt5Widgets)
#    find_package(Qt5OpenGL)
#    find_package(Qt5Test)
#    find_package(Qt5Concurrent)
endif()


unset(CMAKE_CXX_FLAGS CACHE)


find_package(OpenMP)
if (OPENMP_FOUND)
    list(APPEND CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    list(APPEND CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


if (CMAKE_COMPILER_IS_GNUCXX)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall --std=c++0x -g --coverage" CACHE STRING COMPILE_FLAGS FORCE)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall --std=c++0x -g -pg" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic-errors --std=c++0x" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native --fast-math -ffast-math -funroll-loops -Wall -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -march=native -mtune=native -Wall -DDEBUG" CACHE STRING COMPILE_FLAGS FORCE)
elseif (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /W4" CACHE STRING COMPILE_FLAGS FORCE)
endif()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.7 -stdlib=libc++ -Wall" CACHE STRING COMPILE_FLAGS FORCE)
ENDIF()

include_directories(${OpenCV_INCLUDE_DIRS})

option(USE_GPU "Should the GPU be used?" OFF)
if(USE_GPU)
    add_definitions(-DUSE_GPU)
endif(USE_GPU)


# Setup CUDA
if(USE_GPU)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS} ${CUDA_CUT_INCLUDE_DIR})
endif(USE_GPU)

# ----------------------------------------------------------------------
set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/src/bgrnd_substr.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack_objects.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/recognition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/segmentation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
)

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bgrnd_substr.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack_objects.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/recognition.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/segmentation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/some_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack_params.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack_d.h
)

file(GLOB SrcGpuCpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/feintrack_d.cu
)

# ----------------------------------------------------------------------
if(USE_GPU)

# add GPU related source code to the executable list
list(APPEND SOURCE ${SrcGpuCpp})

# Compile CUDA stuff
cuda_include_directories(${local_CUDA_CUT_INCLUDE_DIRS})
cuda_include_directories(${CUDA_INCLUDE_DIRS} ${CUDA_CUT_INCLUDE_DIR} ${local_CUDA_CUT_INCLUDE_DIR})

link_directories(${local_CUDA_CUT_LIBRARY_DIRS})

#target_link_libraries(
#   ${CUDA_LIBRARIES}
#   ${cutil_LIB}
#)

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} --generate-line-info) # used during profiling
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch=sm_20)
 
endif(USE_GPU)

# ----------------------------------------------------------------------
list(REMOVE_ITEM OpenCV_LIBS "opencv_adas")

if (CMAKE_COMPILER_IS_GNUCXX)
set(LIBS
    ${OpenCV_LIBS}
#    ${QT_LIBRARIES}
)

else(CMAKE_COMPILER_IS_GNUCXX)
set(LIBS
    ${OpenCV_LIBS}
)
endif()

if(USE_GPU)
   #list(APPEND LIBS ${CUDA_LIBRARIES} ${cutil_LIB})

   #MESSAGE(${CUDA_CUDA_LIBRARY})
   #MESSAGE(${CUDA_CUDART_LIBRARY})
   list(APPEND LIBS ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
endif(USE_GPU)

if(USE_GPU)
   CUDA_ADD_EXECUTABLE(feintrack  ${SOURCE} ${HEADERS})
else()
   add_executable(feintrack ${SOURCE} ${HEADERS})
endif()

target_link_libraries(feintrack ${LIBS})

if(USE_GPU)
target_link_libraries(feintrack ${local_CUDA_LIB})
endif(USE_GPU)

