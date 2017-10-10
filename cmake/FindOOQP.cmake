# TRY TO FIND THE INCLUDE DIRECTORY
find_path(OOQP_INCLUDE_DIR
  QpGenData.h
  HINTS 
    $ENV{OOQP}/include/ooqp 
    /usr/local/include/ooqp/ 
    ${PROJECT_SOURCE_DIR}/../../libs/thirdPartyCode/OOQP/include
)

# BLAS
find_library(BLAS_LIBRARIES
  NAMES 
    blas
  HINTS 
    /usr/local/libs/ 
    ${CMAKE_SOURCE_DIR}/../libs/thirdPartyCode/CLAPACK/BLAS/Release
)
if(BLAS_LIBRARIES)
  message(STATUS "Found BLAS libraries:" ${BLAS_LIBRARIES})
else()
  message(FATAL_ERROR "Could not find BLAS libraries.")
endif()

# F2CLIBS
find_library(F2CLIBS_I77
  NAMES 
    libI77
  HINTS 
    /usr/local/libs/
    ${CMAKE_SOURCE_DIR}/../libs/thirdPartyCode/CLAPACK/F2CLIBS/ReleaseI77
)
find_library(F2CLIBS_F77
  NAMES 
    libF77
  HINTS 
    /usr/local/libs/
    ${CMAKE_SOURCE_DIR}/../libs/thirdPartyCode/CLAPACK/F2CLIBS/ReleaseF77
)
set(F2CLIBS_LIBRARIES ${F2CLIBS_I77} ${F2CLIBS_F77})
if(F2CLIBS_LIBRARIES)
  message(STATUS "Found F2CLIBS libraries:" ${F2CLIBS_LIBRARIES})
else()
  message(FATAL_ERROR "Could not find F2CLIBS libraries.")
endif()

# CLAPACK
find_library(CLAPACK_LIBRARIES
  NAMES 
    clapack
  HINTS 
    /usr/local/libs/ 
    ${CMAKE_SOURCE_DIR}/../libs/thirdPartyCode/CLAPACK/Release
)
if(CLAPACK_LIBRARIES)
  message(STATUS "Found CLAPACK libraries:" ${CLAPACK_LIBRARIES})
else()
  message(FATAL_ERROR "Could not find CLAPACK libraries.")
endif()

set(OOQP_LIBRARIES ${BLAS_LIBRARIES} ${F2CLIBS_LIBRARIES} ${CLAPACK_LIBRARIES})

# find_package(BLAS REQUIRED)
################################

# Dependent packages, BLAS and HSL
# set(OOQP_LIBRARIES)
# find_package(BLAS REQUIRED)
# if(BLAS_FOUND)
#   message("BLAS FOUND")
#   set(OOQP_LIBRARIES ${OOQP_LIBRARIES} ${BLAS_LIBRARIES})
# else()
#   message(STATUS "OOQP requires BLAS")
# endif()
# find_package(HSL QUIET)
# if(HSL_FOUND)
#   set(OOQP_LIBRARIES ${OOQP_LIBRARIES} ${HSL_LIBRARIES})
# else()
#   message(STATUS "OOQP requires HSL")
# endif()

if(OOQP_INCLUDE_DIR)
  set(OOQP_FOUND_INCLUDE TRUE)
  set(OOQP_INCLUDE_DIRS
  ${OOQP_INCLUDE_DIR})
  message(STATUS "Found OOQP include dirs: ${OOQP_INCLUDE_DIRS}")
else()
  message(STATUS "Could not find OOQP include dir")
endif()

# TRY TO FIND THE LIBRARIES
set(OOQP_LIBS_LIST
  ooqpgensparse ooqpsparse ooqpgondzio ooqpbase MA27
)

set(OOQP_FOUND_LIBS TRUE)
foreach(LIB ${OOQP_LIBS_LIST})
  
  if(WIN32)
    set(LIB lib${LIB}.lib)
  endif()

  find_library(OOQP_LIB_${LIB}
    NAMES ${LIB}
    HINTS /usr/local/libs/ ${PROJECT_SOURCE_DIR}/../../libs/thirdPartyCode/OOQP/lib/
  )
  if(OOQP_LIB_${LIB})
    set(OOQP_LIBRARIES ${OOQP_LIBRARIES} ${OOQP_LIB_${LIB}})
  else()
    message(FATAL_ERROR "Could not find " ${LIB})
    set(OOQP_FOUND_LIBS FALSE)
  endif()
endforeach()

# print OOQP_LIBRARIES
if(OOQP_FOUND_LIBS)
  message(STATUS "Found OOQP libraries: ${OOQP_LIBRARIES}")
else()
  message(STATUS "Cound not find OOQP libraries")
endif()

# SUCCESS if BOTH THE LIBRARIES AND THE INCLUDE DIRECTORIES WERE FOUND
if(OOQP_FOUND_INCLUDE AND OOQP_FOUND_LIBS AND BLAS_FOUND AND HSL_FOUND)
  set(OOQP_FOUND TRUE)
  message(STATUS "Found OOQP")
elseif()
  message(STATUS "Cound not find OOQP")
endif()
