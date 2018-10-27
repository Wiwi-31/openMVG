# ==============================================================================
# Generic CMake File for QTBased project
# ==============================================================================

# The name of my project.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(ProjectName openMVG_control_points_registration_with_chessboad)
PROJECT(${ProjectName})

# CMake version control
#CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# ==============================================================================
# Libraries setup
# ==============================================================================
# Find and setup OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

if(OpenCV_FOUND)

  # ==============================================================================
  # List all source files here.
  # ==============================================================================

  # All Source Files here
  SET(SourceFiles
  main.cpp
  AutomaticRegistration.cpp)

  # All non moc header files here
  SET(HeaderFiles
  AutomaticRegistration.hpp)

  # ==============================================================================
  # Add the following files, modules and libraries to the project.
  # ==============================================================================
  # Files
  ADD_EXECUTABLE(${ProjectName}
  ${SourceFiles}
  ${HeaderFiles})

  # Linking with Libraries.
  TARGET_LINK_LIBRARIES(${ProjectName}
  ${OpenCV_LIBS}
  openMVG_sfm
  ${STLPLUS_LIBRARY})

  SET_PROPERTY(TARGET ${ProjectName} PROPERTY FOLDER OpenMVG/software/ui)
  INSTALL(TARGETS ${ProjectName} DESTINATION bin/)
endif(OpenCV_FOUND)
