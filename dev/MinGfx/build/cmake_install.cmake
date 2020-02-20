# Install script for directory: /Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xCoreLibx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/MinGfx-1.0" TYPE STATIC_LIBRARY FILES "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/libMinGfx.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/MinGfx-1.0/libMinGfx.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/MinGfx-1.0/libMinGfx.a")
    execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/MinGfx-1.0/libMinGfx.a")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xCoreLibx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/MinGfx-1.0" TYPE FILE FILES
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/aabb.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/bvh.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/color.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/craft_cam.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/default_shader.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/gfxmath.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/graphics_app.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/matrix4.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/mesh.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/mingfx.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/mingfx_config.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/opengl_headers.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/platform.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/point2.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/point3.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/quaternion.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/quick_shapes.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/ray.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shader_program.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/text_shader.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/texture2d.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/unicam.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/vector2.h"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/vector3.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xCoreLibx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/MinGfx-1.0/shaders" TYPE FILE FILES
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shaders/default.frag"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shaders/default.vert"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shaders/fullscreen.frag"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shaders/fullscreen.vert"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shaders/text.frag"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/src/shaders/text.vert"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/MinGfx-1.0/data" TYPE DIRECTORY FILES "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/data/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0/MinGfxTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0/MinGfxTargets.cmake"
         "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/CMakeFiles/Export/lib/cmake/MinGfx-1.0/MinGfxTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0/MinGfxTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0/MinGfxTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0" TYPE FILE FILES "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/CMakeFiles/Export/lib/cmake/MinGfx-1.0/MinGfxTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0" TYPE FILE FILES "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/CMakeFiles/Export/lib/cmake/MinGfx-1.0/MinGfxTargets-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/MinGfx-1.0" TYPE FILE FILES
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/cmake/MinGfxConfig.cmake"
    "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/cmake/MinGfxConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/tests/blank_window/cmake_install.cmake")
  include("/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/tests/gui_plus_opengl/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/Users/admin/Desktop/study/Fall 2019/CSCI 4611/repo-liu00222/dev/MinGfx/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
