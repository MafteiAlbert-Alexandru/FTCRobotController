cmake_minimum_required(VERSION 3.4.1)

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
        TeamCode

        # Sets the library as a shared library.
        SHARED

        # Provides a relative path to your source file(s).
        src/main/cpp/pipeline.cpp)

############################################################
# You must clone OpenCv-Repackaged into the same parent    #
# directory as this project in order to be able to compile #
# the native code. (Or, you can edit these paths)          #
############################################################

# Specifies a path to native header files.
include_directories(src/main/cpp/include/
        ${CMAKE_SOURCE_DIR}/../doc/native_headers/)

# Specify path to prebuilt object
add_library(openCvNative SHARED IMPORTED)
set_target_properties(openCvNative PROPERTIES IMPORTED_LOCATION
        #../doc/native_libs/${ANDROID_ABI}/libOpenCvAndroid453.so)
        ${CMAKE_SOURCE_DIR}/../doc/native_libs/${ANDROID_ABI}/libOpenCvAndroid453.so)

# linking libs
target_link_libraries(
        TeamCode
        openCvNative)