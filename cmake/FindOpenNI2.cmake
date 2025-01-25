# Copyright 2019-2021, Collabora, Ltd.
#
# SPDX-License-Identifier: BSL-1.0
#
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)
#
# Original Author:
# 2021 Moses Turner <moses@collabora.com>


set(OpenNI2_ROOT_DIR
    "${OpenNI2_ROOT_DIR}"
    CACHE PATH "Root to search for OpenNI2")

find_path(
    OpenNI2_INCLUDE_DIR
    NAMES openni2/OpenNI.h
    PATHS ${OpenNI2_ROOT_DIR}
    PATH_SUFFIXES include)

find_library(
    OpenNI2_LIBRARY
    NAMES libOpenNI2.so
    PATHS ${OpenNI2_ROOT_DIR}
    PATH_SUFFIXES lib lib/x64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenNI2 REQUIRED_VARS OpenNI2_INCLUDE_DIR OpenNI2_LIBRARY)

if(OpenNI2_FOUND)
    set(OpenNI2_INCLUDE_DIRS "${OpenNI2_INCLUDE_DIR}/openni2")
    set(OpenNI2_LIBRARIES "${OpenNI2_LIBRARY}")
    if(NOT TARGET OpenNI2::OpenNI2)
        add_library(OpenNI2::OpenNI2 UNKNOWN IMPORTED)
    endif()
    set_target_properties(
        OpenNI2::OpenNI2 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                  "${OpenNI2_INCLUDE_DIR}/openni2")
    message(STATUS "Found OpenNI2: ${OpenNI2_LIBRARY}")
    set_target_properties(OpenNI2::OpenNI2 PROPERTIES IMPORTED_LOCATION
                                                    "${OpenNI2_LIBRARY}")
    mark_as_advanced(OpenNI2_INCLUDE_DIR OpenNI2_LIBRARY)
endif()
mark_as_advanced(OpenNI2_ROOT_DIR)