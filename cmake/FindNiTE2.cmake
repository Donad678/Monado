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


set(NiTE2_ROOT_DIR
    "${NiTE2_ROOT_DIR}"
    CACHE PATH "Root to search for NiTE2")

find_path(
    NiTE2_INCLUDE_DIR
    NAMES NiTE.h
    PATHS ${NiTE2_ROOT_DIR}
    PATH_SUFFIXES include/nite)

find_library(
    NiTE2_LIBRARY
    NAMES NiTE2
    PATHS ${NiTE2_ROOT_DIR}
    PATH_SUFFIXES lib lib/x64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NiTE2 REQUIRED_VARS NiTE2_INCLUDE_DIR
                                                       NiTE2_LIBRARY)
if(NiTE2_FOUND)
    set(NiTE2_INCLUDE_DIRS "${NiTE2_INCLUDE_DIR}")
    set(NiTE2_LIBRARIES "${NiTE2_LIBRARY}")
    if(NOT TARGET NiTE2::NiTE2)
        add_library(NiTE2::NiTE2 UNKNOWN IMPORTED)
    endif()
    set_target_properties(
        NiTE2::NiTE2 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                  "${NiTE2_INCLUDE_DIR}")
    set_target_properties(NiTE2::NiTE2 PROPERTIES IMPORTED_LOCATION
                                                    "${NiTE2_LIBRARY}")
    mark_as_advanced(NiTE2_INCLUDE_DIR NiTE2_LIBRARY)
endif()
mark_as_advanced(NiTE2_ROOT_DIR)
