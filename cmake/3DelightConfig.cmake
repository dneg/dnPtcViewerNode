# - Try to find 3DELIGHT
# Once done this will define
#
#  3Delight_FOUND - system has 3Delight
#  3Delight_INCLUDE_DIR - the 3Delight include directory
#  3Delight_LIBRARY - Link this to use 3Delight

# Copyright (c) 2008, Moritz Moeller, <cmake@virtualritz.com>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


IF( 3Delight_INCLUDE_DIR AND 3Delight_LIBRARY )
  SET( 3Delight_FIND_QUIETLY TRUE )
ENDIF( 3Delight_INCLUDE_DIR AND 3Delight_LIBRARY )

FIND_PATH( 3Delight_INCLUDE_DIR ri.h $ENV{DELIGHT}/include )

FIND_LIBRARY( 3Delight_LIBRARY NAMES 3delight PATHS $ENV{DELIGHT}/lib )

# handle the QUIETLY and REQUIRED arguments and set 3Delight_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( 3Delight DEFAULT_MSG 3Delight_LIBRARY 3Delight_INCLUDE_DIR )

MARK_AS_ADVANCED( 3Delight_INCLUDE_DIR 3Delight_LIBRARY )

