# - Try to find PHOTOREALISTC RENDERMAN
# Once done this will define
#
#  PRMan_FOUND - system has PRMan
#  PRMan_INCLUDE_DIR - the PRMan include directory
#  PRMan_LIBRARY - Link this to use PRMan

# Copyright (c) 2008, Moritz Moeller, <cmake@virtualritz.com>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


IF( PRMan_INCLUDE_DIR AND PRMan_LIBRARY )
  SET( PRMan_FIND_QUIETLY TRUE )
ENDIF( PRMan_INCLUDE_DIR AND PRMan_LIBRARY )

FIND_PATH( PRMan_INCLUDE_DIR ri.h $ENV{RMANTREE}/include )

FIND_LIBRARY( PRMan_LIBRARY NAMES prman PATHS $ENV{RMANTREE}/lib )

# handle the QUIETLY and REQUIRED arguments and set PRMan_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( FindPackageHandleStandardArgs )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( PRMan DEFAULT_MSG PRMan_LIBRARY PRMan_INCLUDE_DIR )

MARK_AS_ADVANCED( PRMan_INCLUDE_DIR PRMan_LIBRARY )
