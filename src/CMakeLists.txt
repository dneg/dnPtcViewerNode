########### install locations ###########
if( "$ENV{MAYA_PLUGIN_DIR}" STREQUAL "" )

    set( MAYA_PLUGIN_DIR ${CMAKE_INSTALL_PREFIX}/bin/plug-ins )

endif()


if( "$ENV{MAYA_SCRIPT_DIR}" STREQUAL "" )
    set( MAYA_SCRIPT_DIR ${CMAKE_INSTALL_PREFIX}/scripts/others )

endif()


if( CMAKE_SYSTEM_NAME STREQUAL "Darwin" )        # MacOS

    add_definitions( -Dfoo ) # TODO
    
elseif( CMAKE_SYSTEM_NAME STREQUAL "Linux" )     # Linux

    add_definitions( 
        -DBits64_ -D_BOOL -DLINUX -DFUNCPROTO -D_GNU_SOURCE -DLINUX_64
        -DREQUIRE_IOSTREAM -DUNIX -m64 -fPIC -fno-strict-aliasing -O3 
        -funsigned-char -ftemplate-depth-25 -pthread
        -Wno-deprecated -Wall -Wno-multichar -Wno-comment
        -Wno-sign-compare -Wno-reorder -fno-gnu-keywords )
    set( CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined" )
    
elseif( CMAKE_SYSTEM_NAME STREQUAL "Windows" )   # Windows

    add_definitions( -Dfoo ) # TODO

endif()


#find_package( OpenGL REQUIRED ) # seems to work w/o this
# libGLU is provided by Maya

set( Maya_DIR ../cmake )
find_package( Maya REQUIRED )

if( RMAN STREQUAL "3delight" )

    set( 3Delight_DIR ../cmake )
    find_package( 3Delight REQUIRED )
    set( RMAN_LIBRARY ${3Delight_LIBRARY} )
    set( RMAN_INCLUDE_DIR ${3Delight_INCLUDE_DIR} )

#TODO: Pixie

else() # Default: PRMan

    set( PRMan_DIR ../cmake )
    find_package( PRMan REQUIRED )
    set( RMAN_LIBRARY ${PRMan_LIBRARY} )
    set( RMAN_INCLUDE_DIR ${PRMan_INCLUDE_DIR} )

endif()


include_directories( 
    ${Maya_INCLUDE_DIR}
    ${RMAN_INCLUDE_DIR} )

########### next target #################

set( dnPtcViewerNode_LIB_SRCS
    dnPtcViewerNode.cpp )


add_library( dnPtcViewerNode MODULE ${dnPtcViewerNode_LIB_SRCS} )
# omit "lib" prefix from output
set_target_properties( dnPtcViewerNode PROPERTIES PREFIX "" ) 

target_link_libraries( dnPtcViewerNode ${Maya_LIBRARIES} ${RMAN_LIBRARY} )
    
########### install files ###############

install( TARGETS dnPtcViewerNode
    LIBRARY
        DESTINATION bin/plug-ins )

install( FILES AEdnPtcViewerNodeTemplate.mel
    DESTINATION scripts/others )
