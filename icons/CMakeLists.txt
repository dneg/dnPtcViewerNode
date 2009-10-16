########### install locations ###########

if( "$ENV{MAYA_ICON_DIR}" STREQUAL "" )

    set( MAYA_ICON_DIR ${CMAKE_INSTALL_PREFIX}/icons )

endif()

    
########### install files ###############

set( dnPtcViewerNode_LIB_ICONS
    out_dnPtcViewerNode.xpm )

install( FILES ${dnPtcViewerNode_LIB_ICONS}
    DESTINATION ${MAYA_ICON_DIR} )
