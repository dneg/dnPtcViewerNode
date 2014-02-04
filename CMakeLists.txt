
cmake_minimum_required( VERSION 2.6 )

find_package ( Boost )

include_directories ( ${Boost_INCLUDE_DIR} )

add_subdirectory( icons )
add_subdirectory( src )

