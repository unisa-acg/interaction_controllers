cmake_minimum_required(VERSION 2.8.3)

project(comau_description)

find_package(catkin REQUIRED)

catkin_package( CATKIN_DEPENDS xacro )

foreach(dir meshes robot)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
