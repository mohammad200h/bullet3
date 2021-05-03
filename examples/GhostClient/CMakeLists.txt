set(CMAKE_BUILD_TYPE Debug)

INCLUDE_DIRECTORIES( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
		${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
  )


SET(GhostTest_SRCS
	GhostTestmain.cpp
	../RobotSimulator/b3RobotSimulatorClientAPI.cpp
	../RobotSimulator/b3RobotSimulatorClientAPI.h	
	client.cpp
	client.h
	../../examples/ExampleBrowser/InProcessExampleBrowser.cpp
	../SharedMemory/GraphicsServerExample.cpp
	../SharedMemory/GraphicsClientExample.cpp
	../SharedMemory/RemoteGUIHelper.cpp
        ../SharedMemory/RemoteGUIHelperTCP.cpp
	../SharedMemory/GraphicsServerExample.h
	../SharedMemory/GraphicsClientExample.h
	../SharedMemory/RemoteGUIHelper.h
	../SharedMemory/GraphicsSharedMemoryCommands.h
	../SharedMemory/GraphicsSharedMemoryPublic.h
	../../examples/SharedMemory/PhysicsServerExample.cpp
	../../examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	../../examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)	

IF(BUILD_CLSOCKET)
 ADD_DEFINITIONS(-DBT_ENABLE_CLSOCKET)
 ADD_DEFINITIONS(${OSDEF})
ENDIF(BUILD_CLSOCKET)


#some code to support OpenGL and Glew cross platform
IF (WIN32)
	INCLUDE_DIRECTORIES(
			${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
	)
	ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
	IF(APPLE)
		find_library(COCOA NAMES Cocoa)
		MESSAGE(${COCOA})
		link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

	ELSE(APPLE)
		INCLUDE_DIRECTORIES(
				${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
		)
		ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
		ADD_DEFINITIONS("-DGLEW_STATIC")
		ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

		LINK_LIBRARIES(  pthread ${DL} )
	ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(App_GhostClient  ${GhostTest_SRCS})

SET_TARGET_PROPERTIES(App_GhostClient PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(App_GhostClient PROPERTIES DEBUG_POSTFIX "_d")
SET_TARGET_PROPERTIES(App_GhostClient PROPERTIES COMPILE_DEFINITIONS "B3_USE_ROBOTSIM_GUI")


TARGET_LINK_LIBRARIES(App_GhostClient BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)








