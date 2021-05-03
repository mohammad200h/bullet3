set(CMAKE_BUILD_TYPE Debug)
SUBDIRS( HelloWorld BasicDemo)
IF(BUILD_BULLET3)
	SUBDIRS( GhostClient ExampleBrowser RobotSimulator SharedMemory ThirdPartyLibs/Gwen ThirdPartyLibs/BussIK ThirdPartyLibs/clsocket OpenGLWindow TwoJoint )
ENDIF()

IF(BUILD_PYBULLET)
	SUBDIRS(pybullet)
ENDIF(BUILD_PYBULLET)
