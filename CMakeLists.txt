# create an executable
ADD_EXECUTABLE (Planar_SingleContact
	${CS225A_COMMON_SOURCE}
	Planar_SingleContact.cpp
)

# and link the library against the executable
TARGET_LINK_LIBRARIES (Planar_SingleContact
	${CS225A_COMMON_LIBRARIES}
)

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CS225A_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/Planar_SingleContact)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY testRRR.urdf world_testRRR.urdf DESTINATION ${APP_RESOURCE_DIR})
