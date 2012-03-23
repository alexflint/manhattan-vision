###############################################################################
# BUILD PROTOCOL BUFFERS
###############################################################################

SET( PROTOS
	vector.proto
	map.proto
	model.proto
	payoffs.proto
	training.proto
)

PROTOBUF_GENERATE_CPP( 
	PROTO_SRCS PROTO_HDRS ${PROTOS}
)

ADD_LIBRARY( proto SHARED
	${PROTO_SRCS}
)

TARGET_LINK_LIBRARIES( proto ${PROTOBUF_LIBRARIES} )
