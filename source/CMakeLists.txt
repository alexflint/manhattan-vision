PROJECT( IndoorContext )
SET( CMAKE_BUILD_TYPE RelWithDebInfo )
CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

# Eclipse settings
# Set the message length to help Eclipse parse compiler messages
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0" )
# Uncomment this to help Eclipse discover include paths 
# SET( CMAKE_VERBOSE_MAKEFILE OFF )

# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )

# Find required dependencies
FIND_PACKAGE( OpenGL REQUIRED )
FIND_PACKAGE( GLUT REQUIRED )
FIND_PACKAGE( Protobuf REQUIRED )
FIND_PACKAGE( Cairomm REQUIRED )
FIND_PACKAGE( SigC++ REQUIRED )	# for cairo

SET( VW_USE_LIBS VW VNL )
FIND_PACKAGE( VW REQUIRED )

# Find optional dependencies
FIND_PACKAGE( Matlab )	# for mex stuff

# Find TooN headers (TooN is just headers!)
FIND_PATH( TOON_INCLUDE_DIR TooN.h
	/usr/include/TooN /usr/local/include/TooN)
IF( NOT TOON_INCLUDE_DIR )
	MESSAGE( FATAL_ERROR "Could not find TooN include directory." )
ENDIF( NOT TOON_INCLUDE_DIR )

# Find Boost
SET( Boost_FIND_QUIETLY OFF )
FIND_PACKAGE( Boost 1.42.0
	COMPONENTS date_time filesystem iostreams thread
	REQUIRED )

# Find CUDA if it's enabled
IF( USE_CUDA )
	SET( FIND_CUDA_DIR "${CMAKE_MODULE_PATH}/cuda" )
	LIST( APPEND CMAKE_MODULE_PATH ${FIND_CUDA_DIR} )
	FIND_PACKAGE( Cuda )
ENDIF( USE_CUDA )

# Set up graph cuts
SET( GraphCuts_INCLUDE_DIR /usr/local/include/graphcuts )
SET( GraphCuts_LIBRARIES -lgraphcuts )

# Include dirs
INCLUDE_DIRECTORIES(
	${VW_INCLUDE_DIR}
	${TOON_INCLUDE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}	 # because protobufs will be generated here
	${OPENGL_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${PROTOBUF_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${Cairomm_INCLUDE_DIRS}
	${SigC++_INCLUDE_DIRS}
	${GraphCuts_INCLUDE_DIR}
)

# Generate protos
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS map.proto)

# Core library
SET( CORE_HEADERS
	tinyxml.h
	tinystr.h

	${PROTO_HDRS}

	canny.h
	common_types.h
	concurrent_queue.h
	eigensystem2d.h
	fast_sobel.h 
	fhsegmenter.h
	filters.h
	gaussian.h
	hw_convolver.h
	kmeans.h
	lazyvar.h
	log.h
	counted_foreach.h 
	progress_reporter.h 
	timer.h 
	union_find.h 
	vanishing_points.h 
	worker.h 
	image_bundle.h
	textons.h
	clipping.h
	line_sweeper.h
	unwarped_image.h
	rotation_estimator.h
	map.h
	vars.h
	hotspot.h
	colors.h
	mean_shift.h
	line_segment.h
	
	image_utils.h
	math_utils.h

	viewer3d.h
	widget3d.h
	glut_window.h
	invokation_serializer.h

	canvas.h
	canvas.tpp

	line_detector.h
	guided_line_detector.h
	camera.h
	
	event.h
	floorplan_editor.h
	floor_ceil_map.h
	
	manhattan_bnb.h
	manhattan_dp.h
	bld_helpers.h

	image_utils.tpp
	io_utils.tpp
	range_utils.tpp
	math_utils.tpp
	vector_utils.tpp
	
	table.tpp
	integral_col_image.tpp
	histogram.tpp
	fill_polygon.tpp
	polygon.tpp

	ATANCamera.h
	
	colored_points.h
	read_ply.h
	
	floorplan_renderer.h

	manhattan_bnb.h
	manhattan_dp.h
	manhattan_inference.h

)

SET( CORE_SOURCES 
	tinyxml.cpp
	tinystr.cpp
	tinyxmlerror.cpp
	tinyxmlparser.cpp

	${PROTO_SRCS}

	netlib_sanity_test.cpp

	canny.cpp
	fast_sobel.cpp
	fhsegmenter.cpp
	filters.cpp
	gaussian.cpp
	hw_convolver.cpp
	kmeans.cpp 
	log.cpp
	progress_reporter.cpp
	union_find.cpp
	vanishing_points.cpp
	worker.cpp
	image_bundle.cpp
	textons.cpp
	clipping.cpp
	line_sweeper.cpp
	unwarped_image.cpp
	rotation_estimator.cpp
	map.cpp
	vars.cpp
	colors.cpp
	mean_shift.cpp
	
	geom_utils.cpp
	image_utils.cpp
	math_utils.cpp
	gl_utils.cpp
	io_utils.cpp
	vector_utils.cpp

	viewer3d.cpp
	widget3d.cpp
	map_widgets.cpp
	hotspot.cpp
	invokation_serializer.cpp

	canvas.cpp

	line_detector.cpp
	guided_line_detector.cpp
	
	manhattan_bnb.cpp
	manhattan_dp.cpp
	manhattan_inference.cpp
	
	camera.cpp
	glut_window.cpp
	floorplan_editor.cpp
	floor_ceil_map.cpp
	bld_helpers.cpp
	histogram.cpp
	
	ATANCamera.cpp
	
	colored_points.cpp
	read_ply.cpp
	
	simple_renderer.cpp
	floorplan_renderer.cpp
)

# Optional CUDA components
IF ( USE_CUDA )
	MESSAGE( STATUS "Found CUDA" )

	# The CUDA library
	SET( CUDA_SDK_DIR /usr/local/NVIDIA_CUDA_SDK )
	SET( CUDA_LIBRARIES
		${CUDA_SDK_DIR}/lib/libcutil.a
		${CUDA_TARGET_LINK}
	)
	CUDA_INCLUDE_DIRECTORIES(
		${CMAKE_CURRENT_SOURCE_DIR}
		${CUDA_SDK_DIR}/common/inc
	)
	CUDA_ADD_LIBRARY( cuda_conv cuda_conv.cu )

	# Include the interface code in the main library
	ADD_DEFINITIONS( -DHAVE_CUDA )

ELSE( USE_CUDA )
	MESSAGE( STATUS "Not using CUDA (USE_CUDA was not set)." )
ENDIF( USE_CUDA )

# Core library
ADD_LIBRARY( core SHARED ${CORE_SOURCES} )

# Executable targets
SET( EXECUTABLES
	map_viewer

	edit_floorplan
	edit_map

	cairo_test
	clip_test
	clip3d_test

	map2pmvs
	ply_viewer
	draw_pts
	mws
	gc_example
	glut_test
	
	manhattan_colors
	draw_top_view
	predict_orientations
	
	render_floorplan
	
	toon_speed_test
	include_test
	render_test
	rectification_test

	poster_lines
	poster_vpts
	poster_sweeps
	
	draw_gt

	monocular_bnb
	multiview_bnb
	monocular_dp
	multiview_dp
	feature_dp
	
	test
)


SET (USE_CPU_PROFILER FALSE)

IF (USE_CPU_PROFILER)
    SET( CPU_PROFILER_LIBRARIES -lprofiler )
ENDIF (USE_CPU_PROFILER)

# Setup linked libraries
SET( EXTERNAL_LIBRARIES
	-lGVars3
	-lcvd
	-lGKTools
	-llapack
	${VW_LIBRARIES}
	${Cairomm_LIBRARIES}
	${SigC++_LIBRARIES}
	${GraphCuts_LIBRARIES}
	${PROTOBUF_LIBRARY}
	${GLUT_LIBRARIES}
	${OPENGL_LIBRARIES}
	${Boost_LIBRARIES}
	${CPU_PROFILER_LIBRARIES}
	)
	
MESSAGE(STATUS ${EXTERNAL_LIBRARIES})

FOREACH( EXECUTABLE ${EXECUTABLES} )
	ADD_EXECUTABLE( ${EXECUTABLE} ${EXECUTABLE}.cpp )
	TARGET_LINK_LIBRARIES( ${EXECUTABLE} core ${EXTERNAL_LIBRARIES} )
ENDFOREACH( EXECUTABLE )


# Matlab entrypoints
IF ( MATLAB_FOUND )
    # Here we do some strange contortions due to mex/matlab setup:
    #    1. Build a normal static library called foo.mex.a for each foo.mex.cpp
    #    2. Build a MEX library (foo.mexa64, for example) and link it to (1)
    #    3. Build a normal executable and it it to (1)
    # This gives us a normal executable and a MEX binary that both provide access
    # to the same underlying code.
    #
    # The reason for all these contortions is that we want to have libcore.so
    # containing the logic without needing to link it to Matlab libraries or
    # include mex.h, and whateverwe compile with the "mex" program that comes
    # with matlab can't reference all our core logic because then we would have
    # to find a way to get mex to recognize all our CMake includes and linkages.
    # So we introduce a third piece that bridges the two: these are the .mex.cpp
    # files
    
    INCLUDE_DIRECTORIES( ${MATLAB_INCLUDE_DIR} )
    
    # This library contains common matlab utilities
    SET( MATUTILS_SOURCES matlab_utils.cpp )
    ADD_LIBRARY( matutils ${MATUTILS_SOURCES} )
    
    # The code with the mexFunction(...) stub 
    SET( MEX_STUB_SOURCE mex_stub.cpp )
    # The code with the main(...) stub
    SET( MAIN_STUB_SOURCE "main_stub.cpp" )

    # The list of matlab targets    
    SET( MEX_OUTPUTS
        discriminative_dp
        load_training_data
        compatibility
        solve_dp
    )
    
    # The list of common libraries that they get linked to
    SET ( INTERNAL_MATLAB_LIBRARIES core matutils )
    
    # Create the matlab targets
    FOREACH( MEX_OUTPUT ${MEX_OUTPUTS} )
        SET( EXECUTABLE_NAME ${MEX_OUTPUT} )
        SET( BRIDGE_LIBRARY ${MEX_OUTPUT}.mex )
        SET( BRIDGE_SOURCE ${MEX_OUTPUT}.mex.cpp )
        
        # Build target as a static library
        ADD_LIBRARY( ${BRIDGE_LIBRARY} STATIC ${BRIDGE_SOURCE} )
        TARGET_LINK_LIBRARIES( ${BRIDGE_LIBRARY} core ${MATLAB_LIBRARIES} )
    	
        # Build the matlab entry point to it
    	ADD_CUSTOM_COMMAND(
    	    TARGET ${BRIDGE_LIBRARY} POST_BUILD
    		COMMAND	mex -g -output ${MEX_OUTPUT}
    		    ${CMAKE_CURRENT_SOURCE_DIR}/${MEX_STUB_SOURCE}
    		    ${CMAKE_CURRENT_BINARY_DIR}/lib${BRIDGE_LIBRARY}.a
    		    ${CMAKE_CURRENT_BINARY_DIR}/libcore.so
    		    ${CMAKE_CURRENT_BINARY_DIR}/libmatutils.a
    		    ${EXTERNAL_LIBRARIES}
    		DEPENDS ${MEX_STUB_SOURCE} ${BRIDGE_LIBRARY} ${INTERNAL_MATLAB_LIBRARIES}
    		)

    	# Build the regular executable entry point to it
    	ADD_EXECUTABLE( ${EXECUTABLE_NAME} ${MAIN_STUB_SOURCE} )
    	TARGET_LINK_LIBRARIES( ${MEX_OUTPUT}
    	    ${BRIDGE_LIBRARY}
    	    ${INTERNAL_MATLAB_LIBRARIES}
    	    ${EXTERNAL_LIBRARIES}
    	    )

        # For convenience, always build the MEX target when building the binary target
    	ADD_DEPENDENCIES( ${MEX_OUTPUT} ${BRIDGE_LIBRARY} )
    	
    ENDFOREACH( MEX_OUTPUT )
ENDIF ( MATLAB_FOUND )
