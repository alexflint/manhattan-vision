PROJECT( IndoorContext )
SET( CMAKE_BUILD_TYPE RelWithDebInfo )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Set the message length to help Eclipse parse compiler messages
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0" )

# Setup the path to find modules to prefer local modules
SET( CMAKE_MODULE_PATH
	${CMAKE_SOURCE_DIR}/../cmake_modules
	${CMAKE_ROOT}/Modules )

###############################################################################
# CUSTOM VARIABLES
###############################################################################

# The variable controlling the 'light' version
SET( LIGHT ON CACHE BOOL "Whether to build with minimal dependencies" )

###############################################################################
# LOCAL SOURCE PATHS
###############################################################################

# TODO: put these inside the #includes explicitly
INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/base
	${CMAKE_SOURCE_DIR}/visualization
	${CMAKE_SOURCE_DIR}/tinyxml
)
	
# Include the binary dir because protobufs will be generated here
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )

###############################################################################
# FIND DEPENDENCIES
###############################################################################

# Find libpng
FIND_PACKAGE( PNG REQUIRED )
INCLUDE_DIRECTORIES( ${PNG_INCLUDE_DIR } )
LIST( APPEND EXTERNAL_LIBRARIES ${PNG_LIBRARIES} )

# Find libjpeg
FIND_PACKAGE( JPEG REQUIRED )
INCLUDE_DIRECTORIES( ${JPEG_INCLUDE_DIR } )
LIST( APPEND EXTERNAL_LIBRARIES ${JPEG_LIBRARIES} )

# Find LAPACK
FIND_PACKAGE( LAPACK REQUIRED )
INCLUDE_DIRECTORIES( ${LAPACK_INCLUDE_DIR } )
LIST( APPEND EXTERNAL_LIBRARIES ${LAPACK_LIBRARIES} )

# Find protocol buffers
FIND_PACKAGE( Protobuf REQUIRED )
INCLUDE_DIRECTORIES( ${PROTOBUF_INCLUDE_DIR} )
LIST( APPEND EXTERNAL_LIBRARIES ${PROTOBUF_LIBRARIES} )

# Find Boost
SET( Boost_FIND_QUIETLY OFF )
FIND_PACKAGE( Boost 1.44.0
	COMPONENTS date_time filesystem iostreams thread system program_options # python
	REQUIRED )
LIST(APPEND EXTERNAL_LIBRARIES 
	${Boost_system_LIBRARY_RELEASE}
	${Boost_date_time_LIBRARY}
	${Boost_filesystem_LIBRARY}
	${Boost_iostreams_LIBRARY}
	${Boost_thread_LIBRARY}
	${Boost_program_options_LIBRARY}
)
ADD_DEFINITIONS( "-DBOOST_FILESYSTEM_VERSION=3" )

# Find TooN
FIND_PATH( TOON_INCLUDE_DIR TooN.h
	/usr/include/TooN /usr/local/include/TooN)
IF( NOT TOON_INCLUDE_DIR )
	MESSAGE( FATAL_ERROR "Could not find TooN include directory." )
ENDIF( NOT TOON_INCLUDE_DIR )
INCLUDE_DIRECTORIES( ${TOON_INCLUDE_DIR} )

# Find GVars3
FIND_PATH( GVars3_INCLUDE_DIR gvars3/gvars3.h )
FIND_LIBRARY( GVars3_LIBRARY libGVars3.so )
IF( NOT GVars3_INCLUDE_DIR )
  MESSAGE( FATAL_ERROR "Could not find GVars3 include directory.")
ENDIF( NOT GVars3_INCLUDE_DIR )
INCLUDE_DIRECTORIES( ${GVars3_INCLUDE_DIR} )
LIST( APPEND EXTERNAL_LIBRARIES ${GVars3_LIBRARY} )

# Find VW
SET( VW_USE_LIBS VNL )
FIND_PACKAGE( VW REQUIRED )
INCLUDE_DIRECTORIES( ${VW_INCLUDE_DIR} )
LIST( APPEND EXTERNAL_LIBRARIES ${VW_LIBRARY} )

# Find optional packages
IF( NOT ${LIGHT} )

	# Cairo and friends
	FIND_PACKAGE( Cairo REQUIRED )
	INCLUDE_DIRECTORIES( ${Cairo_INCLUDE_DIR} )
	LIST( APPEND EXTERNAL_LIBRARIES ${Cairo_LIBRARIES} )

	FIND_PACKAGE( Cairomm REQUIRED )
	INCLUDE_DIRECTORIES( ${Cairomm_INCLUDE_DIR} /usr/lib/cairomm-1.0/include )
	LIST( APPEND EXTERNAL_LIBRARIES ${Cairomm_LIBRARIES} )

	FIND_PACKAGE( Freetype REQUIRED )
	INCLUDE_DIRECTORIES( ${Freetype_INCLUDE_DIRS} )
	LIST( APPEND EXTERNAL_LIBRARIES ${Freetype_LIBRARIES} )

	FIND_PACKAGE( SigC++ REQUIRED )
	INCLUDE_DIRECTORIES( ${SigC++_INCLUDE_DIRS} )
	LIST( APPEND EXTERNAL_LIBRARIES ${SigC++_LIBRARIES} )

	# OpenGL and friends
	FIND_PACKAGE( OpenGL REQUIRED )
	INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )
	LIST( APPEND EXTERNAL_LIBRARIES ${GLUT_LIBRARIES} )

	FIND_PACKAGE( GLUT REQUIRED )
	INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} )
	LIST( APPEND EXTERNAL_LIBRARIES ${OPENGL_LIBRARIES} )

	# Python stuff
	FIND_PACKAGE( PythonLibs )
	INCLUDE_DIRECTORIES( ${PYTHON_INCLUDE_DIRS} )
	FIND_PACKAGE( ZLIB )
	INCLUDE_DIRECTORIES( ${ZLIB_INCLUDE_DIRS} )

	# Python-related
	FIND_PACKAGE( Matlab )

	# Find CUDA if it's enabled
	IF( USE_CUDA )
		SET( FIND_CUDA_DIR "${CMAKE_MODULE_PATH}/cuda" )
		LIST( APPEND CMAKE_MODULE_PATH ${FIND_CUDA_DIR} )
		FIND_PACKAGE( Cuda )
	ENDIF( USE_CUDA )
	
	# Configure CPU profiling
	IF ( USE_CPU_PROFILER )
		MESSAGE( STATUS "Linking with CPU profiler" )
		LIST( APPEND EXTERNAL_LIBRARIES -lprofiler )
	ENDIF ( USE_CPU_PROFILER )
	
	# Configure tc-malloc
	IF ( USE_TCMALLOC )
		MESSAGE( STATUS "Linking with tcmalloc" )
		LIST( APPEND EXTERNAL_LIBRARIES -ltcmalloc )
	ENDIF ( USE_TCMALLOC)
ENDIF()


# If in release mode then compile without bounds checking (TooN, Eigen, etc)
IF (CMAKE_BUILD_TYPE STREQUAL "Release" OR
		CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
	ADD_DEFINITIONS( -DNDEBUG )
ENDIF()

# Include prefix.h, which is precompiled to speed up compilation
#IF(CMAKE_COMPILER_IS_GNUCC)
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include ${CMAKE_SOURCE_DIR}/prefix.h")
#ENDIF(CMAKE_COMPILER_IS_GNUCC)

###############################################################################
# GENERATE PROTOCOL BUFFERS
###############################################################################
SET( PROTOS
	vector.proto
	map.proto
	model.proto
	payoffs.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTOS})

ADD_LIBRARY( proto SHARED
	${PROTO_SRCS}
)

###############################################################################
# BUILD CORE LIBRARY
# This library has minimal dependencies on external libraries.
###############################################################################
ADD_LIBRARY( base SHARED 
	# Logging macros and convenience framework
	base/log.cpp
	base/check.cpp
	base/report.cpp
	base/streamable.cpp
	base/vararg_utils.cpp
	base/progress_reporter.cpp

	# Parallelism
	base/worker.cpp
	base/thread_pool.cpp	

	# Miscellaneous utilities
	base/timer.cpp
	base/numeric_utils.cpp
	base/vector_utils.cpp
)	

###############################################################################
# BUILD TINYXML LIBRARY
# Used for reading PTAM maps from XML
###############################################################################
ADD_LIBRARY( tinyxml SHARED 
	tinyxml/tinyxml.cpp
	tinyxml/tinystr.cpp
	tinyxml/tinyxmlerror.cpp
	tinyxml/tinyxmlparser.cpp
)

###############################################################################
# BUILD LIKELIHOODS LIBRARY
# Has minimal dependencies to allow it to be easily used for black blox 
# optimization
###############################################################################
ADD_LIBRARY( likelihoods SHARED
	dp_payoffs.cpp
	payoff_helpers.cpp
	likelihoods.cpp
)

ADD_DEPENDENCIES( likelihoods base proto )

###############################################################################
# BUILD CORE LIBRARY
# This library contains most of the generic image processing machinery
# and many algorithmic implementations.
###############################################################################
ADD_LIBRARY( algorithms SHARED 

	# Simple data structures
	colors.cpp
	camera.cpp
	ATANCamera.cpp
	line_segment.cpp

	# Image data structures
	vw_image.cpp
	vw_image_io.cpp
	image_bundle.cpp

	# Map data structures
	map.cpp
	map_io.cpp

	# Miscellaneous utilities
	netlib_sanity_test.cpp
	vars.cpp
	geom_utils.cpp
	image_utils.cpp
	filesystem_utils.cpp
	clipping.cpp

	# Image processing and algorithms
	canny.cpp
	fast_sobel.cpp
	fhsegmenter.cpp
	filters.cpp
	gaussian.cpp
	histogram.cpp	
	hw_convolver.cpp
	kmeans.cpp 
	union_find.cpp
	vanishing_points.cpp
	textons.cpp
	line_sweeper.cpp
	rotation_estimator.cpp
	mean_shift.cpp
	line_detector.cpp
	guided_line_detector.cpp
	gaussian_pyramid.cpp
	
	# Simple custom rendering pipeline
	simple_renderer.cpp
	floorplan_renderer.cpp

	# 2D drawing
	canvas.cpp

#	kinect_device.cpp
#	kinect_transform.cpp
)

ADD_DEPENDENCIES( algorithms base tinyxml proto )

###############################################################################
# BUILD LIKELIHOODS LIBRARY
# No other libraries depend on this
###############################################################################
ADD_LIBRARY( visualization SHARED
	visualization/gl_utils.cpp
	visualization/viewer3d.cpp
	visualization/widget3d.cpp
	visualization/texture_manager.cpp
	visualization/glut_window.cpp
	visualization/map_widgets.cpp
	visualization/hotspot.cpp
	visualization/invokation_serializer.cpp
	visualization/colored_points.cpp
	visualization/read_ply.cpp
	visualization/floorplan_editor.cpp
)

ADD_DEPENDENCIES( visualization algorithms base )

###############################################################################
# BUILD MANHATTAN LIBRARY
###############################################################################
ADD_LIBRARY( manhattan SHARED
	manhattan_dp.cpp
	manhattan_ground_truth.cpp

	multiview_reconstructor.cpp

	monocular_payoffs.cpp
	stereo_payoffs.cpp
	point_cloud_payoffs.cpp
	joint_payoffs.cpp

	line_sweep_features.cpp
	building_features.cpp
	bld_helpers.cpp

	svm_light_wrappers.cpp
	svm_helpers.cpp

	manhattan_bnb.cpp
)

ADD_DEPENDENCIES( manhattan algorithms base likelihoods proto )

###############################################################################
# BUILD EXECUTABLES
###############################################################################
ADD_SUBDIRECTORY(progs)

###############################################################################
# BUILD CUDA COMPONENTS
###############################################################################
IF ( USE_CUDA )
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

ENDIF( USE_CUDA )


###############################################################################
# BUILD PYTHON WRAPPERS
###############################################################################
IF ( PYTHON_FOUND )
	SET( PYTHON_GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/generate_python_wrappers.py )
	SET( PYTHON_EXPOSED_FILES
		${CMAKE_CURRENT_SOURCE_DIR}/math_utils.h
		)
  ADD_CUSTOM_TARGET( OUTPUT python_wrappers.cpp
		COMMAND python ${PYTHON_GENERATOR}
		${PYTHON_EXPOSED_FILES} 
		)

ENDIF ( PYTHON_FOUND )


###############################################################################
# BUILD MATLAB WRAPPERS
###############################################################################
IF ( MATLAB_FOUND )
    # Here we do some strange contortions due to mex/matlab setup:
    #    1. Build a normal static library called foo.mex.a for each foo.mex.cpp
    #    2. Build a MEX library (foo.mexa64, for example) and link it to (1)
    #    3. Build a normal executable and link it it to (1)
    # This gives us a normal executable and a MEX binary that both provide access
    # to the same underlying code.
    #
    # The reason for all these contortions is that we want to have libcore.so
    # containing the logic without needing to link it to Matlab libraries or
    # include mex.h, and whatever we compile with the "mex" program that comes
    # with matlab can't reference all our core logic because then we would have
    # to find a way to get mex to recognize all our CMake includes and linkages.
    # So we introduce a third piece that bridges the two: these are the .mex.cpp
    # files
    INCLUDE_DIRECTORIES( ${MATLAB_INCLUDE_DIR} )
    
    # This library contains common matlab utilities
    ADD_LIBRARY( matutils
			matlab_utils.h        matlab_utils.cpp
			matlab_sink.h         matlab_sink.cpp
			matlab_structure.h    matlab_structure.cpp
			dp_structures.h       dp_structures.cpp
			)
    
    # The code with the mexFunction(...) stub 
    SET( MEX_STUB_SOURCE mex_stub.cpp )
    # The code with the main(...) stub
    SET( MAIN_STUB_SOURCE main_stub.cpp )

    # The list of matlab targets    
    SET( MEX_OUTPUTS
				mex_output_test
        dp_load_cases
        dp_solve
				logtest
				dp_check_features
    )
    
    # The list of common libraries that they get linked to
    SET ( INTERNAL_MATLAB_LIBRARIES algorithms matutils )
    
    # Create the matlab targets
    FOREACH( MEX_OUTPUT ${MEX_OUTPUTS} )
        SET( EXECUTABLE_NAME ${MEX_OUTPUT} )
				SET( MEX_NAME ${MEX_OUTPUT}.mexa64 )
        SET( BRIDGE_LIBRARY ${MEX_OUTPUT}.mex )
        SET( BRIDGE_SOURCE ${MEX_OUTPUT}.mex.cpp )
        
        # Build target as a static library
        ADD_LIBRARY( ${BRIDGE_LIBRARY} STATIC ${BRIDGE_SOURCE} )
        TARGET_LINK_LIBRARIES( ${BRIDGE_LIBRARY} algorithms ${MATLAB_LIBRARIES} )
    		
        # Build the matlab entry point to it
				#    	ADD_CUSTOM_COMMAND(
				#		  OUTPUT ${MEX_NAME}
    		ADD_CUSTOM_TARGET( ${MEX_NAME}
    			COMMAND mex -g -output ${MEX_OUTPUT}
    			${CMAKE_CURRENT_SOURCE_DIR}/${MEX_STUB_SOURCE}
    			${CMAKE_CURRENT_BINARY_DIR}/lib${BRIDGE_LIBRARY}.a
    			${CMAKE_CURRENT_BINARY_DIR}/libalgorithms.so
    			${CMAKE_CURRENT_BINARY_DIR}/libmatutils.a
    			${EXTERNAL_LIBRARIES}
    			DEPENDS ${MEX_STUB_SOURCE} ${BRIDGE_LIBRARY} ${INTERNAL_MATLAB_LIBRARIES}
    			)

    		# Build the regular executable entry point to it
    		ADD_EXECUTABLE( ${EXECUTABLE_NAME} ${MAIN_STUB_SOURCE} )
    		TARGET_LINK_LIBRARIES( ${EXECUTABLE_NAME}
    	    ${BRIDGE_LIBRARY}
    	    ${INTERNAL_MATLAB_LIBRARIES}
    	    ${EXTERNAL_LIBRARIES}
    	    )

        # Always build the MEX target when building the binary target
    		ADD_DEPENDENCIES( ${EXECUTABLE_NAME} ${MEX_NAME} )
    	
    ENDFOREACH( MEX_OUTPUT )
ENDIF ( MATLAB_FOUND )
