PROJECT( IndoorContext )
SET( CMAKE_BUILD_TYPE RelWithDebInfo )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

# Eclipse settings
# Set the message length to help Eclipse parse compiler messages
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0" )

# Set the local version first
SET( CMAKE_MODULE_PATH
	${CMAKE_SOURCE_DIR}/../cmake_modules
	${CMAKE_ROOT}/Modules )


###############################################################################
# FIND DEPENDENCIES
###############################################################################

# Find Boost
SET( Boost_FIND_QUIETLY OFF )
FIND_PACKAGE( Boost 1.44.0
	COMPONENTS date_time filesystem iostreams thread system program_options # python
	REQUIRED )

# Find required dependencies
#FIND_PACKAGE( Eigen3 )
FIND_PACKAGE( Protobuf REQUIRED )
# GL-related
FIND_PACKAGE( OpenGL REQUIRED )
FIND_PACKAGE( GLUT REQUIRED )
# Cairo-related
FIND_PACKAGE( Cairo REQUIRED )
FIND_PACKAGE( Cairomm REQUIRED )
FIND_PACKAGE( Freetype REQUIRED )
FIND_PACKAGE( SigC++ REQUIRED )
# Python-related
FIND_PACKAGE( PythonLibs REQUIRED ) # Required by Boost.Python
FIND_PACKAGE( ZLIB REQUIRED ) # Required by Python
FIND_PACKAGE( Matlab )	# for mex stuff

# Find lapack
FIND_LIBRARY( LAPACK_LIBRARY liblapack.so )
IF( NOT LAPACK_LIBRARY )
	MESSAGE( FATAL_ERROR "Could not find LAPACK library" )
ENDIF()

# Find VW
SET( VW_USE_LIBS VNL )
FIND_PACKAGE( VW REQUIRED )

# Find TooN
FIND_PATH( TOON_INCLUDE_DIR TooN.h
	/usr/include/TooN /usr/local/include/TooN)
IF( NOT TOON_INCLUDE_DIR )
	MESSAGE( FATAL_ERROR "Could not find TooN include directory." )
ENDIF( NOT TOON_INCLUDE_DIR )

# Find GVars3
FIND_PATH( GVars3_INCLUDE_DIR gvars3/gvars3.h )
FIND_LIBRARY( GVars3_LIBRARY libGVars3.so )
IF( NOT GVars3_INCLUDE_DIR )
  MESSAGE( FATAL_ERROR "Could not find GVars3 include directory.")
ENDIF( NOT GVars3_INCLUDE_DIR )
MESSAGE( ${GVars3_INCLUDE_DIR} )

# Find CUDA if it's enabled
IF( USE_CUDA )
	SET( FIND_CUDA_DIR "${CMAKE_MODULE_PATH}/cuda" )
	LIST( APPEND CMAKE_MODULE_PATH ${FIND_CUDA_DIR} )
	FIND_PACKAGE( Cuda )
ENDIF( USE_CUDA )

# Find graph cuts
#SET( GraphCuts_INCLUDE_DIR /usr/local/include/graphcuts )
#SET( GraphCuts_LIBRARIES -lgraphcuts )

# Find Kinect driver
# TODO: put these into FindFreenect.cmake
# TODO: make these optional
#SET(libfreenect_LIBRARIES "freenect")
#SET(libfreenect_INCLUDE_DIR "/usr/local/include/libfreenect")
#SET(libfreenect_LIBRARY_DIRS "/usr/local/lib/libfreenect")
#INCLUDE_DIRECTORIES(${libfreenect_INCLUDE_DIR})
#LINK_DIRECTORIES(${libfreenect_LIBRARY_DIRS})

# Configure CPU profiling
IF ( USE_CPU_PROFILER )
	MESSAGE( STATUS "Linking CPU Profiler" )
  SET ( CPU_PROFILER_LIBRARIES -lprofiler )
ENDIF (USE_CPU_PROFILER)

# Configure tc-malloc
IF ( USE_TCMALLOC )
	MESSAGE( STATUS "Linking tcmalloc" )
	SET( TCMALLOC_LIBRARY -ltcmalloc )
ENDIF ( USE_TCMALLOC)

# Include dirs
INCLUDE_DIRECTORIES(
	${VW_INCLUDE_DIR}
	${TOON_INCLUDE_DIR}
	${GVars3_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${PROTOBUF_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${Cairo_INCLUDE_DIR}
	${Cairomm_INCLUDE_DIR}
	/usr/lib/cairomm-1.0/include   # Cairomm seem to need this in addition to above!
	${Freetype_INCLUDE_DIRS}
	${SigC++_INCLUDE_DIRS}
	#${GraphCuts_INCLUDE_DIR}
	${PYTHON_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
	#${EIGEN3_INCLUDE_DIR}

	${CMAKE_SOURCE_DIR} # because subdirs will need to find includes from this dir
	${CMAKE_CURRENT_BINARY_DIR}	 # because protobufs will be generated here
)

# Setup linked libraries
SET( EXTERNAL_LIBRARIES
	${GVars3_LIBRARY}
	${LAPACK_LIBRARY}
	${PROTOBUF_LIBRARY}
	${GLUT_LIBRARIES}
	${OPENGL_LIBRARIES}
	${VW_LIBRARIES}
	${Cairomm_LIBRARIES}
	${SigC++_LIBRARIES}  # needed by cairo

	# Enumerate specific libs to avoid linking Boost.Python to the core code
	${Boost_system_LIBRARY_RELEASE}
	${Boost_date_time_LIBRARY}
	${Boost_filesystem_LIBRARY}
	${Boost_iostreams_LIBRARY}
	${Boost_thread_LIBRARY}
	${Boost_program_options_LIBRARY}

	# for GIL
	-ljpeg
	-lpng

	#${GraphCuts_LIBRARIES}
	#${libfreenect_LIBRARIES}

	# Only for stereotracks
	#${EIGEN3_LIBRARIES}

	# Profiling libraries (empty unless USE_CPU_PROFILER / USE_TCMALLOC are true)
	${CPU_PROFILER_LIBRARIES}
	${TCMALLOC_LIBRARY}
)

# Use boost_filesystem version 3
ADD_DEFINITIONS( "-DBOOST_FILESYSTEM_VERSION=3" )

# If in release mode then compile Eigen without bounds checking
IF (CMAKE_BUILD_TYPE STREQUAL "Release" OR
		CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
	MESSAGE(STATUS "Eigen bounds checking is off.")
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

###############################################################################
# BUILD CORE LIBRARY
###############################################################################
ADD_LIBRARY( core SHARED 
	tinyxml.cpp
	tinystr.cpp
	tinyxmlerror.cpp
	tinyxmlparser.cpp

	${PROTO_SRCS}

	netlib_sanity_test.cpp

	log.cpp
	check.cpp
	timer.cpp
	vararg_utils.cpp
	streamable.cpp
	report.cpp

	canny.cpp
	fast_sobel.cpp
	fhsegmenter.cpp
	filters.cpp
	gaussian.cpp
	histogram.cpp	
	hw_convolver.cpp
	kmeans.cpp 
	progress_reporter.cpp
	union_find.cpp
	vanishing_points.cpp
	image_bundle.cpp
	textons.cpp
	clipping.cpp
	line_sweeper.cpp
	rotation_estimator.cpp
	map.cpp
	map_io.cpp
	vars.cpp
	colors.cpp
	mean_shift.cpp
	line_segment.cpp
	camera.cpp
	ATANCamera.cpp

	line_detector.cpp
	guided_line_detector.cpp

	worker.cpp
	thread_pool.cpp
	
	geom_utils.cpp
	image_utils.cpp
	numeric_utils.cpp
	gl_utils.cpp
	vector_utils.cpp
	filesystem_utils.cpp

	viewer3d.cpp
	widget3d.cpp
	texture_manager.cpp
	glut_window.cpp
	map_widgets.cpp
	hotspot.cpp
	invokation_serializer.cpp

	canvas.cpp
	floorplan_editor.cpp
	
	colored_points.cpp
	read_ply.cpp
	
	simple_renderer.cpp
	floorplan_renderer.cpp

#	kinect_device.cpp
#	kinect_transform.cpp

	vw_image.cpp
	vw_image_io.cpp
)


###############################################################################
# BUILD MANHATTAN LIBRARY
###############################################################################
ADD_LIBRARY( manhattan SHARED
	dp_payoffs.cpp
	manhattan_dp.cpp
	manhattan_ground_truth.cpp

	multiview_reconstructor.cpp

	monocular_payoffs.cpp
	stereo_payoffs.cpp
	point_cloud_payoffs.cpp
	joint_payoffs.cpp
	payoff_helpers.cpp
	likelihoods.cpp

	manhattan_bnb.cpp

	line_sweep_features.cpp
	building_features.cpp
	bld_helpers.cpp

	svm_light_wrappers.cpp
	svm_helpers.cpp

	gaussian_pyramid.cpp  # TODO: move to core
)

ADD_DEPENDENCIES( manhattan core )   # mostly for protobufs

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

	#TARGET_LINK_LIBRARIES( ... 
	# ${Boost_LIBRARIES}
	# ${PYTHON_LIBRARIES}
	# ${ZLIB_LIBRARIES}
	#)

ENDIF ( PYTHON_FOUND )


###############################################################################
# BUILD MATLAB WRAPPERS
###############################################################################
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
    SET ( INTERNAL_MATLAB_LIBRARIES core matutils )
    
    # Create the matlab targets
    FOREACH( MEX_OUTPUT ${MEX_OUTPUTS} )
        SET( EXECUTABLE_NAME ${MEX_OUTPUT} )
				SET( MEX_NAME ${MEX_OUTPUT}.mexa64 )
        SET( BRIDGE_LIBRARY ${MEX_OUTPUT}.mex )
        SET( BRIDGE_SOURCE ${MEX_OUTPUT}.mex.cpp )
        
        # Build target as a static library
        ADD_LIBRARY( ${BRIDGE_LIBRARY} STATIC ${BRIDGE_SOURCE} )
        TARGET_LINK_LIBRARIES( ${BRIDGE_LIBRARY} core ${MATLAB_LIBRARIES} )
    		
        # Build the matlab entry point to it
				#    	ADD_CUSTOM_COMMAND(
				#		  OUTPUT ${MEX_NAME}
    		ADD_CUSTOM_TARGET( ${MEX_NAME}
    			COMMAND mex -g -output ${MEX_OUTPUT}
    			${CMAKE_CURRENT_SOURCE_DIR}/${MEX_STUB_SOURCE}
    			${CMAKE_CURRENT_BINARY_DIR}/lib${BRIDGE_LIBRARY}.a
    			${CMAKE_CURRENT_BINARY_DIR}/libcore.so
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
