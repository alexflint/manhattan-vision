PROJECT( IndoorContext )
SET( CMAKE_BUILD_TYPE RelWithDebInfo )
CMAKE_MINIMUM_REQUIRED( VERSION 2.6 )

# Eclipse settings
# Set the message length to help Eclipse parse compiler messages
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
# Uncomment this to help Eclipse discover include paths 
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# Set the local version first
SET( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../cmake_modules" )
LIST( APPEND CMAKE_MODULE_PATH "${CMAKE_ROOT}/Modules" )

# Find dependencies
FIND_PACKAGE( GLUT REQUIRED )
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( Protobuf REQUIRED )
FIND_PACKAGE( Cairomm REQUIRED )
FIND_PACKAGE( SigC++ REQUIRED )	# for cairo

# Find VW
# VWFirewire uses the old libdc1934 so I have disabled it. This rules
# out using VW's Sequencer class.
SET( VW_USE_LIBS VWGL VW VNL )
FIND_PACKAGE( VW REQUIRED )

# Find TooN headers (TooN is just headers!)
FIND_PATH( TOON_INCLUDE_DIR TooN.h
	/usr/include/TooN /usr/local/include/TooN)
IF( NOT TOON_INCLUDE_DIR )
	MESSAGE( FATAL_ERROR "Could not find TooN include directory." )
ENDIF( NOT TOON_INCLUDE_DIR )

# Find Boost
SET( Boost_FIND_QUIETLY OFF )
#SET( Boost_ADDITIONAL_VERSIONS "1.42.0" "1.42")
#SET( _boost_COMPILER_VERSION "43" )
#SET( Boost_USE_MULTITHREAD ON )
FIND_PACKAGE( Boost 1.42.0
	COMPONENTS date_time filesystem iostreams thread
	REQUIRED )

MESSAGE( STATUS "Boost_INCLUDE_DIRS: " ${Boost_INCLUDE_DIRS} )
MESSAGE( STATUS "Boost_LIBRARIES: " ${Boost_LIBRARIES} )

# Find CUDA if it's enabled
IF( USE_CUDA )
	SET( FIND_CUDA_DIR "${CMAKE_MODULE_PATH}/cuda" )
	LIST( APPEND CMAKE_MODULE_PATH ${FIND_CUDA_DIR} )
	FIND_PACKAGE( Cuda )
ENDIF( USE_CUDA )

# Set up graph cuts
SET( GraphCuts_INCLUDE_DIR /usr/local/include/graphcuts )
SET( GraphCuts_LIBRARIES -lgraphcuts )

# Setup paths to PTAM 
# TODO: make this robust and optional. We are using ATANCamera from a
# version of PTAM different to the one we use to actually do the
# mapping. This is slightly worrying.
#SET( PTAM_PATH "/homes/50/alexf/work/ptam_versions/trunk_george" )
#SET( PTAM_LIBRARIES ${PTAM_PATH}/x86_64provost/libptam.a )

# Include dirs
INCLUDE_DIRECTORIES(
	${VW_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${PROTOBUF_INCLUDE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}	 # because protobufs will be generated here
	${TOON_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIR}
	${Cairomm_INCLUDE_DIRS}
	${SigC++_INCLUDE_DIRS}
	${GraphCuts_INCLUDE_DIR}
)

# Setup linked libraries
SET( LIBS
	core
	-lGVars3
	-lprofiler
	-lcvd
	-lGKTools
	-lboost_date_time
	-lboost_filesystem 
	-lboost_iostreams 
	-lboost_thread
	${Boost_LIBRARIES}
	${OPENCV_LIBRARIES}
	${VW_LIBRARIES}
	${PROTOBUF_LIBRARY}
	-llapack
	${Cairomm_LIBRARIES}
	${SigC++_LIBRARIES}
	${GraphCuts_LIBRARIES}
	-lGL
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
	misc.h 
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
	building_estimator.h
	event.h
	floorplan_editor.h
	floor_ceil_map.h
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

	ATANCamera.h
	
	colored_points.h
	read_ply.h
	
	floorplan_renderer.h
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
	misc.cpp
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
	camera.cpp
	building_estimator.cpp
	glut_window.cpp
	floorplan_editor.cpp
	floor_ceil_map.cpp
	manhattan_dp.cpp
	bld_helpers.cpp
	histogram.cpp
	
	ATANCamera.cpp
	
	colored_points.cpp
	read_ply.cpp
	
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
#	find_vpts
#	consensus_vpts
#	heh_features
#	seq_vpts
#	runfhseg
#	seq_segmenter
#	match_spix
#	label_planes
#	edgels
#	fit_rotation
#	repair_vpts
#	legend
#	third_vpt
#	calibrate_vpts
#	map_textons
#	sweep_planes
#	vpts_gui
#	vpts_3d
#	explie
#	unwarp
#	keyframe_vpts
#	multiview_vpts
#	multiview_find_vpts
#	lie_est
#	project_gradient
#	recover_rotation
#	recover_scene_bounds
#	map_annotator

#	learn_textons
#	detect_planes
#	plane_ess

#	multi_horiz_vpts
#	keyframe_line_sweeper
#	frame_line_sweeper
#	map_viewer

#	frame_structure_recovery
#	multiframe_bnb
#	canonical_image

#	vpt_comparison
#	peakline_experiment

#	structure_recovery
#	cvpr_video

	unconstrained_dp

	edit_floorplan
	edit_map

#	os_test
	cairo_test
	clip_test
	clip3d_test
#	glut_keys

#	fcmap_test
#	approx_camera
#	fast_orients

	map2pmvs
	ply_viewer
	draw_pts
	mws
	gc_example
	glut_test
	
	manhattan_colors
	draw_top_view
	predict_orientations
	
	toon_speed_test
	include_test
	poster_lines
)


# SET( TMP_MAP_FILE "${CMAKE_CURRENT_BINARY_DIR}/out.map" )
# LIST( APPEND CMAKE_CXX_FLAGS "-Wl,-Map=${TMP_MAP_FILE} -Wl,--cref" )

FOREACH( EXECUTABLE ${EXECUTABLES} )
	SET( CUR_EXE_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EXECUTABLE})

	ADD_EXECUTABLE( ${EXECUTABLE} ${EXECUTABLE}.cpp )
	TARGET_LINK_LIBRARIES( ${EXECUTABLE} ${LIBS} )

	# Extract symbols and linker map
	#ADD_CUSTOM_COMMAND(
#		TARGET ${EXECUTABLE} POST_BUILD
#		# Run nm to extract symbols
#		COMMAND ${CMAKE_NM} -S ${CUR_EXE_FILE} > ${CUR_EXE_FILE}.syms
#		# Extract everything after the "Cross Reference Table" line in the map file
#		COMMAND grep -A 100000 "Cross Reference Table" ${TMP_MAP_FILE} > ${CUR_EXE_FILE}.cref
#		VERBATIM)
ENDFOREACH( EXECUTABLE )

