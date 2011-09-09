###############################################################################
# BUILD EXECUTABLES
###############################################################################
SET( PROGS
	export_floorplan
	find_vpts
	sweep_lines
	test
	map_viewer
	add_painting

	edit_floorplan
	edit_map

	map2pmvs
	ply_viewer
	draw_pts
	mws
	gc_example
	glut_test
	jac_test
	load_bundle
	
	manhattan_colors
	draw_top_view
	predict_orientations
	
	render_floorplan

	poster_lines
	poster_vpts
	poster_sweeps
	
	draw_gt
	draw_sweeps

	monocular_bnb
	monocular_dp
	multiview_dp
	feature_dp
	evaluate_mv
	stereo_dp
	point_dp
	joint_dp
	kinect_dp
	estimate_floorplan
	generate_video
	maximize_loglik

	dp_optimization_wrapper
	compute_payoff_features

	inspect_geometry
	test_gt_walls
	tata_video

	test_solution_path
	test_atancamera

	joint_vpt_calib

	kinect_viewer

	generate_svm_problem
	bootstrap_train_dp
	oneshot_train_dp
	brostow

	toon_speed_test
	render_test
	rectification_test
	filter_test
	cairo_test
	clip_test
	clip3d_test
	table_speedtest
	gil_test
	image_test
	thread_pool_test
	eigen_test
	expr_test
)

FOREACH( PROG ${PROGS} )
	ADD_EXECUTABLE( ${PROG} ${PROG}.cpp )
	TARGET_LINK_LIBRARIES( ${PROG}
		base
		proto
		manhattan
		algorithms
		likelihoods
		visualization
		tinyxml
		${EXTERNAL_LIBRARIES}
	)
ENDFOREACH( PROG )

# Add this one separately to avoid dependencies
ADD_EXECUTABLE( compute_loglik compute_loglik.cpp )
TARGET_LINK_LIBRARIES( compute_loglik
	likelihoods
	base
	proto
	${EXTERNAL_LIBRARIES}
)
