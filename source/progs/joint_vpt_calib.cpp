#include <LU.h>
#include <functions/derivatives.h>

#include "entrypoint_types.h"
#include "line_detector.h"
#include "image_utils.tpp"
#include "colors.h"

#include "io_utils.tpp"
#include "vector_utils.tpp"

using namespace indoor_context;

static const double kExitTol = 1e-5;
static const int kMaxIters = 1000;

Vec3 NormalizeLineEqn(const Vec3& v) {
	return v / sqrt(v[0]*v[0] + v[1]*v[1]);
}

using namespace toon;

Mat3 makeMatrix(double m11, double m12, double m13,
								double m21, double m22, double m23,
								double m31, double m32, double m33) {
	Mat3 m;
	m[0][0] = m11;
	m[0][1] = m12;
	m[0][2] = m13;
	m[1][0] = m21;
	m[1][1] = m22;
	m[1][2] = m23;
	m[2][0] = m31;
	m[2][1] = m32;
	m[2][2] = m33;
	return m;
}

Mat3 generate(const Vector<>& m, const vector<Mat3>& gs, int begin, int end) {
	CHECK_EQ(m.size(), gs.size());
	Mat3 a = Zeros;
	for (int i = begin; i < end; i++) {
		a += m[i] * gs[i];
	}
	return a;
}

Mat3 generate(const Vector<>& m, const vector<Mat3>& gs, int ngen) {	
	return generate(m, gs, 0, ngen);
}

Mat3 generate(const Vector<>& m, const vector<Mat3>& gs) {	
	return generate(m, gs, 0, gs.size());
}

LineDetection RandomLine(const Vec3& vpt, int axis) {
	// Pick a random point in the image
	Vec3 a = makeVector(rand()%100, rand()%100, 1.0);
	// Pick a random end line in the image
	//Vec3 b = (a^vpt)^RandomVector<3>();
	// Just use the vanishing point itself as the end point
	return LineDetection(a, vpt, axis);
}

LineDetection makeDet(double x1, double y1, double x2, double y2, int axis) {
	return LineDetection(makeVector(x1,y1,1.0), makeVector(x2,y2,1.0), axis);
}

double EvaluateResiduals(const vector<int>& owners,
												 const vector<Vec3>& lines,
												 const Mat3& K0,
												 const Mat3& R0,
												 const vector<Mat3>& Kgenerators,
												 const vector<Mat3>& Rgenerators,
												 const Vector<>& m) {
	CHECK_EQ(m.size(), Kgenerators.size()+Rgenerators.size());

	int nKgen = Kgenerators.size();
	int nRgen = Rgenerators.size();
	Mat3 K = K0 * exp(generate(m.slice(0,nKgen), Kgenerators));
	Mat3 R = R0 * exp(generate(m.slice(nKgen, nRgen), Rgenerators));
	//CHECK_LT(norm_fro(R - SO3<>::exp(m.slice(nKgen, nRgen)).get_matrix()), 1e-8);

	double f = 0.0;
	for (int i = 0; i < 3; i++) {
		// Compute current location of vpt
		Vec3 vi = K * R * GetAxis<3>(i);

		// Compute contribution from each line seg
		for (int j = 0; j < lines.size(); j++) {
			double rij = (owners[j] == i ? 1.0 : 0.0);  // ownership constant
			if (rij == 0.0) continue;
			double Eij = lines[j] * vi / norm(vi);
			f += rij * Eij*Eij;
			//DLOG << "[func] j="<<j<<", Eij="<<Eij<<", rij="<<rij;
		}
	}
	return f;
}

double elem(const Vector<>& v, int i) {
	return v[i];
}

Vec3 EvaluateVi(int i,
								const Mat3& K0,
								const Mat3& R0,
								const vector<Mat3>& Kgenerators,
								const vector<Mat3>& Rgenerators,
								const Vector<>& m) {
	CHECK_EQ(m.size(), Kgenerators.size()+Rgenerators.size());

	int nKgen = Kgenerators.size();
	int nRgen = Rgenerators.size();
	Mat3 K = exp(generate(m.slice(0,nKgen), Kgenerators));
	Mat3 R = exp(generate(m.slice(nKgen, nRgen), Rgenerators));

	// Compute current location of this vpt
	return K0 * K * R0 * R * GetAxis<3>(i);
}

	

int main(int argc, char **argv) {
	InitVars(argc, argv);
	/*if (argc != 2) {
		DLOG << "Usage: "<<argv[0]<<" IMAGE";
		return -1;
		}*/

	// Load image
	//ImageBundle image(argv[1]);

	// Identify line segments
	//CannyLineDetector detector(image);

	//ImageRGB<byte> mock(500,500);


	// Construct generators for space of calibration matrices
	vector<Mat3> Kgenerators;
	Kgenerators.push_back(makeMatrix(1,0,0,
																	 0,1,0,
																	 0,0,0));
	Kgenerators.push_back(makeMatrix(0,0,1,
																	 0,0,0,
																	 0,0,0));
	Kgenerators.push_back(makeMatrix(0,0,0,
																	 0,0,1,
																	 0,0,0));
	Kgenerators.push_back(makeMatrix(0,0,0,
																	 0,1,0,
																	 0,0,0));
	Kgenerators.push_back(makeMatrix(0,1,0,
																	 0,0,0,
																	 0,0,0));
	int nKgen = Kgenerators.size();

	// Construct generators for space of rotations
	vector<Mat3> Rgenerators;
	for (int i = 0; i < 3; i++) {
		Rgenerators.push_back(SO3<>::generator(i));
	}
	int nRgen = Rgenerators.size();


	// Generate random solution
	Vector<> r_gt = RandomVector<3>()*100.0;
	Mat3 R_gt = exp(generate(r_gt, Rgenerators));

	Vector<> k_gt = RandomVector<5>();
	Mat3 K_gt = exp(generate(k_gt, Kgenerators));

	DREPORT(r_gt, R_gt, k_gt, K_gt);

	// Check against SO3
	Mat3 check_R_gt = SO3<>::exp(r_gt).get_matrix();
	CHECK_LT(norm_fro(R_gt - check_R_gt), 1e-8) << EXPR_STR(R_gt) << EXPR_STR(check_R_gt);

	// Rotation must be applied first (ie appear on the right)
	Mat3 M_gt = K_gt * R_gt;


	// Generate some line detections
	vector<LineDetection> dets;
	/*
	SO3<> R_gt;
	Mat3 K_gt = makeMatrix(2,0,4,0,3,5,0,0,1);
	dets.push_back(makeDet(2,1, 5,1, 0));
	dets.push_back(makeDet(-12,3, 5,3, 0));
	dets.push_back(makeDet(0,-2, 1,-2, 0));
	dets.push_back(makeDet(6,0, 7,0, 0));

	dets.push_back(makeDet(1,1,  1,3, 1));
	dets.push_back(makeDet(2.5,-1,  2.5,3, 1));
	dets.push_back(makeDet(1.2,2,  1.2,2.5, 1));
	dets.push_back(makeDet(1,1,  1,5, 1));

	dets.push_back(makeDet(4,5, 3,1, 2));
	dets.push_back(makeDet(4,5, 0,1, 2));
	dets.push_back(makeDet(4,5, 7,8, 2));
	dets.push_back(makeDet(4,5, -1,-1, 2));
	*/

	// Generate some lines
	for (int i = 0; i < 3; i++) {
		Vec3 vi = unit(M_gt * GetAxis<3>(i));
		DREPORT(i, project(vi));
		for (int j = 0; j < 5; j++) {
			dets.push_back(RandomLine(vi, i));
			CHECK_LT(abs(dets.back().eqn * vi), 1e-8);
		}
	}

	// Unpack the lines
	vector<int> owners;
	vector<Vec3> lines;
	BOOST_FOREACH(const LineDetection& detection, dets) {
		lines.push_back(NormalizeLineEqn(detection.eqn));
		owners.push_back(detection.axis);
	}

	//	CHECK_EQ(lines.size(), 18);*/

	// 0 = horizontal (far) vpt
	// 1 = horizontal (centre) vpt
	// 2 = vertical vpt
	//int owners[] = {0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,0};

	/*ImageRGB<byte> own_canvas;
	own_canvas.Fill(Colors::white());
	ImageCopy(image.rgb, own_canvas);
	for (int i = 0; i < detector.detections.size(); i++) {
		detector.detections[i].DrawPixels(own_canvas,
																			Colors::primary(owners[i]),
																			makeVector(0,0), 0);
																			}*/
	//WriteImage("out/ownership.png", own_canvas);

	// Construct the origin in joint space
	int nparams = nKgen + nRgen;
	Vector<> origin(nparams);
	origin = Zeros;

	// Gradient descent loop
	double f;
	Vector<> Jac_f(nparams);
	Vector<> m_cur(nparams);
	m_cur = Zeros;

	// try initializing close
	m_cur.slice(0,nKgen) = k_gt + RandomVector<10>().slice(0,nKgen)*0.1;;
	m_cur.slice(nKgen,nRgen) = r_gt + RandomVector<10>().slice(0,nRgen)*0.1;

	int iter = 0;
	while (iter < kMaxIters) {
		TITLE("iteration "<<iter);

		// Extract parameters for K and R
		CHECK_EQ(m_cur.size(), nparams);
		Vector<> k_cur = m_cur.slice(0, nKgen);
		Vector<> r_cur = m_cur.slice(nKgen, nRgen);

		// Compute the current matrices
		Mat3 K0 = exp(generate(k_cur, Kgenerators));
		Mat3 R0 = exp(generate(r_cur, Rgenerators));
		//CHECK(norm_fro(R0-R_gt)<1e-8);

		DREPORT(k_cur, K0, r_cur, R0);

		// Compute residuals and jacobian
		f = 0;
		Jac_f = Zeros;
		for (int i = 0; i < 3; i++) {
			// Compute the vanishing point and its norm
			Vec3 ei = GetAxis<3>(i);
			Vec3 vi = K0*R0*ei;  // R0 _must_ proceed K0
			double norm_vi = norm(vi);

			// Compute jacobian of the vanishing point
			Matrix<> Jac_vi(3,nparams);
			for (int j = 0; j < nKgen; j++) {
				Jac_vi.slice(0,j,3,1)       = K0 * Kgenerators[j] * R0 * ei.as_col();
			}
			for (int j = 0; j < nRgen; j++) {
				Jac_vi.slice(0,nKgen+j,3,1) = K0 * R0 * Rgenerators[j] * ei.as_col();
			}
			for (int r = 0; r < Jac_vi.num_rows(); r++) {
				for (int c = 0; c < Jac_vi.num_cols(); c++) {
					CHECK(isfinite(Jac_vi[r][c]))
						<< EXPR_STR(Jac_vi) << EXPR_STR(k_cur) << EXPR_STR(r_cur) << EXPR_STR(K0) << EXPR_STR(R0);
				}
			}

			// Compute contribution from each line seg
			for (int j = 0; j < lines.size(); j++) {
				double rij = (owners[j] == i ? 1.0 : 0.0);  // ownership constant
				if (rij==0.0) continue;

				// Compute error and contribution to f
				double Eij = lines[j] * vi / norm_vi;
				double f_contrib = rij * Eij * Eij;

				// Compute jacobian of norm(vi)
				/*Vector<> Jac_norm_vi(nparams);
					Jac_norm_vi.as_row() = vi.as_row() * Jac_vi / norm_vi;*/
				Vector<> Jac_norm_vi = Jac_vi.T()*vi/norm_vi;

				// Compute jacobian of E_ij
				Vector<> Jac_Eij(nparams);
				Jac_Eij.as_row() = lines[j].as_row() * (Jac_vi*norm_vi - vi.as_col()*Jac_norm_vi.as_row()) / (norm_vi*norm_vi);

				// Compute contribution to Jac_f
				Vector<> Jac_f_contrib(nparams);
				Jac_f_contrib = rij * 2.0 * Eij * Jac_Eij;

				// Add
				f += f_contrib;
				Jac_f += Jac_f_contrib;
			}



			// Check against numerical derivatives...
			/*Vec3 vi_check;
			Matrix<> numeric_Jac_vi(3, nparams);
			for (int j = 0; j < 3; j++) {
				boost::function<double(const Vector<>&)> func_vi =
					bind(&elem,
							 bind(EvaluateVi,
										i,
										K0,
										R0,
										ref(Kgenerators),
										ref(Rgenerators),
										_1),
							 j);
				vi_check[j] = func_vi(origin);
				numeric_Jac_vi[j] = numerical_gradient(func_vi, origin);
			}
			CHECK_LT(norm_1(vi-vi_check), 1e-8);
			double relerr_Jac_vi = norm_fro(numeric_Jac_vi-Jac_vi) / norm_fro(numeric_Jac_vi);
			CHECK_LT(relerr_Jac_vi, 1e-8)
			<< EXPR_STR(numeric_Jac_vi) << EXPR_STR(Jac_vi);*/
		}


		// Check against numerical derivatives...
		/*boost::function<double(const Vector<>&)> func_f = bind(&EvaluateResiduals,
																													 ref(owners),
																													 ref(lines),
																													 K0,
																													 R0,
																													 ref(Kgenerators),
																													 ref(Rgenerators),
																													 _1);
  	double check_f = func_f(origin);
	  CHECK_EQ_TOL(f, check_f, 1e-8);
	
	  Vector<> check_Jac_f = numerical_gradient(func_f, origin);
		CHECK_LT(norm_sq(check_Jac_f-Jac_f), 1e-8)
			<<"Jac_f: "<<Jac_f
			<<"\ncheck_Jac_f: "<<check_Jac_f;

			DREPORT(Jac_f.as_col());*/

		// Only update R...
		//Jac_f.slice(nKgen,nRgen) = Zeros;

		// Exit if residual is small enough
		if (f < kExitTol) break;
	
		// Compute step
		double step_len = f / (Jac_f*Jac_f);
		Vector<> m_step = -step_len * Jac_f;

		// Perform step
		DREPORT(m_cur);
		m_cur += m_step;
		DREPORT(m_step, m_cur);

		//DREPORT(Jac_f*Jac_f);
		DLOG << "Residual: " << f;


		iter++;
	}

	DLOG << "\n\n**********";

	if (iter == kMaxIters) {
		DLOG << "Failed to converge after " << iter << " iterations";
	} else {
		DLOG << "Converged after " << iter << " iterations";
	}
	DLOG << "Final residual: " << f;

	Vector<> k_est = m_cur.slice(0, nKgen);
	Vector<> r_est = m_cur.slice(nKgen, nRgen);
	Mat3 K_est = exp(generate(k_est, Kgenerators));
	Mat3 R_est = exp(generate(r_est, Rgenerators));
	Mat3 M_est = K_est * R_est;
	DREPORT(K_est, R_est, M_est);

	DLOG << "Error on final M: " << norm_fro(M_est-M_gt);

	TITLED("Estimated vanishing points") {
		for (int i = 0; i < 3; i++) {
			DLOG <<i<<": "<<project(M_est*GetAxis<3>(i));
		}
	}

	TITLED("True vanishing points") {
		for (int i = 0; i < 3; i++) {
			DLOG <<i<<": "<<project(M_gt*GetAxis<3>(i));
		}
	}

	DREPORT(K_gt, R_gt, M_gt);

	/*for (int i = 0; i < detector.detections.size(); i++) {
		ImageRGB<byte> canvas;
		ImageCopy(image.rgb, canvas);
		detector.detections[i].DrawPixels(canvas, Colors::red(), makeVector(0,0), 0);
		WriteImage(str(format("out/line%02d.png")%i), canvas);
		}*/

	//detector.OutputLineViz("out/detections.png");

	return 0;
}
