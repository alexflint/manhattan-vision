#include <cmath>

#include <iostream>

#include <VW/Events/events.h>
#include <VW/Image/imagecopy.h>
#include <VW/Image/imageio.h>
#include <VW/Sequencers/sequencermovie2.h> 
#include <VWGL/Display/displayglut.h>
#include <VWGL/Display/glutloop.h>
#include <VWGL/Display/drawgl.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "misc.h"
#include "common_types_vw.h"
#include "vanishing_points.h"
#include "timer.h"
#include "image_bundle.h"

using namespace VWEvents;

class MovieFilterer : public EventHandler {
private:
	SequencerMovie2* seq;
	bool isplaying;

	VanishingPointsEM vpts;
	ImageBundle image;
	ImageRGB<byte> frame;

	scoped_ptr<DisplayGLUT> disp;
	Mutex frameMutex;

public:
	MovieFilterer(SequencerMovie2* sequencer)
		: seq(sequencer),
			isplaying(false) { }

	void OnDraw(bool b) {
		if (frame.IsAlloced()) {
			frameMutex.Lock();
			DrawFunctions::Draw(*disp, frame);
			disp->Flush();
			frameMutex.Unlock();
		}
	}

	void OnNewFrame(int i) {
		// Find vanishing points
		seq->CopyImage(image.rgb);
		image.Invalidate();
		TIMED("Compute vanishing points")
			vpts.Compute(image);

		// Render the frame
		frameMutex.Lock();
		vpts.DrawVanPointViz(frame);
		frameMutex.Unlock();

		// Invalidate the display
		disp->RequestDraw();
	}

	void OnKeyboard(unsigned char key, int x, int y) {
		switch (key) {
		case ' ':
			if (isplaying) {
				seq->Stop();
			} else {
				seq->Continuous();
			}
			isplaying = !isplaying;
			break;
		case 'n':
			seq->NextFrame();
			break;
		case 'r':
			seq->Rewind();
			break;
		case 'w':
			WriteImage("frame.png", frame);
			cout << "Wrote frame.png\n";
			break;
		case 'q':
		case 27: 
			exit(0);
			break;
		}
	}

	void Start() {
		const int nx = seq->GetImageWidth();
		const int ny = seq->GetImageHeight();

		frame.AllocImageData(nx*1.5, ny*1.5);
		disp.reset(new DisplayGLUT(frame.GetWidth(),
															 frame.GetHeight(),
															 "Filtered movie"));

		disp->DrawEvent.Attach(this, &MovieFilterer::OnDraw);
		disp->KeyboardEvent.Attach(this, &MovieFilterer::OnKeyboard);
		seq->NewFrameEvent.Attach(this, &MovieFilterer::OnNewFrame);

		GLUTLoop();
	}
};




int main(int argc, char **argv) {
  GLUTInit(argc, argv);
	InitVars(argc, argv, "../common.cfg");
	const char* movie_file = argv[1];

	// Initialize the sequencer
	DLOG << "Initializing the sequencer...\n";
	SequencerMovie2 seq;
	if (!seq.InitSequencer()) {
		cerr << "InitSequencer failed\n";
		exit(-1);
	}
	if (!seq.AddChannel(movie_file)) {
		cerr << "Cannot read movie: " << movie_file << endl;
		exit(-1);
	}

	// Start the movie
	DLOG << "Starting the GUI...\n";
	MovieFilterer filterer(&seq);
	filterer.Start();

	return 0;
}
