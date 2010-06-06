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
#include "fhsegmenter.h"

using namespace VWEvents;

class MovieFilterer : public EventHandler {
private:
	SequencerMovie2* seq;
	bool isplaying;
	bool showseg;  // whether to show the segmentation or the original

	scoped_ptr<FHSegmenter> segmenter;
	ImageRGB<byte> frame;
	MatI seg;

	vector<PixelRGB<byte> > seg_colors;

	scoped_ptr<DisplayGLUT> disp;
	Mutex frameMutex;

public:
	MovieFilterer(SequencerMovie2* sequencer)
		: seq(sequencer),
			isplaying(false),
			showseg(false) { }

	void OnDraw(bool b) {
		if (frame.IsAlloced()) {
			frameMutex.Lock();
			DrawFunctions::Draw(*disp, frame);
			disp->Flush();
			frameMutex.Unlock();
		}
	}

	void OnNewFrame(int i) {
		ImageRGB<byte> imagergb;
		seq->CopyImage(imagergb);
		WriteImage("imagergb.jpg", imagergb);
		ImageF image;
		ImageCopy(imagergb, image);
		segmenter->Compute(image, seg, 1, 25, 150);

		// Read the frame
		frameMutex.Lock();
		DrawSegmentation(seg, frame);
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
		case 's':
			showseg = !showseg;
			disp->RequestDraw();
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
		int width = seq->GetImageWidth();
		int height = seq->GetImageHeight();

		generate_n(back_inserter(seg_colors), 1000, &RandomColor);

		segmenter.reset(new FHSegmenter(width, height));
		frame.AllocImageData(width, height);
		seg.Resize(height, width);

		disp.reset(new DisplayGLUT(width, height, "Filtered movie"));

		disp->DrawEvent.Attach(this, &MovieFilterer::OnDraw);
		disp->KeyboardEvent.Attach(this, &MovieFilterer::OnKeyboard);
		seq->NewFrameEvent.Attach(this, &MovieFilterer::OnNewFrame);

		GLUTLoop();
	}
};




int main(int argc, char **argv) {
  GLUTInit(argc, argv);
	if (argc != 5) {
		cerr << "Usage: runseg K MINSIZE MINDIFF INPUT" << endl;
		exit(-1);
	}

	const float k = atof(argv[1]);
	const int min_size = atoi(argv[2]);
	const float min_diff = atof(argv[3]);
	const char* movie_file = argv[4];

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
