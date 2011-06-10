#include <unistd.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "common_types.h"
#include "vars.h"
#include "glut_window.h"

using namespace indoor_context;
using namespace toon;

void Foo(GlutWindow& w, string s) {
	DLOG << "in glut thread!";
	w.SetTitle(s);
}

void TitleLoop(GlutWindow& w) {
	int i = 1;
	while (true) {
		sleep(1);
		string s = lexical_cast<string>(i);
		DLOG << "called...";
		//GlutWindow::CallInGlutThread(bind(&GlutWindow::SetTitle, ref(w), s));
		//GlutWindow::CallInGlutThread(bind(&Foo, ref(w), s));
		w.SetTitle(s);
		w.SetPosition(ImageRef(i*10, i*10));
		i++;
	}
}

void CreateWin(string s) {
	GlutWindow* w = new GlutWindow(s);
	w->Run();
}

void CreateFoo() {
	GlutWindow* ww = new GlutWindow("Foo Window");
	ww->Create();

	//glutCreateWindow("abc");
}

void idle() { }
void disp() { glutSwapBuffers(); }
int main(int argc, char **argv) {
	GlutWindow::Init(&argc, argv);
	InitVars(argc, argv);

	GlutWindow w1("First");
	GlutWindow w2("Second");

	//thread t1(bind(&TitleLoop, ref(w1)));
	thread t2(bind(&TitleLoop, ref(w2)));

	//w1.Create();
	//GlutWindow::RunInGlutThread(&CreateFoo);
	//GlutWindow::RunInGlutThread(bind(&GlutWindow::Create, ref(w1)));
	GlutWindow::RunInGlutThread(bind(&GlutWindow::Create, ref(w2)));
	w1.RunAsync();
	//GlutWindow::LoopAsync();

	sleep(10);
	return 0;
}
