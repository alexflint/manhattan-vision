#include <iostream>

#include "entrypoint_types.h"

int main(int argc, char **argv) {
	Mat3 x = Ones;
	string s = "foobar title... GAP\n\n";
	string t = "hello\nworld";
	string t2 = "hello\nworld.... NEWLINE\n";
	string foo = "foo\n";
	DLOG << EXPR(x, s, t, t2, foo);
	DLOG << "multiple ends...";
	LogManager::EndCurrentLine();
	LogManager::EndCurrentLine();
	return 0;
}
