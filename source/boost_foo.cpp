#include <iostream>

#include "prefix.h"

using namespace boost;
using namespace std;

int parse(string s) {
	return lexical_cast<int>(s);
}

int main(int argc, char **argv) {

	ptr_vector<string> v;

	function<int(string)> f = &parse;
	function<int()> g = bind(parse, "123");

	v.push_back(new string(lexical_cast<string>(g())));
	cout << v[0] << endl;

	return 0;
}
