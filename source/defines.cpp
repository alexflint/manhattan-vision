#include <iostream>
#include <string>
#include <vector>

using namespace std;

vector<string> ParseVaStrings(const string& s) {
	vector<string> vs;
	int a = 0;
	for (int i = 0; i <= s.length(); i++) {
		if (i == s.length() || s[i] == ',') {
			vs.push_back(s.substr(a, i-a));
			a = i+2;  // plus two for the comma and then then the space
		}
	}
	return vs;
}

template <typename T>
void ReportVar(const T& x, const string& name) {
	cout << name << ": " << x << endl;
}

template <typename A>
void ReportVars(const A& a, const vector<string>& names) {
	ReportVar(a, names[0]);
}

template <typename A, typename B>
void ReportVars(const A& a, const B& b, const vector<string>& names) {
	ReportVar(a, names[0]);
	ReportVar(b, names[1]);
}

template <typename A, typename B, typename C>
void ReportVars(const A& a,
								const B& b,
								const C& c,
								const vector<string>& names) {
	ReportVar(a, names[0]);
	ReportVar(b, names[1]);
	ReportVar(c, names[2]);
}

template <typename A, typename B, typename C, typename D>
void ReportVars(const A& a,
								const B& b,
								const C& c,
								const D& d,
								const vector<string>& names) {
	ReportVar(a, names[0]);
	ReportVar(b, names[1]);
	ReportVar(c, names[2]);
	ReportVar(d, names[3]);
}

#define PRINTALL(...) ReportVars(__VA_ARGS__ , ParseVaStrings(#__VA_ARGS__));


template <typename T, typename S>
const T& first(const T& t, const S& s) { return t; }
template <typename T, typename S>
const S& second(const T& t, const S& s) { return s; }

string firstname(const string& s) { return s.substr(0, s.find(",")); }
string secondname(const string& s) { return s.substr(s.find(",")+2); }

#define PRINTBOTH(...)																									\
	cout << firstname(#__VA_ARGS__) << ": " << first(__VA_ARGS__) << "\n"	\
	<< secondname(#__VA_ARGS__) << ": " << second(__VA_ARGS__) << endl		\

//	#define B s
//	#s


int main(int argc, char **argv) {
	string a = "some string";
	int b = 123;
	char c = 'x';
	PRINTALL(a, b, c);

	return 0;
}
