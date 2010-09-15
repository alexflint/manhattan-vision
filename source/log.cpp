#include <unistd.h>
#include <algorithm>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/operations.hpp>

#include "log.tpp"
#include "common_types.h"

namespace indoor_context {
namespace ios=boost::iostreams;

// Constants
static const int kLogFileNo = STDOUT_FILENO;

// Special tokens for the interface betwen Scoped* and *Filter
struct SpecialLogTokens {
	static const int kNullToken = -1;
	static const int kEndToken = -2;
	static const int kNewline = '\n';
};

// Ends the current line whenever kNewline is
// recieved, unless the line has already be ended.
class AutoNewlineFilter : public ios::output_filter {
public:
	shared_ptr<int> last;  // this is a shared_ptr so all copies are connected
	inline AutoNewlineFilter() : last(new int(SpecialLogTokens::kNullToken)) { }

	// Output a char to the stream
	template <typename Sink>
	bool put(Sink& dest, int c)  {
		if (c == SpecialLogTokens::kEndToken) {
			if (*last == SpecialLogTokens::kNewline) {
				*last = SpecialLogTokens::kNullToken;
				return true;
			} else {
				*last = SpecialLogTokens::kNullToken;
				return ios::put(dest, SpecialLogTokens::kNewline);
			}
		} else {
			*last = c;
			return ios::put(dest, c);
		}
	}

	// Close the stream
	template<typename Sink>
	void close(Sink&) {
		*last = SpecialLogTokens::kNullToken;
	}
};

// Inserts indentation chars at the beginning of a line
class IndentFilter : public ios::output_filter {
public:
	int cur_indent;
	bool at_line_start;
	inline IndentFilter() : at_line_start(true), cur_indent(0) { }

	template <typename Sink>
	bool put(Sink& dest, int c) {
		// Do not read from LogManager::GetIndentLevel() here because
		// internal caching means that this gets called out-of-sync with
		// the indent changes
		if (c <= -10) {
			cur_indent = -10-c;
			return true;
		} else {
			if (c == '\n') {
				at_line_start = true;
			} else if (at_line_start) {
				for (int i = 0; i < cur_indent; i++) {
					ios::put(dest, ' ');
				}
				at_line_start = false;
			}
			return ios::put(dest, c);
		}
	}
};

// Static vars
bool LogManager::enabled = true;
int LogManager::indent_level = 0;

// Static
void LogManager::SetIndent(int newlevel) {
	GetLogStream().put(-10-newlevel);
	indent_level = newlevel;
}

LogManager::ScopedIndenter::ScopedIndenter(int lines) : nlines(lines) {
	LogManager::SetIndent(LogManager::indent_level + kIndentIncrement);
}

LogManager::ScopedIndenter::~ScopedIndenter() {
	LogManager::SetIndent(LogManager::indent_level - kIndentIncrement);
	if (LogManager::enabled) {
		for (int i = 0; i < nlines; i++) {
			LOG_STREAM << "\n";
		}
	}
}

LogManager::ScopedEnabler::ScopedEnabler(bool newstate)
: oldstate(LogManager::enabled) {
	LogManager::enabled = newstate;
}

LogManager::ScopedEnabler::~ScopedEnabler() {
	LogManager::enabled = oldstate;
}

// This function is not a member of LogManager because we do not want to expose
// boost::isotreams in log.tpp, since that would slow down compile time considerably.
ios::filtering_ostream& GetLogStreamImpl() {
	static scoped_ptr<ios::filtering_ostream> stream;
	static AutoNewlineFilter newline_filter;
	static IndentFilter indent_filter;
	if (stream.get() == NULL) {
		stream.reset(new ios::filtering_ostream);
		stream->push(newline_filter);
		stream->push(indent_filter);
		stream->push(ios::file_descriptor_sink(kLogFileNo, ios::never_close_handle));
	}
	return *stream;
}

ostream& LogManager::GetLogStream() {
	return GetLogStreamImpl();
}

bool LogManager::Flush() {
	return GetLogStreamImpl().strict_sync();
}

LogManager::DelayedNewline::~DelayedNewline() {
	s.put(SpecialLogTokens::kEndToken);
	s << flush;
}

vector<string> LogManager::ParseVaExprs(const string& s) {
	// We are essentially parsing C++ expressions here... uh oh.
	// The logic I apply is that commas can only appear in function
	// calls (or in for loops but they don't have a value so can't be
	// passed in here), and they must be surrounded by parentheses, so
	// we should only need to track parentheses, not other types of
	// brackets

	vector<string> vs;
	int a = 0, depth = 0;
	for (int i = 0; i <= s.length(); i++) {
		if (i == s.length() || (depth == 0 && s[i] == ',')) {
			vs.push_back(s.substr(a, i-a));
			a = i+1;
			while (a < s.length() && s[a] == ' ') a++;
		} else if (s[i] == '(') {
			depth++;
		} else if (s[i] == ')') {
			depth--;
		}
	}
	return vs;
}

}
