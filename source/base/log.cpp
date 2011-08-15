#include <unistd.h>
#include <algorithm>

#include <boost/version.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/operations.hpp>
#include <boost/iostreams/categories.hpp>

#include "common_types.h"

#include "io_utils.tpp"
#include "log.tpp"

namespace indoor_context {
using namespace boost;
namespace ios=boost::iostreams;

// Constants
static const int kLogFileNo = STDOUT_FILENO;

// Special tokens for the interface betwen Scoped* and *Filter
struct SpecialLogTokens {
	static const int kNullToken = -1;
	static const int kEndToken = -2;
	static const int kNewline = '\n';
};


// Wrapper for sinks
class SinkWrapper {
public:
	typedef char char_type;
	typedef boost::iostreams::sink_tag category;
	GenericCharSink* inner;
	SinkWrapper(GenericCharSink* sink) : inner(sink) {
	}
	streamsize write(const char* s, streamsize n) {
		return inner->write(s,n);
	}
};


// Ends the current line whenever kNewline is recieved, unless the
// line has already be ended.
class AutoNewlineFilter : public ios::output_filter {
public:
	shared_ptr<int> last;  // this is a shared_ptr so all copies are connected
	inline AutoNewlineFilter() : last(new int(SpecialLogTokens::kNewline)) { }

	// Output a char to the stream
	template <typename Sink>
	bool put(Sink& dest, int c)  {
		if (c == SpecialLogTokens::kEndToken) {
			if (*last == SpecialLogTokens::kNewline) {
				return true;
			} else {
				*last = SpecialLogTokens::kNewline;
				return ios::put(dest, SpecialLogTokens::kNewline);
			}
		} else {
			// if c < 0 then we must have a special indentation token, which
			// should be passed through the stream, but ignored for purposes
			// of deciding when to end the line.
			if (c >= 0) {
				*last = c;
			}
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

// The singleton log stream -- TODO: should this really be a global variable?
scoped_ptr<ios::filtering_ostream> log_stream;

// Set the sink for the log stream
template <typename Sink>
void SetLogSinkImpl(const Sink& sink) {
	// TODO: only reset the stream on first creation, use filtering_ostream::pop() thereafter
	// Note: filtering_stream copies its argument so temp objs are fine
	log_stream.reset(new ios::filtering_ostream);
	log_stream->push(AutoNewlineFilter());
	log_stream->push(IndentFilter());
	log_stream->push(sink);
}

// This function is not exposed in log.tpp for efficient compilation
ios::filtering_ostream& GetLogStreamImpl() {
	if (log_stream.get() == NULL) {
#if BOOST_VERSION >= 104400
		SetLogSinkImpl(ios::file_descriptor_sink(kLogFileNo, ios::never_close_handle));
#else
		SetLogSinkImpl(ios::file_descriptor_sink(kLogFileNo, false));
#endif
	}
	return *log_stream;
}

ostream& LogManager::GetLogStream() {
	return GetLogStreamImpl();
}

void LogManager::SetLogSink(GenericCharSink* sink) {
	static scoped_ptr<GenericCharSink> p;
	p.reset(sink);
	SetLogSinkImpl(SinkWrapper(sink));
}

bool LogManager::Flush() {
	return GetLogStreamImpl().strict_sync();
}

void LogManager::EndCurrentLine() {
	GetLogStream().put(SpecialLogTokens::kEndToken);
	GetLogStream() << flush;	
}

LogManager::DelayedNewline::~DelayedNewline() {
	s.put(SpecialLogTokens::kEndToken);
	s << flush;
}

}
