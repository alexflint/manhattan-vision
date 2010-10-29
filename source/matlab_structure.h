#include <mex.h>
#include "common_types.h"

namespace indoor_context {
	class MatlabProto;

	// Represents an immutable matlab struct array
	class ConstMatlabStructure {
	private:
		const mxArray* m;
	protected:
		map<string, int> fields;
	public:
		ConstMatlabStructure() : m(NULL) { }
		ConstMatlabStructure(const MatlabProto& proto, const mxArray* arr);

		const mxArray* get() const { return m; }

		void Configure(const MatlabProto& proto, const mxArray* arr);
		const mxArray* operator()(int index, const string& field) const;
	};

	// Represents a mutable matlab struct array
	class MatlabStructure : public ConstMatlabStructure {
	private:
		mxArray* m;
	public:
		MatlabStructure() : m(NULL) { }
		MatlabStructure(const MatlabProto& proto, mxArray* arr);
		MatlabStructure(const MatlabProto& proto, int length);

		mxArray* get() { return m; }
		const mxArray* get() const { return m; }

		void put(int index, const string& field, mxArray* p);

		mxArray* operator()(int index, const string& field);
		void Configure(const MatlabProto& proto, mxArray* arr);
		void New(const MatlabProto& proto, int length);
	};

	// Represents a set of field names for a matlab struct array
	class MatlabProto {
	public:
		vector<string> fields;
		typedef const string& s;
		MatlabProto(s);
		MatlabProto(s,s);
		MatlabProto(s,s,s);
		MatlabProto(s,s,s,s);
		MatlabProto(s,s,s,s,s);
		MatlabProto(s,s,s,s,s,s);
		MatlabProto(s,s,s,s,s,s,s);
		MatlabProto(s,s,s,s,s,s,s,s);

		ConstMatlabStructure From(const mxArray* p) const;
		MatlabStructure From(mxArray* p) const;
		MatlabStructure New(int size) const;
	};
}
