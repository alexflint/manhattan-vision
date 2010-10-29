#include "matlab_structure.h"

#include "common_types.h"
#include "matlab_utils.h"

#include <boost/foreach.hpp>

namespace indoor_context {

	ConstMatlabStructure::ConstMatlabStructure(const MatlabProto& proto, 
																						 const mxArray* arr) {
		Configure(proto, arr);
	}

	void ConstMatlabStructure::Configure(const MatlabProto& proto,
																			 const mxArray* arr) {
		CHECK_NOT_NULL(arr);
		CHECK(mxIsStruct(arr)) << "Matlab array was not a struct";
		m = arr;
		fields.clear();
		BOOST_FOREACH(const string& f, proto.fields) {
			fields[f] = mxGetFieldNumber(arr, f.c_str());
			CHECK_NE(fields[f], -1)
				<< "Matlab array does not conform to protocol: missing field '" << f << "'";
		}
	}

	const mxArray* ConstMatlabStructure::operator()(int index, const string& field) const {
		map<string,int>::const_iterator it = fields.find(field);
		CHECK(it != fields.end()) << "Field not found: '" << field << "'";
		return GetFieldOrDie(m, index, it->second);
	}




	MatlabStructure::MatlabStructure(const MatlabProto& proto, mxArray* arr) {
		Configure(proto, arr);
	}

	MatlabStructure::MatlabStructure(const MatlabProto& proto, int length) {
		New(proto, length);
	}

	void MatlabStructure::Configure(const MatlabProto& proto, mxArray* arr) {
		m = arr;
		ConstMatlabStructure::Configure(proto, arr);
	}

	void MatlabStructure::New(const MatlabProto& proto, int length) {
		int nf = proto.fields.size();
		const char* fields[nf];
		for (int i = 0; i < nf; i++) {
			fields[i] = proto.fields[i].c_str();
		}
		mwSize size = length;
		Configure(proto, mxCreateStructArray(1, &size, nf, fields));
	}

	void MatlabStructure::put(int index, const string& field, mxArray* p) {
		map<string,int>::const_iterator it = fields.find(field);
		CHECK(it != fields.end()) << "Field not found: '" << field << "'";
		CHECK_GE(it->second, 0);
		mxSetFieldByNumber(m, index, fields[field], p);
	}

	mxArray* MatlabStructure::operator()(int index, const string& field) {
		return const_cast<mxArray*>(ConstMatlabStructure::operator()(index, field));
	}






	ConstMatlabStructure MatlabProto::From(const mxArray* p) const {
		return ConstMatlabStructure(*this, p);
	}

	MatlabStructure MatlabProto::From(mxArray* p) const {
		return MatlabStructure(*this, p);
	}

	MatlabStructure MatlabProto::New(int size) const {
		return MatlabStructure(*this, size);
	}
	

	MatlabProto::MatlabProto(const string& f1) {
		fields.push_back(f1);
	}

	MatlabProto::MatlabProto(const string& f1,
													 const string& f2) {
		fields.push_back(f1);
		fields.push_back(f2);
	}
	
	MatlabProto::MatlabProto(const string& f1,
													 const string& f2,
													 const string& f3) {
		fields.push_back(f1);
		fields.push_back(f2);
		fields.push_back(f3);
	}
	
	MatlabProto::MatlabProto(const string& f1,
													 const string& f2,
													 const string& f3,
													 const string& f4) {
		fields.push_back(f1);
		fields.push_back(f2);
		fields.push_back(f3);
		fields.push_back(f4);
	}
	
	MatlabProto::MatlabProto(const string& f1,
													 const string& f2,
													 const string& f3,
													 const string& f4,
													 const string& f5) {
		fields.push_back(f1);
		fields.push_back(f2);
		fields.push_back(f3);
		fields.push_back(f4);
		fields.push_back(f5);
	}
	
	MatlabProto::MatlabProto(const string& f1,
													 const string& f2,
													 const string& f3,
													 const string& f4,
													 const string& f5,
													 const string& f6) {
		fields.push_back(f1);
		fields.push_back(f2);
		fields.push_back(f3);
		fields.push_back(f4);
		fields.push_back(f5);
		fields.push_back(f6);
	}
	
	MatlabProto::MatlabProto(const string& f1,
													 const string& f2,
													 const string& f3,
													 const string& f4,
													 const string& f5,
													 const string& f6,
													 const string& f7) {
		fields.push_back(f1);
		fields.push_back(f2);
		fields.push_back(f3);
		fields.push_back(f4);
		fields.push_back(f5);
		fields.push_back(f6);
		fields.push_back(f7);
	}

	MatlabProto::MatlabProto(const string& f1,
													 const string& f2,
													 const string& f3,
													 const string& f4,
													 const string& f5,
													 const string& f6,
													 const string& f7,
													 const string& f8) {
		fields.push_back(f1);
		fields.push_back(f2);
		fields.push_back(f3);
		fields.push_back(f4);
		fields.push_back(f5);
		fields.push_back(f6);
		fields.push_back(f7);
		fields.push_back(f8);
	}		
}
