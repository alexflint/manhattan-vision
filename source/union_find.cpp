#include <ext/numeric>

#include "union_find.h"
#include "common_types_vw.h"
#include "misc.h"

namespace indoor_context {

UnionFind::UnionFind(int n) : num_groups(n), parents(n) {
	Reset(n);
}

void UnionFind::Reset(int m) {
	num_groups = m;
	if (parents.size() < m) {
		parents.Resize(m);
	}
	for (int i = 0; i < m; i++) {
		parents[i] = i;
	}
}

int UnionFind::GetGroup(int v) const {
	// The fastest implementation of this function depends on the target
	// application. If the tree is deep then we should keep a stack and
	// update the entire path after we find the leader. If the tree is
	// shallow then we should update each node to point one parent
	// further on. Currently we use the latter implementation. In the
	// agglomerate segmentation algorithm this change decreased
	// execution time from 260ms to 4ms!
	while (parents[v] != v) {
		const int a = parents[v];
		parents[v] = parents[a];
		v = a;
	}
	//while (parents[v] != v) {
	//	trace.push(v);
	//	v = parents[v];
	//}
	//while (!trace.empty()) {
	//	parents[trace.top()] = v;
	//	trace.pop();
	//}
	return v;
}

int UnionFind::Merge(int a, int b) {
	const int ga = GetGroup(a);
	const int gb = GetGroup(b);
	if (ga != gb) {
		parents[gb] = ga;
		num_groups--;
	}
	return ga;
}

bool UnionFind::Joined(int a, int b) const {
	return GetGroup(a) == GetGroup(b);
}

}
