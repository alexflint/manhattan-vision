#pragma once

#include <numeric>
#include <ext/numeric>
#include <stack>

#include "common_types.h"

namespace indoor_context {

// Implementation of the union-find algorithm. Manages merging of
// groups in ammortized logarithmic time.
class UnionFind {
public:
	// This vector is mutable to allow us to optimize GetGroup, but the
	// const semantics are retained.
	mutable VecI parents;

	// The number of groups (decreases as Merge is called)
	int num_groups;

	// Initialize a union-find for N objects
	UnionFind(int n = 0);

	// Resets the first M elements to belong to their own group
	void Reset(int m);

	// Returns the group ID for the v-th object. Two objects have the
	// same group ID if and only if they are in the same group.
	int GetGroup(int v) const;

	// Merges the group to which A belongs with the group to which B
	// belongs. Returns the ID of the group to which both objects now
	// belong.
	int Merge(int a, int b);

	// Returns true if A and B are in the same group
	bool Joined(int a, int b) const;

	// Get the number of groups remaining
	inline int NumGroups() const { return num_groups; }
};

}
