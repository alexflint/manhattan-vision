#pragma once

#include "common_types.h"

namespace indoor_context {

	// Represents an image in which counting the number of some item in
	// any interval of any column is an O(1) operation. The number of
	// different items is specified by the template parameter N.
	template <unsigned N>
	class IntegralColImage {
	public:
		MatI mint[N];

		// Initialize empty
		IntegralColImage() { }
		// Initialize and compute an integral image
		IntegralColImage(const MatI& m) {
			Compute(m);
		}

		// Compute the integral image: O(N)
		void Compute(const MatI& m) {
			// Allocate and copy the first row
			for (int i = 0; i < N; i++) {
				mint[i].Resize(m.Rows()+1, m.Cols(), 0);
				mint[i].Fill(0);
			}

			// Build the integral-col images
			for (int y = 0; y < m.Rows(); y++) {
				const int* inrow = m[y];
				for (int i = 0; i < N; i++) {
					const int* prevrow = mint[i][y];
					int* outrow = mint[i][y+1];
					for (int x = 0; x < m.Cols(); x++) {
						outrow[x] = (inrow[x] == i) ? prevrow[x]+1 : prevrow[x];
					}
				}
			}
		}

		// Count the number of times a specified item appears within rows
		// [row0,row1] of a particular column. This includes items in both
		// row0 and row1.
		int Count(int item, int col, int row0, int row1) const {
			/*CHECK_LT(item, N);
			CHECK_GE(item, 0);
			CHECK_LE(row0, row1);*/
			return mint[item][row1+1][col] - mint[item][row0][col];
		}
	};
}
