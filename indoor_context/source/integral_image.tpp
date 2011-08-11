#pragma once

#include "common_types.h"

namespace indoor_context {
	// Represents an integral image
	template <typename T>
	class IntegralImage {
	public:
		VNL::Matrix<T> m_int;

		// Initialize empty
		IntegralImage() { }
		// Initialize and compute an integral image
		IntegralImage(const VNL::Matrix<T>& m) {
			Compute(m);
		}

		// Get the size
		inline int nx() const { return m_int.Cols(); }
		inline int ny() const { return m_int.Rows(); }

		// Compute the integral image: O(N)
		template <typename S>
		void Compute(const VNL::Matrix<S>& m) {
			// Allocate and copy the first row
			m_int.Resize(m.Rows()+1, m.Cols()+1, 0);

			// Build the integral-col images
			for (int y = 0; y < m.Rows(); y++) {
				const S* inrow = m[y];
				const T* prevrow = m_int[y];
				T* outrow = m_int[y+1];
				T rowsum = 0;
				for (int x = 0; x < m.Cols(); x++) {
					rowsum += inrow[x];
					outrow[x+1] = prevrow[x+1]+rowsum;
				}
			}
		}

		// Sum over the rectangular region .
		T Sum(const Vec2I& tl, const Vec2I& br) const {
			T a = m_int[ tl[1]   ][ tl[0]   ];
			T b = m_int[ br[1]+1 ][ tl[0]   ];
			T c = m_int[ tl[1]   ][ br[0]+1 ];
			T d = m_int[ br[1]+1 ][ br[0]+1 ];
			return a - b - c + d;
		}
	};


	// Represents an image in which counting the number of some item in
	// any interval of any column is an O(1) operation.
	template <typename T>
	class IntegralColImage {
	public:
		VNL::Matrix<T> m_int;

		// Initialize empty
		IntegralColImage() { }
		// Initialize and compute an integral image
		IntegralColImage(const VNL::Matrix<T>& m) {
			Compute(m);
		}

		// Get the size
		inline int nx() const { return m_int.Cols(); }
		inline int ny() const { return m_int.Rows(); }

		// Compute the integral image: O(N)
		template <typename S>
		void Compute(const VNL::Matrix<S>& m) {
			// Allocate and copy the first row
			m_int.Resize(m.Rows()+1, m.Cols(), 0);

			// Build the integral-col images
			for (int y = 0; y < m.Rows(); y++) {
				const S* inrow = m[y];
				const T* prevrow = m_int[y];
				T* outrow = m_int[y+1];
				for (int x = 0; x < m.Cols(); x++) {
					outrow[x] = prevrow[x]+inrow[x];
				}
			}
		}

		// Sum over the the rows r0...r1 within a particular column. The sum
		// includes the values both in row r0 and row r1.
		T Sum(int col, int r0, int r1) const {
			return m_int[r1+1][col] - m_int[r0][col];
		}
	};
}
