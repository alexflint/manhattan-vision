#pragma once

#include "canvas.h"

#include <boost/range.hpp>

#include <cairommconfig.h>
#include <cairomm/context.h>
#include <cairomm/surface.h>

#include "common_types.h"
#include "polygon.tpp"

namespace indoor_context {
	namespace {
		using namespace boost;
		using namespace toon;
	}

	template <typename T>
	void Canvas::Attach(T surface) {
    c_ = Cairo::Context::create(surface);
	}
	
	template <typename Range>
	void Canvas::AddPath(const Range& poly) {
		if (!empty(poly)) {
			typename range_const_iterator<Range>::type it = begin(poly);
			typename range_const_iterator<Range>::type endit = end(poly);
			for (MoveTo(*it); it != endit; it++) {
				LineTo(*it);
			}
		}
	}


	template <typename Range>
	void Canvas::FillPolygon(const Range& poly) {
		AddPath(poly);
		c_->fill();
	}

	template <typename Range>
	void Canvas::FillPolygon(const Range& poly, const PixelRGB<byte>& color) {
		SetColor(color);
		FillPolygon(poly);
	}

	template <unsigned N>
	void Canvas::FillPolygon(const Polygon<N>& poly) {
		FillPolygon(poly.verts);
	}

	template <unsigned N>
	void Canvas::FillPolygon(const Polygon<N>& poly, const PixelRGB<byte>& color) {
		FillPolygon(poly.verts, color);
	}




	template <typename Range>
	void Canvas::StrokePolygon(const Range& poly) {
		AddPath(poly);
		c_->stroke();
	}

	template <typename Range>
	void Canvas::StrokePolygon(const Range& poly, const PixelRGB<byte>& color) {
		SetColor(color);
		StrokePolygon(poly);
	}

	template <unsigned N>
	void Canvas::StrokePolygon(const Polygon<N>& poly) {
		StrokePolygon(poly.verts);
	}

	template <unsigned N>
	void Canvas::StrokePolygon(const Polygon<N>& poly, const PixelRGB<byte>& color) {
		StrokePolygon(poly.verts, color);
	}
}
