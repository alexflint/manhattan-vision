/*
 * histogram.cpp
 *
 *  Created on: 25 May 2010
 *      Author: alexf
 */

#include "histogram.tpp"
#include "common_types.h"

namespace indoor_context {

RgbLayout::RgbLayout() { }

RgbLayout::RgbLayout(int na) {
	Configure(na);
}

RgbLayout::RgbLayout(int nr, int ng, int nb) {
	Configure(nr, ng, nb);
}

void RgbLayout::Configure(int na) {
	Configure(na, na, na);
}

void RgbLayout::Configure(int nr, int ng, int nb) {
	nr_ = nr;
	ng_ = ng;
	nb_ = nb;
	n_ = nr*ng*nb;
}

int RgbLayout::GetBinCount() const {
	return n_;
}

int RgbLayout::GetBinIndex(const PixelRGB<byte>& p) const {
	return (int)(p.r*nr_)/256 + (int)(p.g*ng_/256)*nr_ + (int)(p.b*nb_/256)*nr_*ng_;
}

PixelRGB<byte> RgbLayout::GetBinExemplar(int bin) const {
	PixelRGB<byte> p;
	p.r = (0.5+(bin%nr_)) * 256/nr_;
	bin /= nr_;
	p.g = (0.5+(bin%ng_)) * 256/ng_;
	bin /= ng_;
	p.b = (0.5+(bin%nb_)) * 256/nb_;
	p.alpha = 0;
	return p;
}

}
