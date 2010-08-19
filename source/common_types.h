#pragma once

#include <string>
#include <vector>
#include <utility>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/lexical_cast.hpp>

#include <VNL/vector.h>
#include <VNL/vectorfixed.h>
#include <VNL/matrix.h>
#include <VNL/matrixfixed.h>

#include <VW/Image/imagergb.h>
#include <VW/Image/imagehsv.h>
#include <VW/Image/imagemono.h>
#include <VW/Image/imageio.h>

#include <TooN/TooN.h>
namespace toon=TooN;

#include <gvars3/instances.h>

namespace boost { namespace filesystem { } }

namespace indoor_context {
using namespace std;
using namespace boost;
namespace fs=boost::filesystem;

typedef unsigned char byte;

using VW::ImageRGB;
using VW::ImageHSV;
using VW::ImageMono;
using VW::ImageRef;
using VW::PixelRGB;
using VW::PixelHSV;
using VW::PixelMono;

typedef toon::Vector<2> Vec2;
typedef toon::Vector<3> Vec3;
typedef toon::Vector<4> Vec4;
typedef toon::Vector<5> Vec5;
typedef toon::Vector<6> Vec6;

typedef toon::Vector<2,int> Vec2I;
typedef toon::Vector<3,int> Vec3I;
typedef toon::Vector<4,int> Vec4I;
typedef toon::Vector<5,int> Vec5I;
typedef toon::Vector<6,int> Vec6I;

typedef toon::Matrix<2> Mat2;
typedef toon::Matrix<3> Mat3;
typedef toon::Matrix<4> Mat4;

typedef VNL::Matrix<int> MatI;
typedef VNL::Matrix<float> MatF;
typedef VNL::Matrix<double> MatD;

typedef VNL::Vector<int> VecI;
typedef VNL::Vector<float> VecF;
typedef VNL::Vector<double> VecD;

typedef VW::ImageMono<float> ImageF;
typedef VW::ImageMono<double> ImageD;
typedef VW::PixelMono<float> PixelF;
typedef VW::PixelMono<double> PixelD;

} // indoor_context

// These rely on the above typedefs...
#include "log.tpp"
#include "check.tpp"
#include "lazyvar.h"
#include "image_bundle.h"
#include "initialized_ptr.tpp"
