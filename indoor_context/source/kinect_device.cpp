/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include "kinect_device.h"
#include "libfreenect.hpp"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>

namespace indoor_context {
	KinectDevice& KinectDriver::device() {
		if (m_device == NULL) {
			m_device = &m_driver.createDevice<KinectDevice>(0);
		}
		return *m_device;
	}

	KinectDriver::~KinectDriver() {
		CHECK_EQ(m_driver.deviceCount(), m_device?1:0)
			<< "Warning: There are Kinect devices in scope other than the one owned by KinectDriver";
		if (m_device != NULL) {
			m_driver.deleteDevice(0);
		}
	}

	KinectDevice::KinectDevice(freenect_context *_ctx, int _index)
		: Freenect::FreenectDevice(_ctx, _index),
			m_buffer_depth(FREENECT_VIDEO_RGB_SIZE),
			m_buffer_video(FREENECT_VIDEO_RGB_SIZE),
			m_gamma(2048),
			m_new_rgb_frame(false),
			m_new_depth_frame(false),
			m_nx(640),
			m_ny(480) {
		for( unsigned int i = 0 ; i < 2048 ; i++) {
			float v = i/2048.0;
			v = std::pow(v, 3)* 6;
			m_gamma[i] = v*6*256;
		}
	}

	void KinectDevice::start() {
		startVideo();
		startDepth();
	}

	void KinectDevice::stop() {
		stopVideo();
		stopDepth();
	}

	void KinectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
		boost::mutex::scoped_lock lock(m_rgb_mutex);
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		std::copy(rgb, rgb+FREENECT_FRAME_PIX*3, m_buffer_video.begin());
		m_new_rgb_frame = true;
	};

	void KinectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
		boost::mutex::scoped_lock lock(m_depth_mutex);
		uint16_t* depth = static_cast<uint16_t*>(_depth);
		std::copy(depth, depth+FREENECT_FRAME_PIX, m_buffer_depth.begin());
		m_new_depth_frame = true;
	}

	bool KinectDevice::getRGB(std::vector<uint8_t>& buffer) {
		boost::mutex::scoped_lock lock(m_rgb_mutex);
		if(m_new_rgb_frame) {
			buffer.swap(m_buffer_video);
			m_new_rgb_frame = false;
			return true;
		} else {
			return false;
		}
	}

	bool KinectDevice::getRGB(ImageRGB<byte>& out) {
		boost::mutex::scoped_lock lock(m_rgb_mutex);
		if (m_new_rgb_frame) {
			int ii = 0;
			out.AllocImageData(m_nx, m_ny);
			for (int y = 0; y < m_ny; y++) {
				PixelRGB<byte>* row = out[y];
				for (int x = 0; x < m_nx; x++) {
					row[x].Set(m_buffer_video[ii*3], m_buffer_video[ii*3+1], m_buffer_video[ii*3+2]);
					ii++;
				}
			}
			return true;
		} else {
			return false;
		}
	}

	void KinectDevice::waitForRGB(ImageRGB<byte>& out) {
		while (!getRGB(out)) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(5));
		}
	}

	bool KinectDevice::getDepth(std::vector<uint16_t>& buffer) {
		boost::mutex::scoped_lock lock(m_depth_mutex);
		if(m_new_depth_frame) {
			buffer.swap(m_buffer_depth);
			m_new_depth_frame = false;
			return true;
		} else {
			return false;
		}
	}

	bool KinectDevice::getDepth(MatF& out) {
		boost::mutex::scoped_lock lock(m_depth_mutex);
		if(m_new_depth_frame) {
			out.Resize(m_ny, m_nx);
			int ii = 0;
			for (int y = 0; y < m_ny; y++) {
				float* row = out[y];
				for (int x = 0; x < m_nx; x++) {
					row[x] = m_buffer_depth[ii++];
				}
			}
			m_new_depth_frame = false;
			return true;
		} else {
			return false;
		}
	}

	void KinectDevice::waitForDepth(MatF& out) {
		while (!getDepth(out)) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(5));
		}
	}

} // namespace indoor_conext
