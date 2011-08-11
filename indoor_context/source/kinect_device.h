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


#include "common_types.h"
#include "libfreenect.hpp"
#include "image_bundle.h"

#include <boost/thread.hpp>

namespace indoor_context {
	class KinectDevice;

	// Represents a handle to the kinect driver. Only one instance of
	// this should exist. In its constructor it starts a thread and in
	// its destructor it stops that thread, so it is important that this
	// object goes out of scope when the device is finished with.
	class KinectDriver {
	public:
		KinectDriver() : m_device(NULL) { }
		~KinectDriver();
		KinectDevice& device();
	private:
		KinectDevice* m_device;
		Freenect::Freenect m_driver;
	};

	// For accessing the kinect
	class KinectDevice : public Freenect::FreenectDevice {
	public:
		// Get singleton instance of this class
		static KinectDevice& instance();
		// Get singleton instance of the manager
		static Freenect::Freenect& driver_instance();

		// Constructor (_must_ have this signature as it is assumed by Freenect::createDevice
		KinectDevice(freenect_context *_ctx, int _index);
		// start recieving video and depth
		void start();
		// stop recieving video and depth
		void stop();
		// Get RGB data, or return false if there is no new frame
		bool getRGB(std::vector<uint8_t>& buffer);
		bool getRGB(ImageRGB<byte>& image);
		// Wait for the next RGB frame
		void waitForRGB(ImageRGB<byte>& image);
		// Get depth data
		bool getDepth(std::vector<uint16_t>& buffer);
		bool getDepth(MatF& depth);
		// Wait for the next RGB frame
		void waitForDepth(MatF& depth);
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp);
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp);
		// Get size of returned image
		inline int image_width() const { return m_nx; }
		inline int image_height() const { return m_ny; }
	private:
		int m_nx;
		int m_ny;
		std::vector<uint16_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_video;
		std::vector<uint16_t> m_gamma;
		boost::mutex m_rgb_mutex;
		boost::mutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
	};
}
