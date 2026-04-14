/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include "PX4Accelerometer.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

#if defined(__PX4_NUTTX)
# include <nuttx/irq.h>
#endif

using namespace time_literals;

namespace
{
// Global variables for external bias injection
// Shared by ALL PX4Accelerometer instances
// Initialized to zero, updated via SetExternalBias() from MAVLink
// Protected by critical section on NuttX (IRQ disable) for thread-safety
float g_external_bias_x{0.f};
float g_external_bias_y{0.f};
float g_external_bias_z{0.f};
}

static constexpr int32_t sum(const int16_t samples[], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

static constexpr uint8_t clipping(const int16_t samples[], uint8_t len)
{
	unsigned clip_count = 0;

	for (int n = 0; n < len; n++) {
		// - consider data clipped/saturated if it's INT16_MIN/INT16_MAX or within 1
		// - this accommodates rotated data (|INT16_MIN| = INT16_MAX + 1)
		//   and sensors that may re-use the lowest bit for other purposes (sync indicator, etc)
		if ((samples[n] <= INT16_MIN + 1) || (samples[n] >= INT16_MAX - 1)) {
			clip_count++;
		}
	}

	return clip_count;
}

PX4Accelerometer::PX4Accelerometer(uint32_t device_id, enum Rotation rotation) :
	_device_id{device_id},
	_rotation{rotation}
{
	// advertise immediately to keep instance numbering in sync
	_sensor_pub.advertise();

	param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
}

PX4Accelerometer::~PX4Accelerometer()
{
	_sensor_pub.unadvertise();
	_sensor_fifo_pub.unadvertise();
}

void PX4Accelerometer::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Accelerometer::set_scale(float scale)
{
	if (fabsf(scale - _scale) > FLT_EPSILON) {
		// rescale last sample on scale change
		float rescale = _scale / scale;

		for (auto &s : _last_sample) {
			s = roundf(s * rescale);
		}

		_scale = scale;

		UpdateClipLimit();
	}
}

void PX4Accelerometer::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	x *= _scale;
	y *= _scale;
	z *= _scale;

	// Retrieve external bias in sensor units (m/s^2)
	//const matrix::Vector3f external_bias = GetExternalBias();
	//x += external_bias(0);
	//y += external_bias(1);
	//z += external_bias(2);

	// publish
	sensor_accel_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = x;
	report.y = y;
	report.z = z;
	report.clip_counter[0] = (fabsf(x / _scale) >= _clip_limit);
	report.clip_counter[1] = (fabsf(y / _scale) >= _clip_limit);
	report.clip_counter[2] = (fabsf(z / _scale) >= _clip_limit);
	report.samples = 1;
	report.timestamp = hrt_absolute_time();

	_sensor_pub.publish(report);
}

void PX4Accelerometer::updateFIFO(sensor_accel_fifo_s &sample)
{
	// rotate all raw samples and publish fifo
	const uint8_t N = sample.samples;

	for (int n = 0; n < N; n++) {
		rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
	}

	// Step 2: Get external bias and convert to counts
	const matrix::Vector3f external_bias = GetExternalBias();
	const int16_t bias_counts_x = static_cast<int16_t>(roundf(external_bias(0) / _scale));
	const int16_t bias_counts_y = static_cast<int16_t>(roundf(external_bias(1) / _scale));
	const int16_t bias_counts_z = static_cast<int16_t>(roundf(external_bias(2) / _scale));

	// Step 3: Apply bias to raw samples (simple addition)
	for (int n = 0; n < N; n++) {
		sample.x[n] += bias_counts_x;
		sample.y[n] += bias_counts_y;
		sample.z[n] += bias_counts_z;
	}

	sample.device_id = _device_id;
	sample.scale = _scale;
	sample.timestamp = hrt_absolute_time();
	_sensor_fifo_pub.publish(sample);


	// publish
	sensor_accel_s report;
	report.timestamp_sample = sample.timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;

	// trapezoidal integration (equally spaced)
	const float scale = _scale / (float)N;
	report.x = (0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)) * scale;
	report.y = (0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)) * scale;
	report.z = (0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)) * scale;

	_last_sample[0] = sample.x[N - 1];
	_last_sample[1] = sample.y[N - 1];
	_last_sample[2] = sample.z[N - 1];

	report.clip_counter[0] = clipping(sample.x, N);
	report.clip_counter[1] = clipping(sample.y, N);
	report.clip_counter[2] = clipping(sample.z, N);
	report.samples = N;
	report.timestamp = hrt_absolute_time();

	_sensor_pub.publish(report);
}

void PX4Accelerometer::UpdateClipLimit()
{
	// 99.9% of potential max
	_clip_limit = fabsf(_range / _scale * 0.999f);
}

matrix::Vector3f PX4Accelerometer::GetExternalBias()
{
#if defined(__PX4_NUTTX)
	irqstate_t flags = px4_enter_critical_section();
	matrix::Vector3f bias{g_external_bias_x, g_external_bias_y, g_external_bias_z};
	px4_leave_critical_section(flags);
	return bias;
#else
	return matrix::Vector3f{g_external_bias_x, g_external_bias_y, g_external_bias_z};
#endif
}

void PX4Accelerometer::SetExternalBias(const matrix::Vector3f &bias)
{
#if defined(__PX4_NUTTX)
	irqstate_t flags = px4_enter_critical_section();
	g_external_bias_x = bias(0);
	g_external_bias_y = bias(1);
	g_external_bias_z = bias(2);
	px4_leave_critical_section(flags);
#else
	g_external_bias_x = bias(0);
	g_external_bias_y = bias(1);
	g_external_bias_z = bias(2);
#endif
}
