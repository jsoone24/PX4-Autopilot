#ifndef GET_GYRO_BIAS_HPP
#define GET_GYRO_BIAS_HPP

#include <uORB/topics/gyro_bias.h>

class MavlinkStreamGetGyroBias : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGetGyroBias(mavlink); }
	const char *get_name() const override { return get_name_static(); }
	static const char *get_name_static() { return "GET_GYRO_BIAS"; }
	uint16_t get_id() override { return get_id_static(); }
	static uint16_t get_id_static() { return MAVLINK_MSG_ID_GET_GYRO_BIAS; }
	unsigned get_size() override { return MAVLINK_MSG_ID_GET_GYRO_BIAS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

protected:
	explicit MavlinkStreamGetGyroBias(Mavlink *mavlink) : MavlinkStream(mavlink) {}


	bool send() override
	{
		gyro_bias_s bias;

		if (_gyro_bias_sub.copy(&bias)) {
			mavlink_get_gyro_bias_t info_msg{};

			info_msg.gyro_bias_x = bias.gyro_bias_x;
			info_msg.gyro_bias_y = bias.gyro_bias_y;
			info_msg.gyro_bias_z = bias.gyro_bias_z;

			mavlink_msg_get_gyro_bias_send_struct(_mavlink->get_channel(), &info_msg);
			return true;
		}

		return false;
	}

private:
	uORB::Subscription _gyro_bias_sub{ORB_ID(gyro_bias)};
	MavlinkStreamGetGyroBias(const MavlinkStreamGetGyroBias &) = delete;
	MavlinkStreamGetGyroBias &operator=(const MavlinkStreamGetGyroBias &) = delete;
};

#endif // GET_GYRO_BIAS_HPP
