#ifndef GET_ACCEL_BIAS_HPP
#define GET_ACCEL_BIAS_HPP

#include <uORB/topics/accel_bias.h>

class MavlinkStreamGetAccelBias : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGetAccelBias(mavlink); }
	const char *get_name() const override { return get_name_static(); }
	static const char *get_name_static() { return "GET_ACCEL_BIAS"; }
	uint16_t get_id() override { return get_id_static(); }
	static uint16_t get_id_static() { return MAVLINK_MSG_ID_GET_ACCEL_BIAS; }
	unsigned get_size() override { return MAVLINK_MSG_ID_GET_ACCEL_BIAS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

protected:
	explicit MavlinkStreamGetAccelBias(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	bool send() override
	{
		accel_bias_s bias;

		if (_accel_bias_sub.copy(&bias)) {
			mavlink_get_accel_bias_t info_msg{};

			info_msg.accel_bias_x = bias.accel_bias_x;
			info_msg.accel_bias_y = bias.accel_bias_y;
			info_msg.accel_bias_z = bias.accel_bias_z;

			mavlink_msg_get_accel_bias_send_struct(_mavlink->get_channel(), &info_msg);
			return true;
		}

		return false;
	}

private:
	uORB::Subscription _accel_bias_sub{ORB_ID(accel_bias)};
	MavlinkStreamGetAccelBias(const MavlinkStreamGetAccelBias &) = delete;
	MavlinkStreamGetAccelBias &operator=(const MavlinkStreamGetAccelBias &) = delete;
};

#endif // GET_ACCEL_BIAS_HPP
