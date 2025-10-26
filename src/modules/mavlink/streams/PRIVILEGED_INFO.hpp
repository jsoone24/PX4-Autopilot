#ifndef PRIVILEGED_INFO_HPP
#define PRIVILEGED_INFO_HPP

#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

class MavlinkStreamPrivilegedInfo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamPrivilegedInfo(mavlink); }
	const char *get_name() const override { return get_name_static(); }
	static const char *get_name_static() { return "PRIVILEGED_INFO"; }
	uint16_t get_id() override { return get_id_static(); }
	static uint16_t get_id_static() { return MAVLINK_MSG_ID_PRIVILEGED_INFO; }
	unsigned get_size() override { return MAVLINK_MSG_ID_PRIVILEGED_INFO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

protected:
	explicit MavlinkStreamPrivilegedInfo(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	bool send() override
	{
		estimator_sensor_bias_s bias;
		estimator_innovations_s innovations;
		estimator_innovations_s innovations_var;
		estimator_status_s status;
		vehicle_rates_setpoint_s rates;
		vehicle_attitude_setpoint_s attitude;
		vehicle_local_position_setpoint_s local_position;

		if (_estimator_sensor_bias_sub.copy(&bias) &&
		    _estimator_innovations_sub.copy(&innovations) &&
		    _estimator_innovations_var_sub.copy(&innovations_var) &&
		    _estimator_status_sub.copy(&status) &&
		    _vehicle_rates_setpoint_sub.copy(&rates) &&
		    _vehicle_attitude_setpoint_sub.copy(&attitude) &&
		    _vehicle_local_position_setpoint_sub.copy(&local_position)) {

			mavlink_privileged_info_t info_msg{};

			info_msg.ekf_gyro_bias[0] = bias.gyro_bias[0];
			info_msg.ekf_gyro_bias[1] = bias.gyro_bias[1];
			info_msg.ekf_gyro_bias[2] = bias.gyro_bias[2];

			info_msg.ekf_gyro_bias_variance[0] = bias.gyro_bias_variance[0];
			info_msg.ekf_gyro_bias_variance[1] = bias.gyro_bias_variance[1];
			info_msg.ekf_gyro_bias_variance[2] = bias.gyro_bias_variance[2];

			info_msg.ekf_accel_bias[0] = bias.accel_bias[0];
			info_msg.ekf_accel_bias[1] = bias.accel_bias[1];
			info_msg.ekf_accel_bias[2] = bias.accel_bias[2];

			info_msg.ekf_accel_bias_variance[0] = bias.accel_bias_variance[0];
			info_msg.ekf_accel_bias_variance[1] = bias.accel_bias_variance[1];
			info_msg.ekf_accel_bias_variance[2] = bias.accel_bias_variance[2];

			for (int i = 0; i < 2; i++) {
				info_msg.gps_hpos_innov[i] = innovations.gps_hpos[i];
				info_msg.gps_hvel_innov[i] = innovations.gps_hvel[i];
				info_msg.gps_hpos_innov_var[i] = innovations_var.gps_hpos[i];
				info_msg.gps_hvel_innov_var[i] = innovations_var.gps_hvel[i];
			}

			info_msg.gps_vpos_innov = innovations.gps_vpos;
			info_msg.gps_vvel_innov = innovations.gps_vvel;
			info_msg.heading_innov = innovations.heading;

			info_msg.gps_vpos_innov_var = innovations_var.gps_vpos;
			info_msg.gps_vvel_innov_var = innovations_var.gps_vvel;
			info_msg.heading_innov_var = innovations_var.heading;

			info_msg.pos_test_ratio = status.pos_test_ratio;
			info_msg.vel_test_ratio = status.vel_test_ratio;
			info_msg.mag_test_ratio = status.mag_test_ratio;

			info_msg.pos_horiz_accuracy = status.pos_horiz_accuracy;
			info_msg.pos_vert_accuracy = status.pos_vert_accuracy;

			info_msg.local_position_setpoint_p[0] = local_position.x;
			info_msg.local_position_setpoint_p[1] = local_position.y;
			info_msg.local_position_setpoint_p[2] = local_position.z;

			info_msg.local_position_setpoint_v[0] = local_position.vx;
			info_msg.local_position_setpoint_v[1] = local_position.vy;
			info_msg.local_position_setpoint_v[2] = local_position.vz;

			info_msg.attitude_setpoint_q[0] = attitude.q_d[0];
			info_msg.attitude_setpoint_q[1] = attitude.q_d[1];
			info_msg.attitude_setpoint_q[2] = attitude.q_d[2];
			info_msg.attitude_setpoint_q[3] = attitude.q_d[3];

			info_msg.rates_setpoint[0] = rates.roll;
			info_msg.rates_setpoint[1] = rates.pitch;
			info_msg.rates_setpoint[2] = rates.yaw;

			info_msg.attitude_setpoint_thrust[0] = attitude.thrust_body[0];
			info_msg.attitude_setpoint_thrust[1] = attitude.thrust_body[1];
			info_msg.attitude_setpoint_thrust[2] = attitude.thrust_body[2];

			mavlink_msg_privileged_info_send_struct(_mavlink->get_channel(), &info_msg);
			return true;
		}

		return false;
	}

private:
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _estimator_innovations_sub{ORB_ID(estimator_innovations)};
	uORB::Subscription _estimator_innovations_var_sub{ORB_ID(estimator_innovation_variances)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};

	MavlinkStreamPrivilegedInfo(const MavlinkStreamPrivilegedInfo &) = delete;
	MavlinkStreamPrivilegedInfo &operator=(const MavlinkStreamPrivilegedInfo &) = delete;
};

#endif // PRIVILEGED_INFO_HPP
