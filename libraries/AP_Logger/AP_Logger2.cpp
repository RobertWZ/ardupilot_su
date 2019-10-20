#include "AP_Logger2.h"
#include <AP_Logger/AP_Logger.h>

AP_Logger2 *AP_Logger2::_singleton;

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_Logger2::var_info[] = {
    // @Param: _BITMASK
    // @DisplayName: AP_Logger2 Bitmask
    // @Description: Bitmask for user defined log.
    // @Bitmask: 0:tilt,1:feed-forward,2:baro-lowpass,3:AltControl,4:AltLP,5:MavlinkTrace,6:RPYT
    // @User: Standard
    AP_GROUPINFO("_BITMASK",  0, AP_Logger2, params.bit_mask,       0),

    AP_GROUPEND
};

AP_Logger2::AP_Logger2()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Logger2 must be singleton");
    }

    _singleton = this;
}

void
AP_Logger2::Log_Write_Tilt(Vector3f error_vector, float error_angle)
{
	if ( (params.bit_mask & (0x1 << 0)) == 0 ) return;

    AP::logger().Write(LOG2_TILT_NAME,
    		           LOG2_TILT_LABELS,
					   LOG2_TILT_FMT,
                       AP_HAL::micros64(),
                       error_vector.x,
                       error_vector.y,
					   error_vector.z,
					   error_angle);
}

void
AP_Logger2::Log_Write_FeedForward(Vector3f target_angle_vel, float desired_angle_vel_x, float desired_angle_vel_y, float desired_angle_vel_z)
{
	if ( (params.bit_mask & (0x1 << 1)) == 0 ) return;

    AP::logger().Write(LOG2_ATT_FFWD_NAME,
    		           LOG2_ATT_FFWD_LABELS,
					   LOG2_ATT_FFWD_FMT,
                       AP_HAL::micros64(),
                       target_angle_vel.x,
					   target_angle_vel.y,
					   target_angle_vel.z,
					   desired_angle_vel_x,
					   desired_angle_vel_y,
					   desired_angle_vel_z);
}

void
AP_Logger2::Log_Write_BaroLowpass(float press, float press_raw)
{
	if ( (params.bit_mask & (0x1 << 2)) == 0 ) return;

    AP::logger().Write(LOG2_BARO_LOWPASS_NAME,
    		           LOG2_BARO_LOWPASS_LABELS,
					   LOG2_BARO_LOWPASS_FMT,
                       AP_HAL::micros64(),
                       press,
					   press_raw);
}

void
AP_Logger2::Log_Write_Alt(float pt,
                          float pr,
		                  float vt,
		                  float vd,
		                  float vr,
		                  float at,
		                  float ad,
		                  float ar)
{
	if ( (params.bit_mask & (0x1 << 3)) == 0 ) return;

    AP::logger().Write(LOG2_ALT_NAME,
    		           LOG2_ALT_LABELS,
					   LOG2_ALT_FMT,
                       AP_HAL::micros64(),
                       pt,
					   pr,
					   vt,
					   vd,
					   vr,
					   at,
					   ad,
					   ar);
}

void
AP_Logger2::Log_Write_LPVE(float ve, float lpve)
{
	if ( (params.bit_mask & (0x1 << 4)) == 0 ) return;

    AP::logger().Write(LOG2_LOWPASS_ALT_NAME,
    		           LOG2_LOWPASS_ALT_LABELS,
					   LOG2_LOWPASS_ALT_FMT,
                       AP_HAL::micros64(),
                       ve,
					   lpve);
}

void
AP_Logger2::Log_Write_MALK(uint8_t chan, uint32_t message_id)
{
	if ( (params.bit_mask & (0x1 << 5)) == 0 ) return;

    AP::logger().Write(LOG2_MAV_TRACE_NAME,
    		           LOG2_MAV_TRACE_LABELS,
					   LOG2_MAV_TRACE_FMT,
                       AP_HAL::micros64(),
                       chan,
					   message_id);
}

void
AP_Logger2::Log_Write_RPYT(float r,
                           float p,
                           float y,
                           float t)
{
	if ( (params.bit_mask & (0x1 << 6)) == 0 ) return;

    AP::logger().Write(LOG2_RPYT_THRUST_NAME,
    		           LOG2_RPYT_THRUST_LABELS,
					   LOG2_RPYT_THRUST_FMT,
                       AP_HAL::micros64(),
                       r,
					   p,
					   y,
					   t);
}

namespace AP {

AP_Logger2 &logger2()
{
    return *AP_Logger2::get_singleton();
}

};
