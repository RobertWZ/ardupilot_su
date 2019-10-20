/* ************************************************************ */
/* Test for AP_Logger2 Log library                               */
/* ************************************************************ */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <stdint.h>

#define LOG2_TILT_NAME   "TILT"
#define LOG2_TILT_LABELS "TimeUS,ErrX,ErrY,ErrZ,ErrAng"
#define LOG2_TILT_FMT    "Qffff"

#define LOG2_ATT_FFWD_NAME   "FFWD"
#define LOG2_ATT_FFWD_LABELS "TimeUS,TarX,TarY,TarZ,DesX,DesY,DesZ"
#define LOG2_ATT_FFWD_FMT    "Qffffff"

#define LOG2_BARO_LOWPASS_NAME   "BARX"
#define LOG2_BARO_LOWPASS_LABELS "TimeUS,P,PRaw"
#define LOG2_BARO_LOWPASS_FMT    "Qff"

#define LOG2_ALT_NAME   "ALT"
#define LOG2_ALT_LABELS "TimeUS,PT,PR,VT,VD,VR,AT,AD,AR"
#define LOG2_ALT_FMT    "Qffffffff"

#define LOG2_LOWPASS_ALT_NAME   "LPAT"
#define LOG2_LOWPASS_ALT_LABELS "TimeUS,VE,LPVE"
#define LOG2_LOWPASS_ALT_FMT    "Qff"

#define LOG2_MAV_TRACE_NAME   "MALK"
#define LOG2_MAV_TRACE_LABELS "TimeUS,chan,msgid"
#define LOG2_MAV_TRACE_FMT    "QBI"

#define LOG2_RPYT_THRUST_NAME   "RPYT"
#define LOG2_RPYT_THRUST_LABELS "TimeUS,r,p,y,t"
#define LOG2_RPYT_THRUST_FMT    "Qffff"

class AP_Logger2
{
public:

    AP_Logger2();

    /* Do not allow copies */
    AP_Logger2(const AP_Logger2 &other) = delete;
    AP_Logger2 &operator=(const AP_Logger2&) = delete;

    // get singleton instance
    static AP_Logger2 *get_singleton(void) {
        return _singleton;
    }

    // parameter support
    static const struct AP_Param::GroupInfo        var_info[];

    struct {
        AP_Int32 bit_mask;
    } params;

    void Log_Write_Tilt(Vector3f error_vector, float error_angle);
    void Log_Write_FeedForward(Vector3f target_angle_vel, float desired_angle_vel_x, float desired_angle_vel_y, float desired_angle_vel_z);
    void Log_Write_BaroLowpass(float press, float press_raw);
    void Log_Write_Alt(float pt,
    		           float pr,
					   float vt,
					   float vd,
					   float vr,
					   float at,
					   float ad,
					   float ar);
    void Log_Write_LPVE(float ve, float lpve);
    void Log_Write_MALK(uint8_t chan, uint32_t message_id);
    void Log_Write_RPYT(float r,
    		            float p,
						float y,
						float t);

private:
	static AP_Logger2 *_singleton;

};

namespace AP {
    AP_Logger2 &logger2();
};
