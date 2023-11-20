/**
 * @file reset_bias.cpp
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ft_reset_bias_request.h>

extern "C" __EXPORT int reset_bias_main(int argc, char *argv[]);

int reset_bias_main(int argc, char *argv[]){
	PX4_INFO("Reset bias test");
	struct ft_reset_bias_request_s resetBias;
	uORB::Publication<ft_reset_bias_request_s> reset_pub{ORB_ID(ft_reset_bias_request)};
	resetBias.timestamp = hrt_absolute_time();
	resetBias.reset_bias = true;
	reset_pub.publish(resetBias);
	usleep(1000*1000);
	PX4_INFO("Done!");
	return 0;
}
