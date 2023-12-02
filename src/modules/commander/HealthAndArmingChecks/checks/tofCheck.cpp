/****************************************************************************
 *** CUSTOM ***
 ****************************************************************************/

#include "tofCheck.hpp"

using namespace time_literals;

void TofChecks::checkAndReport(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().tof_invalid = true;

	concrete_tool_data_s tool_data;

	if (_tool_data_sub.copy(&tool_data)) {

		bool data_is_recent = hrt_absolute_time() < tool_data.timestamp
				      + static_cast<hrt_abstime>(_param_com_lama_loss_t.get() * 1_s);

		bool lama_available = tool_data.distance[concrete_tool_data_s::TOP_LEFT] > 0 		&&
					tool_data.distance[concrete_tool_data_s::TOP_RIGHT] > 0 	&&
					tool_data.distance[concrete_tool_data_s::BOTTOM_LEFT] > 0 	&&
					tool_data.distance[concrete_tool_data_s::BOTTOM_RIGHT] > 0	&&
					data_is_recent;

		// This is a mode requirement, no need to report
		reporter.failsafeFlags().tof_invalid = !lama_available;
	}
}
