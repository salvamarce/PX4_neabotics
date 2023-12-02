/****************************************************************************
 *** CUSTOM ***
 ****************************************************************************/

#pragma once

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/concrete_tool_data.h>

class TofChecks : public HealthAndArmingCheckBase
{
public:
	TofChecks() = default;
	~TofChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	uORB::Subscription _tool_data_sub{ORB_ID(concrete_tool_data)};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamFloat<px4::params::COM_LAMA_LOSS_T>) _param_com_lama_loss_t
				       );
};
