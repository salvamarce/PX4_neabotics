/**** CUSTOM ****/

#ifndef CONCRETE_TOOL_DATA_HPP
#define CONCRETE_TOOL_DATA_HPP

#include <uORB/topics/concrete_tool_data.h>

class MavlinkStreamConcreteToolData : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamConcreteToolData(mavlink); }

	static constexpr const char *get_name_static() { return "CONCRETE_TOOL_DATA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CONCRETE_TOOL_DATA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _tool_sub.advertised() ? MAVLINK_MSG_ID_CONCRETE_TOOL_DATA + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamConcreteToolData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _tool_sub{ORB_ID(concrete_tool_data)};

	bool send() override
	{
		concrete_tool_data_s tool;

		if (_tool_sub.update(&tool)) {
			mavlink_concrete_tool_data_t msg{};
			msg.timestamp = tool.timestamp;
            msg.distance = 0.25f * (tool.distance[0] + tool.distance[1] + tool.distance[2] + tool.distance[3]);
            msg.force = tool.force[0];

			mavlink_msg_concrete_tool_data_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // LAMA_STATE_HPP
