/**** CUSTOM ****/

#ifndef LAMA_STATE_HPP
#define LAMA_STATE_HPP

#include <uORB/topics/lama_state.h>

class MavlinkStreamLamaState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLamaState(mavlink); }

	static constexpr const char *get_name_static() { return "LAMA_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_LAMA_STATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _lama_sub.advertised() ? MAVLINK_MSG_ID_LAMA_STATE + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamLamaState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _lama_sub{ORB_ID(lama_state)};

	bool send() override
	{
		lama_state_s lama;

		if (_lama_sub.update(&lama)) {
			mavlink_lama_state_t msg{};
			msg.timestamp = lama.timestamp;
			msg.state = lama.state;
			msg.engage_approach = lama.engage_approach;
			msg.engage_interaction = lama.engage_interaction;

			mavlink_msg_lama_state_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // LAMA_STATE_HPP
