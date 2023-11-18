/**
 * @file mbedi2c.cpp
 *
 * @author Michele Marolla <mic.marolla@gmail.com>
 *
 * Driver for I2C communication with mbed boards.
 * Default I2C address 0x31 is used.
 *
 * Structure taken from: distance_sensor/lightware_laser_i2c
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ft_reset_bias_request.h>
#include <uORB/topics/concrete_tool_data.h>


#define MBED_BASEADDR	0x25

#define CELL_V_DIST		1.0		// [m]
#define CELL_H_DIST		0.5		// [m]


using namespace time_literals;

typedef struct{
    uint16_t distances[4];
    uint16_t fx;
    int16_t ty;
    int16_t tz;
} SensorData;




class MbedI2C : public device::I2C, public I2CSPIDriver<MbedI2C>
{
public:
	MbedI2C(const I2CSPIDriverConfig &config);
	~MbedI2C() override;

	static void print_usage();
	int init() override;
	void print_status() override;

	void RunImpl();

private:

	enum class Register : uint8_t {
		ProductName = 2,
		ToolPosition = 3,
		ReadSensors = 4,
		ResetBias = 5
	};

	enum class State {
		Configuring,
		Running
	};

	int probe() override;
	void start();

	int readRegister(Register reg, uint8_t *data, int len);
	int configure();
	int collect();



private:
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};

	State _state{State::Configuring};
	int _consecutive_errors{0};

    uORB::Publication<concrete_tool_data_s> tooldata_pub{ORB_ID(concrete_tool_data)};
    concrete_tool_data_s tooldata_msg;

	uORB::Subscription _resetBias_sub {ORB_ID(ft_reset_bias_request)};
	struct ft_reset_bias_request_s resetBias_request;

	int32_t _param_consecutive_errors;

	int _conversion_interval{-1};	// us		200hz -> 5ms -> 5000us .. ??

	bool _first = true;
};
   

MbedI2C::MbedI2C(const I2CSPIDriverConfig &config) :
	I2C(config), I2CSPIDriver(config){
        tooldata_msg.tool_pos = TOOL_FRONT;
        for(int i=0; i<3; ++i){
            tooldata_msg.force[i] = 0;
            tooldata_msg.torque[i] = 0;
        }
        for(int i=0; i<4; ++i)
            tooldata_msg.distance[i] = 0;
}

MbedI2C::~MbedI2C(){
	perf_free(_comms_errors);
}

int MbedI2C::init(){
	int ret = PX4_ERROR;

	_param_consecutive_errors = 3;
	_conversion_interval = 5 * 1000;

	ret = I2C::init();

	if (ret == PX4_OK)
		start();
	
	return ret;
}

int MbedI2C::readRegister(Register reg, uint8_t *data, int len){
	const uint8_t cmd = (uint8_t)reg;
	return transfer(&cmd, 1, data, len);
}


int MbedI2C::probe(){
	PX4_INFO("Looking for mbed nucleo board...");

    // Read the product name (expected 'mbed')
    char product_name[5];
    readRegister(Register::ProductName, (uint8_t*)product_name, 4);
    product_name[sizeof(product_name) - 1] = '\0';
	int nameCheck = strncmp((const char *)product_name, "mbed", sizeof(product_name)-1);

    if (nameCheck == 0){
		PX4_INFO("connected nucleo with name: %s", product_name);
        return 0;
	}
    return -1;
}


int MbedI2C::configure(){	
	// Get tool position
    uint8_t toolPos = 0;
	int ret = readRegister(Register::ToolPosition, &toolPos, 1);
	if(PX4_OK != ret){
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}
	PX4_INFO("Tool position: %d", toolPos);

	tooldata_msg.tool_pos = toolPos;
	tooldata_pub.publish(tooldata_msg);

	// Start measurements
	const uint8_t cmd = (uint8_t)Register::ReadSensors;
	ret = transfer(&cmd, 1, nullptr, 0);
    return ret;
	//return 0;
}

int MbedI2C::collect(){
    /* read from the sensor */
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    
 	// retrieve data
    SensorData sensorData;
	int res = transfer(nullptr, 0, (uint8_t*)&sensorData, sizeof(sensorData));
    if (res < 0) {
        perf_count(_comms_errors);
        return PX4_ERROR;
    }

    // publish data
    tooldata_msg.timestamp = timestamp_sample;
	tooldata_msg.force[0] = sensorData.fx * 0.001 * 9.81;		// from [g] to [N]
	tooldata_msg.force[1] = ftdata.force[2] = 0;
	tooldata_msg.torque[0] = 0;
	tooldata_msg.torque[1] = sensorData.ty * 0.001 * 9.81 * 0.5 * CELL_V_DIST;		// from [g] to [Nm]
	tooldata_msg.torque[2] = sensorData.tz * 0.001 * 9.81 * 0.5 * CELL_H_DIST;	// from [g] to [Nm]
    
	for(int i=0; i<4; ++i)
		tooldata_msg.distance[i] = 0.001f * sensorData.distances[i];		// from [m] to [mm]

	tooldata_pub.publish(tooldata_msg);

	return PX4_OK;
}

void MbedI2C::start(){
	ScheduleDelayed(_conversion_interval);
}

void MbedI2C::RunImpl(){
	switch (_state) {
	case State::Configuring: {
			if (configure() == 0) {
				_state = State::Running;
				ScheduleDelayed(_conversion_interval);
			} else {
				// retry after a while
				PX4_DEBUG("Retrying...");
				ScheduleDelayed(300_ms);
			}

			break;
		}

	case State::Running:
		if(_resetBias_sub.updated()){
			_resetBias_sub.copy(&resetBias_request);
			if(resetBias_request.reset_bias){
				Register cmd = Register::ResetBias;
				transfer((uint8_t*)&cmd, 1, nullptr, 0);
			}
		}

		if (PX4_OK != collect()) {
			PX4_DEBUG("collection error");

			if (++_consecutive_errors > _param_consecutive_errors) {
				_state = State::Configuring;
				_consecutive_errors = 0;
			}
		}

		ScheduleDelayed(_conversion_interval);
		break;
	}
}


void MbedI2C::print_status(){
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
}

void MbedI2C::print_usage(){
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for communication with mbed boards.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mbedi2c", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(MBED_BASEADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int mbedi2c_main(int argc, char *argv[])
{
	using ThisDriver = MbedI2C;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = MBED_BASEADDR;

	int ch;
	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}


	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_LIGHTWARE_LASER); // ?

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
