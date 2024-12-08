/// @file
///	@ingroup 	minexamples
///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include "c74_min.h"
#include "../../Fusion/Fusion/Fusion.h"
#include <chrono>
#include <mutex>

using namespace c74::min;

class sensor_fusion : public object<sensor_fusion>
{
private:
	FusionVector accel_data_{0.0f, 0.0f, 0.0f};
	const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
	FusionVector gyro_data{0.0f, 0.0f, 0.0f};
	const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
	const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};

	FusionVector mag_data{0.0f, 0.0f, 0.0f};
	const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
	const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

	FusionOffset offset{};
	FusionAhrs ahrs_{};
	FusionAhrsSettings settings_{
		.convention = FusionConventionNwu,
		.gain = 0.5f,
		.gyroscopeRange = 0.0f,
		.accelerationRejection = 90.0f,
		.magneticRejection = 90.0f,
		.recoveryTriggerPeriod = 0,
	};

	FusionAhrsSettings settings_internal_{
		.convention = FusionConventionNwu,
		.gain = 0.5f,
		.gyroscopeRange = 0.0f,
		.accelerationRejection = 90.0f,
		.magneticRejection = 90.0f,
		.recoveryTriggerPeriod = 0,
	};


	dict dump_dict_{symbol(true)};

	mutex guard_;
	std::chrono::time_point<std::chrono::steady_clock> last_time;

	void apply_settings()
	{
		settings_internal_.gyroscopeRange = settings_.gyroscopeRange;
		settings_internal_.accelerationRejection = settings_.accelerationRejection;
		settings_internal_.gain = settings_.gain;
		settings_internal_.convention = settings_.convention;
		settings_internal_.magneticRejection = settings_.magneticRejection;
		settings_internal_.recoveryTriggerPeriod = settings_.recoveryTriggerPeriod;
		FusionAhrsSetSettings(&ahrs_, &settings_internal_);
		FusionAhrsReset(&ahrs_);
	}

public:
	MIN_DESCRIPTION{"Fuse IMU data with sensor fusion"};
	MIN_TAGS{"imu"};
	MIN_AUTHOR{"Christopher Poovey"};
	MIN_RELATED{""};

	enum class imuconvention : int
	{
		East_North_Up,
		North_East_Down,
		North_West_Up,
		enum_count
	};

	~sensor_fusion()
	{
		std::lock_guard<mutex> lock(guard_);
	}

	enum_map imuconvention_range = {"East_North_Up", "North_East_Down", "North_West_Up"};

	inlet<> gyro{this, "(list) gyroscope xyz", "list"};
	inlet<> accel{this, "(list) accelerometer xyz", "list"};
	inlet<> mag{this, "(list)magnetometer xyz", "list"};

	outlet<> euler{this, "(list) euler rotation", "list"};
	outlet<> quat{this, "(list) quaternion rotation", "list"};
	outlet<> earth{this, "(list) earth acceleration", "list"};
	outlet<> dump{this, "(dictionary) dump outlet", "dictionary"};
	outlet<> time_step{this, "(float) time step", "float"};


	timer<timer_options::defer_delivery> internal_update{
		this, [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			auto flags = FusionAhrsGetFlags(&ahrs_);
			dump_dict_["initialisingFlag"] = flags.initialising;
			dump_dict_["angularRateRecoveryFlag"] = flags.angularRateRecovery;
			dump_dict_["accelerationRecoveryFlag"] = flags.accelerationRecovery;
			dump_dict_["magneticRecoveryFlag"] = flags.magneticRecovery;

			auto states = FusionAhrsGetInternalStates(&ahrs_);
			dump_dict_["accelerationErrorState"] = states.accelerationError;
			dump_dict_["accelerometerIgnoredState"] = states.accelerometerIgnored;
			dump_dict_["accelerationRecoveryTriggerState"] = states.accelerationRecoveryTrigger;
			dump_dict_["magneticErrorState"] = states.magneticError;
			dump_dict_["magnetometerIgnoredState"] = states.magnetometerIgnored;
			dump_dict_["magneticRecoveryTriggerState"] = states.magneticRecoveryTrigger;

			dump.send(atoms{"dictionary", dump_dict_.name()});
			internal_update.delay(33);
			return {};
		}
	};
	message<> m_setup{
		this, "setup", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			FusionAhrsSetSettings(&ahrs_, &settings_internal_);
			FusionOffsetInitialise(&offset, 100);

			last_time = std::chrono::high_resolution_clock::now();
			apply_settings();
			internal_update.delay(33);
			return {};
		}
	};

	message<> m_reset{
		this, "reset", "resets the fusion", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			FusionAhrsReset(&ahrs_);
			return {};
		}
	};

	attribute<number> a_gyroscale{
		this, "gyroscopeScale", 1, description{"a scalar applied to the gyroscope data before processing"}
	};

	attribute<bool> a_use_mag{this, "useMagnetometer", false, description{"enable if your device has a magnetometer"}};

	message<> list{
		this, "list", "accelerometer data", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			//std::lock_guard<mutex> lock(guard_);
			if (args.size() < 3)
				return {};

			if (inlet == 2)
			{
				mag_data.array[0] = static_cast<float>(args[0]);
				mag_data.array[1] = static_cast<float>(args[1]);
				mag_data.array[2] = static_cast<float>(args[2]);
				return {};
			}
			if (inlet == 1)
			{
				accel_data_.array[0] = static_cast<float>(args[0]);
				accel_data_.array[1] = static_cast<float>(args[1]);
				accel_data_.array[2] = static_cast<float>(args[2]);
				return {};
			}
			if (inlet != 0)
				return {};

			gyro_data.array[0] = static_cast<float>(args[0]) * a_gyroscale;
			gyro_data.array[1] = static_cast<float>(args[1]) * a_gyroscale;
			gyro_data.array[2] = static_cast<float>(args[2]) * a_gyroscale;

			auto now = std::chrono::high_resolution_clock::now();

			auto duration = std::chrono::duration<float, std::chrono::seconds::period>(now - last_time).count();
			last_time = now;
			if (duration > 2) return {};
			gyro_data = FusionCalibrationInertial(gyro_data, gyroscopeMisalignment, gyroscopeSensitivity,
			                                      gyroscopeOffset);
			gyro_data = FusionOffsetUpdate(&offset, gyro_data);

			accel_data_ = FusionCalibrationInertial(accel_data_, accelerometerMisalignment, accelerometerSensitivity,
			                                        accelerometerOffset);

			if (a_use_mag)
			{
				mag_data = FusionCalibrationMagnetic(mag_data, softIronMatrix, hardIronOffset);
				FusionAhrsUpdate(&ahrs_, gyro_data, accel_data_, mag_data, duration);
			}
			else FusionAhrsUpdateNoMagnetometer(&ahrs_, gyro_data, accel_data_, duration);

			const auto quat_data = FusionAhrsGetQuaternion(&ahrs_);
			const FusionEuler euler_data = FusionQuaternionToEuler(quat_data);
			const FusionVector earth_data = FusionAhrsGetEarthAcceleration(&ahrs_);


			time_step.send(duration * 1000.0f);
			earth.send(atoms(std::begin(earth_data.array), std::end(earth_data.array)));
			quat.send(atoms(std::begin(quat_data.array), std::end(quat_data.array)));
			euler.send(atoms(std::begin(euler_data.array), std::end(euler_data.array)));


			return {};
		}
	};


#pragma region ahrssetting
	attribute<number> a_gain{
		this, "gain", 0.5, setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				settings_.gain = static_cast<float>(args[0]);
				apply_settings();
				return args;
			}
		},
		description{
			"Determines the influence of the gyroscope relative to other sensors. A value of zero will disable initialisation and the acceleration and magnetic rejection features. A value of 0.5 is appropriate for most applications."
		}
	};


	attribute<imuconvention> a_convention{
		this,
		"convention",
		imuconvention::North_West_Up,
		imuconvention_range,
		setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				switch (static_cast<imuconvention>(args[0]))
				{
				case imuconvention::East_North_Up:
					settings_.convention = FusionConventionEnu;
					break;
				case imuconvention::North_West_Up:
					settings_.convention = FusionConventionNwu;
					break;

				case imuconvention::North_East_Down:
					settings_.convention = FusionConventionNed;
					break;
				}
				apply_settings();
				return args;
			}
		},
		description{"The orientation of the sensor in the device"}
	};

	attribute<number> a_accelerometer_rejection{
		this,
		"accelRejection",
		90.0,
		setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				settings_.accelerationRejection = static_cast<float>(args[0]);
				apply_settings();
				return args;
			}
		},
		description{
			"Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this feature. A value of 10 degrees is appropriate for most applications."
		}

	};

	attribute<number> a_gyroscope_range{
		this, "gyroRange", 0,
		setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				settings_.gyroscopeRange = static_cast<float>(args[0]);
				apply_settings();
				return args;
			}
		},
		description{
			"Gyroscope range (in degrees per second). Angular rate recovery will activate if the gyroscope measurement exceeds 98% of this value. A value of zero will disable this feature. The value should be set to the range specified in the gyroscope datasheet."
		}
	};


	attribute<number> a_magnetic_rejection{
		this, "magRejection", 90.0,
		setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				settings_.magneticRejection = static_cast<float>(args[0]);
				apply_settings();
				return args;
			}
		},
		description{
			"Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature. A value of 10 degrees is appropriate for most applications."
		}
	};

	attribute<number> a_recovery_period{
		this, "recoveryTriggerPeriod", 0,
		setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				settings_.recoveryTriggerPeriod = static_cast<float>(args[0]);
				apply_settings();
				return args;
			}
		},
		description{
			"Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications."
		}
	};


#pragma endregion
};


MIN_EXTERNAL(sensor_fusion);
