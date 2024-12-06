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
	FusionOffset offset{};
	FusionAhrs ahrs_{};
	mutex guard_;
	std::chrono::time_point<std::chrono::steady_clock> last_time;

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

	enum_map imuconvention_range = {"East_North_Up", "North_East_Down", "North_West_Up"};

	inlet<> gyro{this, "gyroscope xyz"};
	inlet<> accel{this, "accelerometer xyz"};
	outlet<> euler{this, "euler rotation"};
	outlet<> quat{this, "quaternion rotation"};
	outlet<> time_step{this, "time step"};

	message<> m_setup{
		this, "setup", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			FusionAhrsInitialise(&ahrs_);
			FusionOffsetInitialise(&offset, 100);

			last_time = std::chrono::high_resolution_clock::now();
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

	attribute<number> a_gyroscale{this, "gyroscopeScale", 50};

	message<> list{
		this, "list", "accelerometer data", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			//std::lock_guard<mutex> lock(guard_);
			if (args.size() < 3)
				return {};
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


			FusionAhrsUpdateNoMagnetometer(&ahrs_, gyro_data, accel_data_, duration);
			const FusionEuler euler_data = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs_));

			time_step.send(atoms(atom{duration}));

			quat.send(atoms(std::begin(ahrs_.quaternion.array), std::end(ahrs_.quaternion.array)));

			euler.send(atoms(std::begin(euler_data.array), std::end(euler_data.array)));


			return {};
		}
	};

#pragma region ahrssetting
	attribute<number, threadsafe::no> a_gain{
		this, "gain", 0.5, setter{
			[this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
			{
				ahrs_.settings.gain = static_cast<float>(args[0]);
				FusionAhrsReset(&ahrs_);
				return args;
			}
		},
		getter{
			[this] { return atoms{ahrs_.settings.gain}; },
		}
	};


	attribute<imuconvention, threadsafe::no> a_convention{
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
					ahrs_.settings.convention = FusionConventionEnu;
					break;
				case imuconvention::North_West_Up:
					ahrs_.settings.convention = FusionConventionNwu;
					break;

				case imuconvention::North_East_Down:
					ahrs_.settings.convention = FusionConventionNed;
					break;
				}
				FusionAhrsReset(&ahrs_);
				return args;
			}
		},
	};


#pragma endregion
};


MIN_EXTERNAL(sensor_fusion);
