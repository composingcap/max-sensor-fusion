/// @file
///	@ingroup 	minexamples
///	@copyright	Copyright 2018 The Min-DevKit Authors. All rights reserved.
///	@license	Use of this source code is governed by the MIT License found in the License.md file.

#include "c74_min.h"
#include "../../Fusion/Fusion/Fusion.h"
#include <chrono>

using namespace c74::min;

class sensor_fusion : public object<sensor_fusion>
{
private:
	FusionVector accel_data{0.0f, 0.0f, 0.0f};
	FusionVector gyro_data{0.0f, 0.0f, 0.0f};
	FusionAhrs ahrs{};
	std::chrono::time_point<std::chrono::steady_clock> last_time;
	;

public:
	MIN_DESCRIPTION{"Fuse IMU data with sensor fusion"};
	MIN_TAGS{"imu"};
	MIN_AUTHOR{"Christopher Poovey"};
	MIN_RELATED{""};

	inlet<> gyro{this, "gyroscope xyz"};
	inlet<> accel{this, "accelerometer xyz"};
	outlet<> euler{this, "euler rotation"};
	outlet<> quat{this, "quaternion rotation"};
	outlet<> time_step{this, "time step"};

	message<> m_setup{
		this, "setup", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			FusionAhrsInitialise(&ahrs);
			last_time = std::chrono::high_resolution_clock::now();
			return {};
		}
	};

	message<> m_reset{
		this, "reset", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			FusionAhrsInitialise(&ahrs);
			return {};
		}
	};

	message<> list{
		this, "list", "accelerometer data", [this](const c74::min::atoms& args, const int inlet) -> c74::min::atoms
		{
			if (args.size() < 3)
				return {};
			if (inlet == 1)
			{
				accel_data.array[0] = static_cast<float>(args[0]);
				accel_data.array[1] = static_cast<float>(args[1]);
				accel_data.array[2] = static_cast<float>(args[2]);
				return {};
			}
			if (inlet != 0)
				return {};

			gyro_data.array[0] = static_cast<float>(args[0]);
			gyro_data.array[1] = static_cast<float>(args[1]);
			gyro_data.array[2] = static_cast<float>(args[2]);

			auto now = std::chrono::high_resolution_clock::now();

			auto duration = std::chrono::duration<float, std::chrono::seconds::period>(now - last_time).count();
			last_time = now;
			if (duration > 2) return {};

			FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_data, accel_data, duration);
			const FusionEuler euler_data = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

			time_step.send(atoms(atom{duration}));

			quat.send(atoms(std::begin(ahrs.quaternion.array), std::end(ahrs.quaternion.array)));

			euler.send(atoms(std::begin(euler_data.array), std::end(euler_data.array)));


			return {};
		}
	};

#pragma region ahrssetting


#pragma endregion
};


MIN_EXTERNAL(sensor_fusion);
