#include <iomanip>
#include <iostream>
#if 0

#include "ctre/phoenix/platform/Platform.h"

int main(void)
{
	ctre::phoenix::platform::can::canframe_t canFrame[1000];

	if (const auto rc = ctre::phoenix::platform::can::SetCANInterface("can0"))
	{
		std::cout << "ctre::phoenix::platform::can::SetCANInterface returned " << rc << std::endl;
		return -1;
	}
	if (const auto rc = ctre::phoenix::platform::can::SetCANInterface("can0"))
	{
		std::cout << "ctre::phoenix::platform::can::SetCANInterface call 2 returned " << rc << std::endl;
		return -1;
	}

	while (true)
	{
		uint32_t numFilled = 0;
		const auto rc = ctre::phoenix::platform::can::CANbus_ReceiveFrame(canFrame, sizeof(canFrame) / sizeof(canFrame[0]), &numFilled);

		std::cout << rc << " " << numFilled << std::endl;
		for (uint32_t i = 0; i < numFilled; i++)
		{
			std::cout << "\t";
			const auto &c = canFrame[i];
			std::cout << std::dec << c.timeStampUs << " ";
			std::cout << std::hex << c.arbID << " ";
			std::cout << std::hex << c.flags << " ";
			std::cout << std::hex << static_cast<uint32_t>(c.dlc) << " ";
			for (size_t j = 0; j < c.dlc; j++)
				std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<uint32_t>(c.data[j]) << " ";
			std::cout << std::endl;
		}
	}

	return 0;
}
#else

#include <ctre/phoenix/CANifier.h>
//#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include <chrono>
#include <thread>

int main(void)
{
	if (const auto rc = ctre::phoenix::platform::can::SetCANInterface("can0"))
	{
		std::cout << "ctre::phoenix::platform::can::SetCANInterface returned " << rc << std::endl;
		return -1;
	}
	c_Phoenix_Diagnostics_Create();
	//ctre::phoenix::motorcontrol::can::TalonSRX t(0);
	//ctre::phoenix::CANifier c(0);
	ctre::phoenix::platform::can::canframe_t canFrame[1000];
	while (true)
	{
		uint32_t numFilled = 0;
		const auto rc = ctre::phoenix::platform::can::CANbus_ReceiveFrame(canFrame, sizeof(canFrame) / sizeof(canFrame[0]), &numFilled);

		std::cout << rc << " " << numFilled << std::endl;
		for (uint32_t i = 0; i < numFilled; i++)
		{
			std::cout << "\t";
			const auto &c = canFrame[i];
			std::cout << std::dec << c.timeStampUs << " ";
			std::cout << std::hex << c.arbID << " ";
			std::cout << std::hex << c.flags << " ";
			std::cout << std::hex << static_cast<uint32_t>(c.dlc) << " ";
			for (size_t j = 0; j < c.dlc; j++)
				std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<uint32_t>(c.data[j]) << " ";
			std::cout << std::endl;
		}
	}
	//while (1)
		//std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	return 0;
}

#endif
