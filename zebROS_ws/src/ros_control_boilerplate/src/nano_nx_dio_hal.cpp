// Code to handle IO with Nano NX carrier board general-purpose IO
// This includes a class to map from board pin number to chip GPIO number
// Also includes code which uses libgpiod to access those pins

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "hal/DIO.h"
#include "hal/Types.h"
#include "DigitalInternal.h"

#include <gpiod.h>

// These arrays contain tuples of all the relevant GPIO data for each Jetson
// Platform. The fields are:
// - Linux GPIO pin number,
// - GPIO chip sysfs directory
// - Pin number (BOARD mode)
// - Pin number (BCM mode)
// - Pin name (CVM mode)
// - Pin name (TEGRA_SOC mode)
// - PWM chip sysfs directory
// - PWM ID within PWM chip
// The values are use to generate dictionaries that map the corresponding pin
// mode numbers to the Linux GPIO pin number and GPIO chip directory

class NxGPIOPin
{
	public:
		NxGPIOPin(
				uint8_t     linuxPin,
				std::string sysfsDir,
				uint8_t     boardPin,
				uint8_t     bcmPin,
				std::string pinNameCVM,
				std::string pinNameTegraSOC,
				std::string pwmSysfsDir,
				uint8_t     pwmID)
	: linuxPin_(linuxPin)
	, sysfsDir_(sysfsDir)
	, boardPin_(boardPin)
	, bcmPin_(bcmPin)
	, pinNameCVM_(pinNameCVM)
	, pinNameTegraSOC_(pinNameTegraSOC)
	, pwmSysfsDir_(pwmSysfsDir)
	, pwmID_(pwmID)
	, inUse_(0)
    , isInput_(false)
	{
	}

	uint8_t     linuxPin_;
	std::string sysfsDir_;
	uint8_t     boardPin_;
	uint8_t     bcmPin_;
	std::string pinNameCVM_;
	std::string pinNameTegraSOC_;
	std::string pwmSysfsDir_;
	uint8_t     pwmID_;
	size_t      inUse_;
	bool        isInput_;
	bool        isPWM_;
};

static const std::vector<NxGPIOPin> JETSON_NX_PIN_DEFS = {
    {148, "/sys/devices/2200000.gpio", 7, 4, "GPIO09", "AUD_MCLK", "", 0},
    {140, "/sys/devices/2200000.gpio", 11, 17, "UART1_RTS", "UART1_RTS", "", 0},
    {157, "/sys/devices/2200000.gpio", 12, 18, "I2S0_SCLK", "DAP5_SCLK", "", 0},
    {192, "/sys/devices/2200000.gpio", 13, 27, "SPI1_SCK", "SPI3_SCK", "", 0},
    {20,  "/sys/devices/c2f0000.gpio", 15, 22, "GPIO12", "TOUCH_CLK", "", 0},
    {196, "/sys/devices/2200000.gpio", 16, 23, "SPI1_CS1", "SPI3_CS1_N", "", 0},
    {195, "/sys/devices/2200000.gpio", 18, 24, "SPI1_CS0", "SPI3_CS0_N", "", 0},
    {205, "/sys/devices/2200000.gpio", 19, 10, "SPI0_MOSI", "SPI1_MOSI", "", 0},
    {204, "/sys/devices/2200000.gpio", 21, 9, "SPI0_MISO", "SPI1_MISO", "", 0},
    {193, "/sys/devices/2200000.gpio", 22, 25, "SPI1_MISO", "SPI3_MISO", "", 0},
    {203, "/sys/devices/2200000.gpio", 23, 11, "SPI0_SCK", "SPI1_SCK", "", 0},
    {206, "/sys/devices/2200000.gpio", 24, 8, "SPI0_CS0", "SPI1_CS0_N", "", 0},
    {207, "/sys/devices/2200000.gpio", 26, 7, "SPI0_CS1", "SPI1_CS1_N", "", 0},
    {133, "/sys/devices/2200000.gpio", 29, 5, "GPIO01", "SOC_GPIO41", "", 0},
    {134, "/sys/devices/2200000.gpio", 31, 6, "GPIO11", "SOC_GPIO42", "", 0},
    {136, "/sys/devices/2200000.gpio", 32, 12, "GPIO07", "SOC_GPIO44", "/sys/devices/32f0000.pwm", 0},
    {105, "/sys/devices/2200000.gpio", 33, 13, "GPIO13", "SOC_GPIO54", "/sys/devices/3280000.pwm", 0},
    {160, "/sys/devices/2200000.gpio", 35, 19, "I2S0_FS", "DAP5_FS", "", 0},
    {141, "/sys/devices/2200000.gpio", 36, 16, "UART1_CTS", "UART1_CTS", "", 0},
    {194, "/sys/devices/2200000.gpio", 37, 26, "SPI1_MOSI", "SPI3_MOSI", "", 0},
    {159, "/sys/devices/2200000.gpio", 38, 20, "I2S0_DIN", "DAP5_DIN", "", 0},
    {158, "/sys/devices/2200000.gpio", 40, 21, "I2S0_DOUT", "DAP5_DOUT", "", 0}
};

class NxGPIOPins
{
	using PinHandle = std::map<uint8_t, NxGPIOPin>::iterator;
	public :
		NxGPIOPins(void)
		{
			for (const auto &p: JETSON_NX_PIN_DEFS)
			{
				nxGPIOPinMap_[p.boardPin_] = p;
			}
		}
		bool allocateDigitalIO(uint8_t pinNum, bool input)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return false;
			}
			// Allow duplicate references to the same pin
			// if the input/output direction is the same for all of them
			if (ph->second.inUse_)
			{
				if ((ph->second.isInput_ != input) || ph->second.isPWM_)
				return false;
			}
			// Set the pin in use, set the correct io direction and mark it as non-PWM (PWM is a TODO)
			ph->second.inUse_   += 1;
			ph->second.isInput_  = input;
			ph->second.isPWM_    = false;
			return true;
		}
		// Remove a reference count to this device
		// Return count of remaining references
		size_t freeDigitalIO(uint8_t pinNum)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return std::numeric_limits<size_t>::max();
			}
			if (!ph->second.inUse_)
			{
				return std::numeric_limits<size_t>::max();
			}
			ph->second.inUse_ -= 1;
			return ph->second.inUse_;
		}
		bool getLinuxPin(uint8_t pinNum, uint8_t &linuxPin)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return false;
			}
			linuxPin = ph->second.linuxPin_;
			return true;
		}
		bool getPinOutputEnable(uint8_t pinNum, bool &outputEnable)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return false;
			}
			outputEnable = !ph->second.isInput_;
			return true;
		}
		bool isValidChannel(uint8_t pinNum)
		{
			return nxGPIOPinMap_.find(pinNum) != nxGPIOPinMap_.end();
		}
	private:
		bool getByPinNum(uint8_t pinNum, PinHandle &pin)
		{
			pin = nxGPIOPinMap_.find(pinNum);
			if (pin == nxGPIOPinMap_.end())
				return false;
			return true;
		}
		// Map from pin number to GPIO data
		std::map<uint8_t, NxGPIOPin> nxGPIOPinMap_;
} nxGPIOPins;


#ifdef __cplusplus
extern "C" {
#endif

static std::map<uint8_t, struct gpiod_line *> gpiodLineMap;
static struct gpiod_chip *gpiodChip = nullptr;
HAL_DigitalHandle HAL_InitializeDIOPort(HAL_PortHandle portHandle,
                                        HAL_Bool input, int32_t* status)
{
	if (*status)
		return HAL_kInvalidHandle;

	// portHandle will be a NX pin number
	// check that pin is usable as DIn/Out
	//   - all should be OK, assuming they're not already allocated to
	//     a different direction (i.e. can't reuse an out and an in

	if (!nxGPIOPins.allocateDigitalIO(portHandle, input))
	{
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	// Create gpiod struct needed to access GPIO hardware
	uint8_t offset;
	if (!nxGPIOPins.getLinuxPin(portHandle, offset))
	{
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	if (!gpiodChip)
		gpiodChip = gpiod_chip_open_lookup("tegra-gpio");
	if (!gpiodChip)
	{
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	const auto line = gpiod_chip_get_line(gpiodChip, offset);
	if (!line)
	{
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	gpiodLineMap[portHandle] = line;

	return portHandle;
}

HAL_Bool HAL_CheckDIOChannel(int32_t channel)
{
	return nxGPIOPins.isValidChannel(channel);
}


void HAL_FreeDIOPort(HAL_DigitalHandle dioPortHandle)
{
	if (nxGPIOPins.freeDigitalIO(dioPortHandle) == 0)
	{
		auto it = gpiodLineMap.find(dioPortHandle);
		if (it != gpiodLineMap.end())
		{
			gpiodLineMap.erase(it);
		}
	}
}


void HAL_SetDIOSimDevice(HAL_DigitalHandle handle, HAL_SimDeviceHandle device)
{
}


HAL_DigitalPWMHandle HAL_AllocateDigitalPWM(int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
}


void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator, int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
}


void HAL_SetDigitalPWMRate(double rate, int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
}


void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator,
                                double dutyCycle, int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
}


void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator,
                                    int32_t channel, int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
}


void HAL_SetDIO(HAL_DigitalHandle dioPortHandle, HAL_Bool value,
                int32_t* status)
{
	if (*status)
	{
		return;
	}

	// Check pin is valid output
	bool outputEnable;
	if (!nxGPIOPins.getPinOutputEnable(dioPortHandle, outputEnable) || !outputEnable)
	{
		*status = HAL_HANDLE_ERROR;
		return;
	}
	// Get value from gpiodLineMap
	const auto gpiodLine = gpiodLineMap.find(dioPortHandle);
	if (gpiodLine == gpiodLineMap.end())
	{
		*status = HAL_HANDLE_ERROR;
		return;
	}

	if (gpiod_line_request_output(gpiodLine->second, "", value) < 0)
	{
		*status = HAL_HANDLE_ERROR;
	}
}


void HAL_SetDIODirection(HAL_DigitalHandle dioPortHandle, HAL_Bool input,
                         int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
		return;
}


HAL_Bool HAL_GetDIO(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	if (*status)
	{
		return false;
	}

	// Check pin is valid output
	bool outputEnable;
	if (!nxGPIOPins.getPinOutputEnable(dioPortHandle, outputEnable) || outputEnable)
	{
		*status = HAL_HANDLE_ERROR;
		return false;
	}
	// Get value from gpiodLineMap
	const auto gpiodLine = gpiodLineMap.find(dioPortHandle);
	if (gpiodLine == gpiodLineMap.end())
	{
		*status = HAL_HANDLE_ERROR;
		return false;
	}

	return gpiod_line_request_input(gpiodLine->second, "");
}


HAL_Bool HAL_GetDIODirection(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	// Check pin is valid output
	bool outputEnable;
	if (!nxGPIOPins.getPinOutputEnable(dioPortHandle, outputEnable))
	{
		*status = HAL_HANDLE_ERROR;
		return false;
	}
	return outputEnable;
}


void HAL_Pulse(HAL_DigitalHandle dioPortHandle, double pulseLength,
               int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return;
}


HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return false;
}


HAL_Bool HAL_IsAnyPulsing(int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return false;
}


void HAL_SetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t filterIndex,
                         int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return;
}


int32_t HAL_GetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return 0;
}


void HAL_SetFilterPeriod(int32_t filterIndex, int64_t value, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return;
}


int64_t HAL_GetFilterPeriod(int32_t filterIndex, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return 0;
}

#ifdef __cplusplus
}  // extern "C"
#endif
