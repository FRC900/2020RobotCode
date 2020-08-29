#include <ros/ros.h>

#include <WPILibVersion.h>
const char* GetWPILibVersion(void)
{
	ROS_ERROR("Called GetWPILibVersion on unsupported platform");
	return "900.2020";
}


#include <AHRS.h>
AHRS::AHRS(SPI::Port)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id) on unsupported platform");
}
AHRS::AHRS(I2C::Port)
{
	ROS_ERROR("Called AHRS(I2C::Port i2c_port_id) on unsupported platform");
}
AHRS::AHRS(SerialPort::Port)
{
	ROS_ERROR("Called AHRS(SerialPort::Port serial_port_id) on unsupported platform");
}

AHRS::AHRS(SPI::Port, uint8_t)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id, uint8_t update_rate_hz) on unsupported platform");
}
AHRS::AHRS(SPI::Port, uint32_t, uint8_t)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id, uint32_t spi_bitrate, uint8_t update_rate_hz) on unsupported platform");
}

AHRS::AHRS(I2C::Port, uint8_t)
{
	ROS_ERROR("Called AHRS(I2C::Port i2c_port_id, uint8_t update_rate_hz) on unsupported platform");
}

AHRS::AHRS(SerialPort::Port, AHRS::SerialDataType , uint8_t)
{
	ROS_ERROR("Called AHRS(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) on unsupported platform");
}

float  AHRS::AHRS::GetPitch()
{
	ROS_ERROR("Called GetPitch() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::AHRS::GetRoll()
{
	ROS_ERROR("Called GetRoll() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::AHRS::GetYaw()
{
	ROS_ERROR("Called GetYaw() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetCompassHeading()
{
	ROS_ERROR("Called GetCompassHeading() on unsupported platform");
	return std::numeric_limits<float>::max();
}
void   AHRS::ZeroYaw()
{
	ROS_ERROR("Called ZeroYaw() on unsupported platform");
}
bool   AHRS::IsCalibrating()
{
	ROS_ERROR("Called IsCalibrating() on unsupported platform");
	return false;
}
bool   AHRS::IsConnected()
{
	ROS_ERROR("Called IsConnected() on unsupported platform");
	return false;
}
double AHRS::GetByteCount()
{
	ROS_ERROR("Called GetByteCount() on unsupported platform");
	return std::numeric_limits<double>::max();
}
double AHRS::GetUpdateCount()
{
	ROS_ERROR("Called GetUpdateCount() on unsupported platform");
	return std::numeric_limits<double>::max();
}
long   AHRS::GetLastSensorTimestamp()
{
	ROS_ERROR("Called GetLastSensorTimestamp() on unsupported platform");
	return std::numeric_limits<long>::max();
}
float  AHRS::GetWorldLinearAccelX()
{
	ROS_ERROR("Called GetWorldLinearAccelX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetWorldLinearAccelY()
{
	ROS_ERROR("Called GetWorldLinearAccelY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetWorldLinearAccelZ()
{
	ROS_ERROR("Called GetWorldLinearAccelZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
bool   AHRS::IsMoving()
{
	ROS_ERROR("Called IsMoving() on unsupported platform");
	return false;
}
bool   AHRS::IsRotating()
{
	ROS_ERROR("Called IsRotating() on unsupported platform");
	return false;
}
float  AHRS::GetBarometricPressure()
{
	ROS_ERROR("Called GetBarometricPressure() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetAltitude()
{
	ROS_ERROR("Called GetAltitude() on unsupported platform");
	return std::numeric_limits<float>::max();
}
bool   AHRS::IsAltitudeValid()
{
	ROS_ERROR("Called IsAltitudeValid() on unsupported platform");
	return false;
}
float  AHRS::GetFusedHeading()
{
	ROS_ERROR("Called GetFusedHeading() on unsupported platform");
	return std::numeric_limits<float>::max();
}
bool   AHRS::IsMagneticDisturbance()
{
	ROS_ERROR("Called IsMagneticDisturbance() on unsupported platform");
	return false;
}
bool   AHRS::IsMagnetometerCalibrated()
{
	ROS_ERROR("Called IsMagnetometerCalibrated() on unsupported platform");
	return false;
}
float  AHRS::GetQuaternionW()
{
	ROS_ERROR("Called GetQuaternionW() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetQuaternionX()
{
	ROS_ERROR("Called GetQuaternionX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetQuaternionY()
{
	ROS_ERROR("Called GetQuaternionY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetQuaternionZ()
{
	ROS_ERROR("Called GetQuaternionZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
void   AHRS::ResetDisplacement()
{
	ROS_ERROR("Called ResetDisplacement() on unsupported platform");
}
void   AHRS::UpdateDisplacement( float , float ,
						   int , bool )
{
	ROS_ERROR("Called UpdateDisplacement( float accel_x_g, float accel_y_g, int update_rate_hz, bool is_moving ) on unsupported platform");
}

float  AHRS::GetVelocityX()
{
	ROS_ERROR("Called GetVelocityX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetVelocityY()
{
	ROS_ERROR("Called GetVelocityY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetVelocityZ()
{
	ROS_ERROR("Called GetVelocityZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetDisplacementX()
{
	ROS_ERROR("Called GetDisplacementX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetDisplacementY()
{
	ROS_ERROR("Called GetDisplacementY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetDisplacementZ()
{
	ROS_ERROR("Called GetDisplacementZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
double AHRS::GetAngle()
{
	ROS_ERROR("Called GetAngle() on unsupported platform");
	return std::numeric_limits<double>::max();
}
double AHRS::GetRate()
{
	ROS_ERROR("Called GetRate() on unsupported platform");
	return std::numeric_limits<double>::max();
}
void   AHRS::SetAngleAdjustment(double )
{
	ROS_ERROR("Called double angle) on unsupported platform");
}
double AHRS::GetAngleAdjustment()
{
	ROS_ERROR("Called GetAngleAdjustment() on unsupported platform");
	return std::numeric_limits<double>::max();
}
void   AHRS::Reset()
{
	ROS_ERROR("Called Reset() on unsupported platform");
}
float  AHRS::GetRawGyroX()
{
	ROS_ERROR("Called GetRawGyroX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawGyroY()
{
	ROS_ERROR("Called GetRawGyroY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawGyroZ()
{
	ROS_ERROR("Called GetRawGyroZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawAccelX()
{
	ROS_ERROR("Called GetRawAccelX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawAccelY()
{
	ROS_ERROR("Called GetRawAccelY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawAccelZ()
{
	ROS_ERROR("Called GetRawAccelZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawMagX()
{
	ROS_ERROR("Called GetRawMagX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawMagY()
{
	ROS_ERROR("Called GetRawMagY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawMagZ()
{
	ROS_ERROR("Called GetRawMagZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetPressure()
{
	ROS_ERROR("Called GetPressure() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetTempC()
{
	ROS_ERROR("Called GetTempC() on unsupported platform");
	return std::numeric_limits<float>::max();
}
AHRS::BoardYawAxis AHRS::GetBoardYawAxis()
{
	ROS_ERROR("Called GetBoardYawAxis() on unsupported platform");
	return AHRS::BoardYawAxis();
}
std::string AHRS::GetFirmwareVersion()
{
	ROS_ERROR("Called GetFirmwareVersion() on unsupported platform");
	return std::string();
}

bool AHRS::RegisterCallback( ITimestampedDataSubscriber *, void *)
{
	ROS_ERROR("Called *callback_context) on unsupported platform");
	return false;
}
bool AHRS::DeregisterCallback( ITimestampedDataSubscriber *)
{
	ROS_ERROR("Called *callback ) on unsupported platform");
	return false;
}

int AHRS::GetActualUpdateRate()
{
	ROS_ERROR("Called GetActualUpdateRate() on unsupported platform");
	return std::numeric_limits<int>::max();
}
int AHRS::GetRequestedUpdateRate()
{
	ROS_ERROR("Called GetRequestedUpdateRate() on unsupported platform");
	return std::numeric_limits<int>::max();
}

void AHRS::EnableLogging(bool)
{
	ROS_ERROR("Called bool AHRS::EnableLogging(bool enable) on unsupported platform");
}
void AHRS::SPIInit( SPI::Port, uint32_t, uint8_t)
{
	ROS_ERROR("Called AHRS::SPIInit( SPI::Port spi_port_id, uint32_t bitrate, uint8_t update_rate_hz ) on unsupported platform");
}
void AHRS::I2CInit( I2C::Port, uint8_t)
{
	ROS_ERROR("Called AHRS::I2CInit( I2C::Port i2c_port_id, uint8_t update_rate_hz ) on unsupported platform");
}
void AHRS::SerialInit(SerialPort::Port, AHRS::SerialDataType, uint8_t)
{
	ROS_ERROR("Called AHRS::SerialInit(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) on unsupported platform");
}
void AHRS::commonInit( uint8_t)
{
	ROS_ERROR("Called AHRS::commonInit( uint8_t update_rate_hz ) on unsupported platform");
}
int AHRS::ThreadFunc(IIOProvider *)
{
	ROS_ERROR("Called int AHRS::ThreadFunc(IIOProvider *io_provider) on unsupported platform");
	return -1;
}
void AHRS::InitSendable(frc::SendableBuilder&)
{
	ROS_ERROR("Called AHRS::InitSendable(frc::SendableBuilder&) on unsupported platform");
}
double AHRS::PIDGet()
{
	ROS_ERROR("Called AHRS::PIDGet() on unsupported platform");
	return std::numeric_limits<double>::max();
}
uint8_t AHRS::GetActualUpdateRateInternal(uint8_t)
{
	ROS_ERROR("Called AHRS::GetActualUpdateRateInternal(uint8_t update_rate) on unsupported platform");
	return std::numeric_limits<uint8_t>::max();
}

#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
frc::GenericHID::GenericHID(int) : m_ds(&DriverStation::GetInstance())
{
	ROS_ERROR("Called GenericHID::GenericHID(int) on unsupported platform");
}
int frc::GenericHID::GetPOV(int) const
{
	ROS_ERROR("Called GenericHID::GetPOV(int) const on unsupported platform");
	return -1;
}
double frc::GenericHID::GetRawAxis(int) const
{
	ROS_ERROR("Called GenericHID::GetRawAxis(int) const on unsupported platform");
	return std::numeric_limits<double>::max();
}
bool frc::GenericHID::GetRawButton(int) const
{
	ROS_ERROR("Called GenericHID::GetRawButton(int) const on unsupported platform");
	return false;
}
bool frc::GenericHID::GetRawButtonPressed(int)
{
	ROS_ERROR("Called GenericHID::GetRawButtonPressed(int) on unsupported platform");
	return false;
}
bool frc::GenericHID::GetRawButtonReleased(int)
{
	ROS_ERROR("Called GenericHID::GetRawButtonReleased(int) on unsupported platform");
	return false;
}

int frc::GenericHID::GetButtonCount() const
{
	ROS_ERROR("Called frc::Joystick::GetButtonCount() const on unsupported platform");
	return -1;
}

int frc::GenericHID::GetAxisCount() const
{
	ROS_ERROR("Called frc::Joystick::GetAxisCount() const on unsupported platform");
	return -1;
}

#include <frc/NidecBrushless.h>
frc::NidecBrushless::NidecBrushless(int pwmChannel, int dioChannel) : m_dio(dioChannel), m_pwm(pwmChannel)
{
	ROS_ERROR("Called NidecBrushless::NidecBrushless(int, int) on unsupported platform");
}

void frc::NidecBrushless::Set(double)
{
	ROS_ERROR("Called ::NidecBrushless::Set(double speed) on unsupported platform");
}
double frc::NidecBrushless::Get() const
{
	ROS_ERROR("Called ::NidecBrushless::Get() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::NidecBrushless::SetInverted(bool)
{
	ROS_ERROR("Called ::NidecBrushless::SetInverted(bool isInverted) on unsupported platform");
}
bool frc::NidecBrushless::GetInverted() const
{
	ROS_ERROR("Called ::NidecBrushless::GetInverted() const on unsupported platform");
	return false;
}
void frc::NidecBrushless::Disable()
{
	ROS_ERROR("Called ::NidecBrushless::Disable() on unsupported platform");
}
void frc::NidecBrushless::StopMotor()
{
	ROS_ERROR("Called ::NidecBrushless::StopMotor() on unsupported platform");
}

void frc::NidecBrushless::Enable()
{
	ROS_ERROR("Called ::NidecBrushless::Enable() on unsupported platform");
}

void frc::NidecBrushless::PIDWrite(double)
{
	ROS_ERROR("Called ::NidecBrushless::PIDWrite(double output) on unsupported platform");
}

void frc::NidecBrushless::GetDescription(wpi::raw_ostream&) const
{
	ROS_ERROR("Called ::NidecBrushless::GetDescription(wpi::raw_ostream& desc) const on unsupported platform");
}

int frc::NidecBrushless::GetChannel() const
{
	ROS_ERROR("Called ::NidecBrushless::GetChannel() const on unsupported platform");
	return -1;
}

void frc::NidecBrushless::InitSendable(SendableBuilder&)
{
	ROS_ERROR("Called ::NidecBrushless::InitSendable(SendableBuilder& builder) on unsupported platform");
}
/*frc::MotorSafety::MotorSafety()
{
	ROS_ERROR("Called MotorSafety::MotorSafety on unsupported platform");
}
frc::MotorSafety::~MotorSafety()
{
	ROS_ERROR("Called MotorSafety::~MotorSafety on unsupported platform");
}
void frc::MotorSafety::SetSafetyEnabled(bool)
{
	ROS_ERROR("Called MotorSafety::SetSafetyEnabled(bool) on unsupported platform");
}
*/
#include <frc/PWM.h>
frc::PWM::PWM(int)
{
	ROS_ERROR("Called PWM::PWM(int) on unsupported platform");
}
frc::PWM::~PWM()
{
	ROS_ERROR("Called PWM::~PWM() on unsupported platform");
}
void frc::PWM::SetRaw(uint16_t)
{
	ROS_ERROR("Called PWM::SetRaw(uint16_t value) on unsupported platform");
}
uint16_t frc::PWM::GetRaw() const
{
	ROS_ERROR("Called PWM::GetRaw() const on unsupported platform");
	return -1;
}
void frc::PWM::SetPosition(double)
{
	ROS_ERROR("Called PWM::SetPosition(double pos) on unsupported platform");
}
double frc::PWM::GetPosition() const
{
	ROS_ERROR("Called PWM::GetPosition() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::PWM::SetSpeed(double)
{
	ROS_ERROR("Called PWM::SetSpeed(double speed) on unsupported platform");
}
double frc::PWM::GetSpeed() const
{
	ROS_ERROR("Called PWM::GetSpeed() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::PWM::SetDisabled()
{
	ROS_ERROR("Called PWM::SetDisabled() on unsupported platform");
}
void frc::PWM::SetPeriodMultiplier(PeriodMultiplier)
{
	ROS_ERROR("Called PWM::SetPeriodMultiplier(PeriodMultiplier mult) on unsupported platform");
}
void frc::PWM::SetZeroLatch()
{
	ROS_ERROR("Called PWM::SetZeroLatch() on unsupported platform");
}
void frc::PWM::EnableDeadbandElimination(bool)
{
	ROS_ERROR("Called PWM::EnableDeadbandElimination(bool eliminateDeadband) on unsupported platform");
}
void frc::PWM::SetBounds(double, double, double, double, double)
{
	ROS_ERROR("Called PWM::SetBounds(double max, double deadbandMax, double center, double deadbandMin, double min) on unsupported platform");
}
void frc::PWM::SetRawBounds(int, int, int, int, int)
{
	ROS_ERROR("Called PWM::SetRawBounds(int max, int deadbandMax, int center, int deadbandMin, int min) on unsupported platform");
}
void frc::PWM::GetRawBounds(int32_t*, int32_t*, int32_t*, int32_t*, int32_t*)
{
	ROS_ERROR("Called PWM::GetRawBounds(int32_t* max, int32_t* deadbandMax, int32_t* center, int32_t* deadbandMin, int32_t* min) on unsupported platform");
}
void frc::PWM::InitSendable(SendableBuilder&)
{
	ROS_ERROR("Called PWM::InitSendable(SendableBuilder& builder) on unsupported platform");
}

void frc::PWM::StopMotor()
{
	ROS_ERROR("Called PWM::StopMotor() on unsupported platform");
}

void frc::PWM::GetDescription(wpi::raw_ostream &) const
{
	ROS_ERROR("Called PWM::GetDescription(wpi::raw_ostream &) on unsupported platform");
}

#include <frc/RobotBase.h>
bool frc::RobotBase::IsAutonomous() const
{
	ROS_ERROR("Called RobotBase::IsAutonomous() const on unsupported platform");
	return false;
}
bool frc::RobotBase::IsDisabled() const
{
	ROS_ERROR("Called RobotBase::IsDisabled() const on unsupported platform");
	return false;
}
bool frc::RobotBase::IsOperatorControl() const
{
	ROS_ERROR("Called RobotBase::IsOperatorControl() const on unsupported platform");
	return false;
}
frc::RobotBase::RobotBase() : m_ds(DriverStation::GetInstance())
{
	ROS_ERROR("Called RobotBase::RobotBase() on unsupported platform");
}
frc::RobotBase::~RobotBase()
{
	ROS_ERROR("Called RobotBase::~RobotBase() on unsupported platform");
}

#include <frc/PIDSource.h>
void frc::PIDSource::SetPIDSourceType(PIDSourceType)
{
	ROS_ERROR("Called frc::PIDSource::SetPIDSourceType(PIDSourceType pidSource) on unsupported platform");
}
PIDSourceType frc::PIDSource::GetPIDSourceType() const
{
	ROS_ERROR("Called frc::PIDSource::GetPIDSourceType() on unsupported platform");
	return frc::PIDSourceType::kDisplacement;
}

#include <frc/Notifier.h>
frc::Notifier::Notifier(std::function<void ()>)
{
	ROS_ERROR("Called frc::Notifier::Notifier(std::function<void ()>) on unsupported platform");
}
frc::Notifier::~Notifier()
{
	ROS_ERROR("Called frc::Notifier::~Notifier() on unsupported platform");
}

#include <frc/SpeedController.h>
void frc::SpeedController::SetVoltage(units::volt_t output)
{
	ROS_ERROR("Called frc::SpeedController::Set(volt_t output) on unsupported platform");
}
