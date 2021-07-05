
/*==================[inclusions]=============================================*/

#include "MPU_Libreria.h"   /* <= sAPI MPU60X0 header */
#include "main.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void mpu60X0InitializeControlStructure( void );
static int8_t MPU6050_WriteRegister( uint8_t subAddress, uint8_t data );
static int8_t mpu60X0CalibrateGyro( void );
static int8_t mpu60X0SetGyroRange( MPU60X0_GyroRange_t range );
static int8_t mpu60X0SetDlpfBandwidth( MPU60X0_DlpfBandwidth_t bandwidth );
static int8_t mpu60X0SetSrd( uint8_t srd );

static MPU60X0_control_t control;

/*==================[internal functions definition]==========================*/

static void mpu60X0InitializeControlStructure( void )
{
	control._tempScale = 333.87f;
	control._tempOffset = 21.0f;
	control._numSamples = 100;
	control._axs = 1.0f;
	control._ays = 1.0f;
	control._azs = 1.0f;
	control.tX[0] = 1;
	control.tX[1] = 0;
	control.tX[2] = 0;
	control.tY[0] = 0;
	control.tY[1] = 1;
	control.tY[2] = 0;
	control.tZ[0] = 0;
	control.tZ[1] = 0;
	control.tZ[2] = 1;
}

static int8_t MPU6050_WriteRegister( uint8_t subAddress, uint8_t data )
{
	uint8_t transmitDataBuffer[2];
	control.address = MPU60X0_ADDRESS_0;
	transmitDataBuffer[0] = subAddress;
	transmitDataBuffer[1] = data;
	//Data = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, control.address, &transmitDataBuffer, 2, 1000);

	//HAL_I2C_Mem_Write(&hi2c1, control.address, subAddress, 1,&transmitDataBuffer, 1, 1000);
	HAL_Delay(10);
	//transmitDataBuffer = 0x00;
	HAL_I2C_Mem_Read(&hi2c1, control.address, subAddress, 1,&transmitDataBuffer, 1, 1000);
	if(transmitDataBuffer[1] != data){
		return -1;
	}
	else{
		return 1;
	}
}


uint8_t MPU6050_Init (MPU60X0_address_t address)
{
	uint8_t check;
	mpu60X0InitializeControlStructure();
	control.address = address;

	if(MPU6050_WriteRegister(MPU60X0_PWR_MGMT_1,MPU60X0_CLOCK_SEL_PLL_X_GYRO) < 0){
		return -1;
	}
	if(MPU6050_WriteRegister(MPU60X0_USER_CTRL,MPU60X0_I2C_MST_EN) < 0){
		return -2;
	}
	if(MPU6050_WriteRegister(MPU60X0_I2C_MST_CTRL,MPU60X0_I2C_MST_CLK) < 0){
		return -3;
	}
	MPU6050_WriteRegister(MPU60X0_PWR_MGMT_1,MPU60X0_PWR_RESET);
	HAL_Delay(1);
	if(MPU6050_WriteRegister(MPU60X0_PWR_MGMT_1,MPU60X0_CLOCK_SEL_PLL_X_GYRO) < 0){
		return -4;
	}
	if(HAL_I2C_Mem_Read (&hi2c1, MPU60X0_ADDRESS_0,MPU60X0_WHO_AM_I,1, &check, 1, 1000) < 0){
		return -5;
	}
	if(MPU6050_WriteRegister(MPU60X0_PWR_MGMT_2,MPU60X0_SEN_ENABLE) < 0){
		return -6;
	}
	if(MPU6050_WriteRegister(MPU60X0_ACCEL_CONFIG,MPU60X0_ACCEL_FS_SEL_16G) < 0){
		return -7;
	}
	control._accelScale = MPU60X0_G * 16.0f / 32767.5f; // setting the accel scale to 16G
	control._accelRange = MPU60X0_ACCEL_RANGE_16G;
	if(MPU6050_WriteRegister(MPU60X0_GYRO_CONFIG,MPU60X0_GYRO_FS_SEL_2000DPS) < 0){
		return -8;
	}
	// setting the gyro scale to 2000DPS
	control._gyroScale = 2000.0f / 32767.5f * MPU60X0_D2R;
	control._gyroRange = MPU60X0_GYRO_RANGE_2000DPS;
	if(MPU6050_WriteRegister(MPU60X0_CONFIG,MPU60X0_DLPF_184HZ) < 0){
		return -9;
	}
	control._bandwidth = MPU60X0_DLPF_BANDWIDTH_184HZ;
	if(MPU6050_WriteRegister(MPU60X0_SMPRT_DIV,0x00) < 0){
		return -10;
	}
	control._srd = 0;
	if(MPU6050_WriteRegister(MPU60X0_USER_CTRL,MPU60X0_I2C_MST_EN) < 0){
		return -11;
	}
	if(MPU6050_WriteRegister(MPU60X0_I2C_MST_CTRL,MPU60X0_I2C_MST_CLK) < 0){
		return -12;
	}
	if(MPU6050_WriteRegister(MPU60X0_PWR_MGMT_1,MPU60X0_CLOCK_SEL_PLL_X_GYRO) < 0){
		return -13;
	}
	if(mpu60X0CalibrateGyro() < 0){
		return -14;
	}
	return 1;
}

static int8_t mpu60X0CalibrateGyro( void )
{
	// set the range, bandwidth, and srd
	if (mpu60X0SetGyroRange(MPU60X0_GYRO_RANGE_250DPS) < 0) {
		return -1;
	}
	if (mpu60X0SetDlpfBandwidth(MPU60X0_DLPF_BANDWIDTH_21HZ) < 0) {
		return -2;
	}
	if (mpu60X0SetSrd(19) < 0) {
		return -3;
	}

	// take samples and find bias
	control._gxbD = 0;
	control._gybD = 0;
	control._gzbD = 0;
	for (uint8_t i=0; i < control._numSamples; i++) {
		mpu60X0Read();
		control._gxbD += ((mpu60X0GetGyroX_rads() + control._gxb)/control._numSamples);
		control._gybD += ((mpu60X0GetGyroY_rads() + control._gyb)/control._numSamples);
		control._gzbD += ((mpu60X0GetGyroZ_rads() + control._gzb)/control._numSamples);
		HAL_Delay(20);
	}
	control._gxb = (float)control._gxbD;
	control._gyb = (float)control._gybD;
	control._gzb = (float)control._gzbD;

	// set the range, bandwidth, and srd back to what they were
	if (mpu60X0SetGyroRange(control._gyroRange) < 0) {
		return -4;
	}
	if (mpu60X0SetDlpfBandwidth(control._bandwidth) < 0) {
		return -5;
	}
	if (mpu60X0SetSrd(control._srd) < 0) {
		return -6;
	}
	return 1;
}


static int8_t mpu60X0SetGyroRange( MPU60X0_GyroRange_t range )
{
	switch(range) {
	case MPU60X0_GYRO_RANGE_250DPS: {
		// setting the gyro range to 250DPS
		if(MPU6050_WriteRegister(MPU60X0_GYRO_CONFIG, MPU60X0_GYRO_FS_SEL_250DPS) < 0){
			return -1;
		}
		// setting the gyro scale to 250DPS
		control._gyroScale = 250.0f/32767.5f * MPU60X0_D2R;
		break;
	}
	case MPU60X0_GYRO_RANGE_500DPS: {
		// setting the gyro range to 500DPS
		if(MPU6050_WriteRegister(MPU60X0_GYRO_CONFIG, MPU60X0_GYRO_FS_SEL_500DPS) < 0){
			return -1;
		}
		// setting the gyro scale to 500DPS
		control._gyroScale = 500.0f/32767.5f * MPU60X0_D2R;
		break;
	}
	case MPU60X0_GYRO_RANGE_1000DPS: {
		// setting the gyro range to 1000DPS
		if(MPU6050_WriteRegister(MPU60X0_GYRO_CONFIG, MPU60X0_GYRO_FS_SEL_1000DPS) < 0){
			return -1;
		}
		// setting the gyro scale to 1000DPS
		control._gyroScale = 1000.0f/32767.5f * MPU60X0_D2R;
		break;
	}
	case MPU60X0_GYRO_RANGE_2000DPS: {
		// setting the gyro range to 2000DPS
		if(MPU6050_WriteRegister(MPU60X0_GYRO_CONFIG, MPU60X0_GYRO_FS_SEL_2000DPS) < 0){
			return -1;
		}
		// setting the gyro scale to 2000DPS
		control._gyroScale = 2000.0f/32767.5f * MPU60X0_D2R;
		break;
	}
	}
	control._gyroRange = range;
	return 1;
}

static int8_t mpu60X0SetDlpfBandwidth( MPU60X0_DlpfBandwidth_t bandwidth )
{
	switch (bandwidth) {
	case MPU60X0_DLPF_BANDWIDTH_260HZ: {
		// setting accel bandwidth to 260Hz and gyro bandwidth to 256 Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_260HZ) < 0) {
			return -1;
		}
		break;
	}
	case MPU60X0_DLPF_BANDWIDTH_184HZ: {
		// setting accel bandwidth to 184Hz and gyro bandwidth to 189Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_184HZ) < 0) {
			return -1;
		}
		break;
	}
	case MPU60X0_DLPF_BANDWIDTH_94HZ: {
		// setting accel bandwidth to 94 Hz and gyro bandwidth to 98 Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_94HZ) < 0) {
			return -1;
		}
		break;
	}
	case MPU60X0_DLPF_BANDWIDTH_44HZ: {
		// setting accel bandwidth to 44 Hz and gyro bandwidth to 42 Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_44HZ) < 0) {
			return -1;
		}
		break;
	}
	case MPU60X0_DLPF_BANDWIDTH_21HZ: {
		// setting accel bandwidth to 21 Hz and gyro bandwidth to 20 Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_21HZ) < 0) {
			return -1;
		}
		break;
	}
	case MPU60X0_DLPF_BANDWIDTH_10HZ: {
		// setting accel bandwidth to 10 Hz and gyro bandwidth to 10 Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_10HZ) < 0) {
			return -1;
		}
		break;
	}
	case MPU60X0_DLPF_BANDWIDTH_5HZ: {
		// setting accel bandwidth to 5 Hz and gyro bandwidth to 5 Hz
		if (MPU6050_WriteRegister(MPU60X0_CONFIG, MPU60X0_DLPF_5HZ) < 0) {
			return -1;
		}
		break;
	}
	}
	control._bandwidth = bandwidth;
	return 1;
}

static int8_t mpu60X0SetSrd( uint8_t srd )
{
	/* setting the sample rate divider */
	if (MPU6050_WriteRegister(MPU60X0_SMPRT_DIV, srd) < 0) { // setting the sample rate divider
		return -1;
	}
	control._srd = srd;
	return 1;
}

//Read sensor registers and store data at control structure
bool_t mpu60X0Read(void)
{
	// grab the data from the MPU60X0
	//HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,MPU60X0_WHO_AM_I,1, &check, 1, 1000)
	//if( !mpu60X0ReadRegisters(MPU60X0_ACCEL_OUT, 21) ){
	//uint8_t control._buffer[21];
	//HAL_I2C_Mem_Read (&hi2c1, control.address, MPU60X0_ACCEL_OUT, 1, check, 21, 1000);
	//	if( HAL_I2C_Mem_Read(&hi2c1, control.address ,MPU60X0_ACCEL_OUT,1, control._buffer, 21, 1000) == 0){
	//		return 0;
	//	}
	HAL_I2C_Mem_Read(&hi2c1, control.address ,MPU60X0_ACCEL_OUT,1, control._buffer, 21, 1000);

	// combine into 16 bit values
	control._axcounts = (((int16_t)control._buffer[0]) << 8)  | control._buffer[1];
	control._aycounts = (((int16_t)control._buffer[2]) << 8)  | control._buffer[3];
	control._azcounts = (((int16_t)control._buffer[4]) << 8)  | control._buffer[5];
	control._tcounts  = (((int16_t)control._buffer[6]) << 8)  | control._buffer[7];
	control._gxcounts = (((int16_t)control._buffer[8]) << 8)  | control._buffer[9];
	control._gycounts = (((int16_t)control._buffer[10]) << 8) | control._buffer[11];
	control._gzcounts = (((int16_t)control._buffer[12]) << 8) | control._buffer[13];
	// transform and convert to float values
	control._ax = (((float)(control.tX[0]*control._axcounts + control.tX[1]*control._aycounts + control.tX[2]*control._azcounts) * control._accelScale) - control._axb)*control._axs;
	control._ay = (((float)(control.tY[0]*control._axcounts + control.tY[1]*control._aycounts + control.tY[2]*control._azcounts) * control._accelScale) - control._ayb)*control._ays;
	control._az = (((float)(control.tZ[0]*control._axcounts + control.tZ[1]*control._aycounts + control.tZ[2]*control._azcounts) * control._accelScale) - control._azb)*control._azs;
	control._gx = ((float) (control.tX[0]*control._gxcounts + control.tX[1]*control._gycounts + control.tX[2]*control._gzcounts) * control._gyroScale) -  control._gxb;
	control._gy = ((float) (control.tY[0]*control._gxcounts + control.tY[1]*control._gycounts + control.tY[2]*control._gzcounts) * control._gyroScale) -  control._gyb;
	control._gz = ((float) (control.tZ[0]*control._gxcounts + control.tZ[1]*control._gycounts + control.tZ[2]*control._gzcounts) * control._gyroScale) -  control._gzb;
	control._t = ((((float) control._tcounts)  - control._tempOffset)/ control._tempScale) + control._tempOffset;
	return 1;
}

// Returns the accelerometer measurement in the x direction, m/s/s
float mpu60X0GetAccelX_mss( void )
{
	return control._ax;
}

// Returns the accelerometer measurement in the y direction, m/s/s
float mpu60X0GetAccelY_mss( void )
{
	return control._ay;
}

// Returns the accelerometer measurement in the z direction, m/s/s
float mpu60X0GetAccelZ_mss( void )
{
	return control._az;
}

// Returns the gyroscope measurement in the x direction, rad/s
float mpu60X0GetGyroX_rads( void )
{
	return control._gx;
}

// Returns the gyroscope measurement in the y direction, rad/s
float mpu60X0GetGyroY_rads( void )
{
	return control._gy;
}

// Returns the gyroscope measurement in the z direction, rad/s
float mpu60X0GetGyroZ_rads( void )
{
	return control._gz;
}

// Returns the die temperature, C
float mpu60X0GetTemperature_C( void )
{
	return control._t;
}

