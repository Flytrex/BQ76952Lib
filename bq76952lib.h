/*
* Description :   Header file of BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   Pranjal Joshi, Grisha Revzin @ Flytrex
* Date        :   17/10/2020 
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/
#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>

enum bq76952_thermistor {
	TS1,
	TS2,
	TS3,
	HDQ,
	DCHG,
	DDSG
};

enum bq76952_fet {
	CHG,
	DCH,
	ALL
};

enum bq76952_fet_state {
	OFF,
	ON
};

enum bq76952_scd_thresh {
	SCD_10,
	SCD_20,
	SCD_40,
	SCD_60,
	SCD_80,
	SCD_100,
	SCD_125,
	SCD_150,
	SCD_175,
	SCD_200,
	SCD_250,
	SCD_300,
	SCD_350,
	SCD_400,
	SCD_450,
	SCD_500
};

typedef union protection {
	struct {
		uint8_t SC_DCHG            :1;
		uint8_t OC2_DCHG           :1;
		uint8_t OC1_DCHG           :1;
		uint8_t OC_CHG             :1;
		uint8_t CELL_OV            :1;
		uint8_t CELL_UV            :1;
	} bits;
} bq76952_protection_t;

typedef union temperatureProtection {
	struct {
		uint8_t OVERTEMP_FET		:1;
		uint8_t OVERTEMP_INTERNAL	:1;
		uint8_t OVERTEMP_DCHG		:1;
		uint8_t OVERTEMP_CHG		:1;
		uint8_t UNDERTEMP_INTERNAL	:1;
		uint8_t UNDERTEMP_DCHG		:1;
		uint8_t UNDERTEMP_CHG		:1;
	} bits;
} bq76952_temperature_t;

class BQConfig;

class BQRegister {
	int m_type;
	byte m_value[4];
	uint16_t m_address;
	const char *m_descr;
public:
	BQRegister(void);
	BQRegister(const BQRegister &rhs) = default;
	BQRegister(int address, byte val, const char *descr = nullptr);
	BQRegister(int address, float val, const char *descr = nullptr);
	BQRegister(int address, uint16_t val, const char *descr = nullptr);
	BQRegister(int address, uint32_t val, const char *descr = nullptr);

	void setI8(byte val);
	void setF32(float val);
	void setI16(short val);
	void setI32(int32_t val);
	byte getI8(void) const;
	float getF32(void) const;
	short getI16(void) const;
	int32_t getI32(void) const;
	size_t getSize(void) const;
	int getType(void) const;
	int getAddress(void) const;
	const char *getDescription(void) const;

	friend class BQConfig;
};

#define BQ76952_TOT_REGISTERS 272

class bq76952;

class BQConfig {
	friend class bq76952;
	BQRegister m_registers[BQ76952_TOT_REGISTERS];
	int m_checksum;
public:
	BQConfig() = default;
	static void getDefaultConfig(BQConfig *buf);
	void setRegister(size_t i, const BQRegister &reg);
	int CRC32(void) const;
};

#define BQ_I2C_DEFAULT_ADDRESS        0x08

class bq76952 {
private:
	TwoWire *m_I2C;
	int m_I2C_Address;
	int m_alertPin;
	bool m_loud;
	bool m_inUpdateConfig;
	byte m_RAMAccessBuffer[32];
	BQConfig m_currentConfig;
	int m_defaultConfigCRC;

	int m_writeBulkAddress(int command, size_t size);
	int m_bulkWrite(int address, int size, byte *data);
	int m_pollTransferSetup(int address, unsigned short maxWait = 10);

	int m_bulkRead(int address, int expectedSize, byte *o_data);
	int m_directCommandWrite(byte command, size_t size, int data);
	int m_directCommandRead(byte command, size_t size, int *o_data);
	int m_subCommandRead(int address, size_t size, byte *o_data);
	int m_subCommandWrite(int command, size_t size = 0, byte *data = nullptr);

	int m_memReadI8(int address, byte *o_data);
	int m_memReadI16(int address, short *o_data);
	int m_memReadI32(int address, int *o_data);
	int m_memReadF32(int address, float *o_data);

	int m_memWriteI8(int address, byte data);
	int m_memWriteI16(int address, short data);
	int m_memWriteI32(int address, int data);
	int m_memWriteF32(int address, float data);

	int m_enterConfigUpdate(void);
	int m_exitConfigUpdate(void);

	int m_configDownload(BQConfig *buffer);

public:
	bq76952() = default;
	int begin(byte alertPin, TwoWire *I2C, bool loud = false, byte address = BQ_I2C_DEFAULT_ADDRESS);
	int configUpload(const BQConfig *config);
	int configChecksum(void) const;
	


	void reset(void);
	bool isConnected(void);
	unsigned int getCellVoltage(byte cellNumber);
	void getAllCellVoltages(unsigned int* cellArray);
	unsigned int getCurrent(void);
	float getInternalTemp(void);
	float getThermistorTemp(bq76952_thermistor);
	bq76952_protection_t getProtectionStatus(void);
	bq76952_temperature_t getTemperatureStatus(void);
	void setFET(bq76952_fet, bq76952_fet_state);
	bool isDischarging(void);
	bool isCharging(void);

	void setCellOvervoltageProtection(unsigned int, unsigned int);
	void setCellUndervoltageProtection(unsigned int, unsigned int);
	void setChargingOvercurrentProtection(byte, byte);
	void setChargingTemperatureMaxLimit(signed int, byte);
	void setDischargingOvercurrentProtection(byte, byte);
	void setDischargingShortcircuitProtection(bq76952_scd_thresh, unsigned int);
	void setDischargingTemperatureMaxLimit(signed int, byte);
};
