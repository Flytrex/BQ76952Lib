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
	BQ_THERMISTOR_TS1 = 0x70,
	BQ_THERMISTOR_TS2 = 0x72,
	BQ_THERMISTOR_TS3 = 0x74,
	BQ_THERMISTOR_HDQ = 0x76,
	BQ_THERMISTOR_DCHG = 0x78,
	BQ_THERMISTOR_DDSG = 0x7A
};

struct BQTemps_C {
	float 	ts1,
			ts2,
			ts3,
			hdq,
			dchg,
			ddsg;
};

#define BQ_N_CELLS 16

enum bq76952_voltages {
	BQ_VOLTAGE_CELL1 = 0x14,
	BQ_VOLTAGE_CELL2 = 0x16,
	BQ_VOLTAGE_CELL3 = 0x18,
	BQ_VOLTAGE_CELL4 = 0x1A,
	BQ_VOLTAGE_CELL5 = 0x1C,
	BQ_VOLTAGE_CELL6 = 0x1E,
	BQ_VOLTAGE_CELL7 = 0x20,

	BQ_VOLTAGE_CELL8 = 0x22,
	BQ_VOLTAGE_CELL9 = 0x24,
	BQ_VOLTAGE_CELL10 = 0x26,
	BQ_VOLTAGE_CELL11 = 0x28,
	BQ_VOLTAGE_CELL12 = 0x30,
	BQ_VOLTAGE_CELL13 = 0x2C,
	BQ_VOLTAGE_CELL14 = 0x2E,
	BQ_VOLTAGE_CELL15 = 0x30,
	BQ_VOLTAGE_CELL16 = 0x32,
	BQ_VOLTAGE_STACK = 0x34,	
	BQ_VOLTAGE_PACK = 0x36,
	BQ_VOLTAGE_LD = 0x38
};

struct BQVoltages_V {
	float cells[16];
	float stack, pack, ld;
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

struct BQPrimaryState {
	bool charging;
	bool precharging;
	bool discharging;
	bool predischarging;
};

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
	float getUserAScaling(void) const;
	float getUserVScaling(void) const;
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
	float m_userA_amps;				/* How many amperes are in one unit of 'userA' registers? */
	float m_userV_volts;			/* How many volts in one unit of 'userV' registers? */

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

	void m_updateUnits(void);

public:
	bq76952() = default;

	/* Init */
	int begin(byte alertPin, TwoWire *I2C, bool loud = false, byte address = BQ_I2C_DEFAULT_ADDRESS);
	
	/* Upload config */
	int configUpload(const BQConfig *config);
	
	/* Last config's checksum */
	int configChecksum(void) const;

	/* Send a reset command (this will reset the configuration to OTP) */
	int reset(void);
	
	/* Read primary current */
	int getCC2Current(float *out_A);
	
	/* Get voltage on channel */
	int getVoltage(bq76952_voltages channel, float *o_V);
	
	/* Get all voltages -- faster than reading one by one */
	int getVoltages(BQVoltages_V *out);			
	
	/* Get thermistor value */
	int getThermistorTemp(bq76952_thermistor thermistor, float *o_tempC);

	/* Get all thermistors -- faster than reading one by one */
	int getThermistors(BQTemps_C *out);

	/* Get chip die temp */
	float getDieTemp(float *o_intTempC);

	/* Primary FET state */
	int getPrimaryState(BQPrimaryState *out);

	/* FTX: not tested */	
	bq76952_protection_t getProtectionStatus(void);
	bq76952_temperature_t getTemperatureStatus(void);
	void setFET(bq76952_fet, bq76952_fet_state);
	void setCellOvervoltageProtection(unsigned int, unsigned int);
	void setCellUndervoltageProtection(unsigned int, unsigned int);
	void setChargingOvercurrentProtection(byte, byte);
	void setChargingTemperatureMaxLimit(signed int, byte);
	void setDischargingOvercurrentProtection(byte, byte);
	void setDischargingShortcircuitProtection(bq76952_scd_thresh, unsigned int);
	void setDischargingTemperatureMaxLimit(signed int, byte);
};
