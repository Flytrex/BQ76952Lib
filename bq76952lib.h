/*
* Description :   Header file of BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   Pranjal Joshi
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

class bq76952 {
private:
	TwoWire *m_I2C;
	int m_alertPin;
	bool m_loud;

public:
	bq76952() = default;
	int begin(byte alertPin, TwoWire *I2C, bool loud = false);
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

	int directCommandWrite(byte command, size_t size, int data);
	int directCommandRead(byte command, size_t size, int *o_data);


	void setCellOvervoltageProtection(unsigned int, unsigned int);
	void setCellUndervoltageProtection(unsigned int, unsigned int);
	void setChargingOvercurrentProtection(byte, byte);
	void setChargingTemperatureMaxLimit(signed int, byte);
	void setDischargingOvercurrentProtection(byte, byte);
	void setDischargingShortcircuitProtection(bq76952_scd_thresh, unsigned int);
	void setDischargingTemperatureMaxLimit(signed int, byte);

	bool subCommandWithResponse(uint16_t subCommand, uint8_t* buffer, uint8_t* buffer_len );
	int directCommand(byte command, uint16_t *o_response);
	void subCommand(unsigned int);
	unsigned int subCommandResponseInt(void);
	void enterConfigUpdate(void);
	void exitConfigUpdate(void);
	byte computeChecksum(byte, byte, bool reset=false);
	void writeDataMemory(unsigned int , unsigned int, byte);
	byte readDataMemory(unsigned int);
};