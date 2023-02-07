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
	BQ_THERMISTOR_INT = 0x68,
	BQ_THERMISTOR_CFETOFF = 0x6A,
	BQ_THERMISTOR_DFETOFF = 0x6C,
	BQ_THERMISTOR_ALERT = 0x6E,
	BQ_THERMISTOR_TS1 = 0x70,
	BQ_THERMISTOR_TS2 = 0x72,
	BQ_THERMISTOR_TS3 = 0x74,
	BQ_THERMISTOR_HDQ = 0x76,
	BQ_THERMISTOR_DCHG = 0x78,
	BQ_THERMISTOR_DDSG = 0x7A
};

struct BQTemps_C {
	/* temperature at the given BQ76952 pin
	   (check your config to see which ones're configured 
	   as thermistors and what do they actually measure)*/
	float 	int_,
			cfetoff,
			dfetoff,
			alert,
			ts1,
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
	float cells[BQ_N_CELLS];
	float 	STACK, 	/* Top-of-stack voltage, internal, no pin */
			PACK, 	/* PACK pin voltage */
			LD;		/* LD pin voltage */
};

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

struct BQCalibration {
	/* This class contains only the values that are likely to be different in each unit
		as opposed to values that are likely to be shared in a whole model run. */
	uint16_t cellGain[BQ_N_CELLS];
    uint16_t packGain;
    uint16_t tosGain;
    uint16_t ldGain;
    uint16_t currentOffset;
    float senseResistor;
};

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
	void applyCalibration(const BQCalibration &calib);
	void getCalibration(BQCalibration &calib);
};

#define BQ_ALERT_ENC(reg, bit) (reg << 8 | bit)

enum BQSafety {
	/* Safety Alert A */
	BQSafety_A_SCD		= BQ_ALERT_ENC(0x02, 7),
	BQSafety_A_OCD2		= BQ_ALERT_ENC(0x02, 6),
	BQSafety_A_OCD1		= BQ_ALERT_ENC(0x02, 5),
	BQSafety_A_OCC		= BQ_ALERT_ENC(0x02, 4),
	BQSafety_A_COV		= BQ_ALERT_ENC(0x02, 3),
	BQSafety_A_CUV		= BQ_ALERT_ENC(0x02, 2),

	/* Safety Status A */
	BQSafety_F_SCD		= BQ_ALERT_ENC(0x03, 7),
	BQSafety_F_OCD2		= BQ_ALERT_ENC(0x03, 6),
	BQSafety_F_OCD1		= BQ_ALERT_ENC(0x03, 5),
	BQSafety_F_OCC		= BQ_ALERT_ENC(0x03, 4),
	BQSafety_F_COV		= BQ_ALERT_ENC(0x03, 3),
	BQSafety_F_CUV		= BQ_ALERT_ENC(0x03, 2),

	/* Safety Alert B */
	BQSafety_A_OTF		= BQ_ALERT_ENC(0x04, 7),
	BQSafety_A_OTINT		= BQ_ALERT_ENC(0x04, 6),
	BQSafety_A_OTD		= BQ_ALERT_ENC(0x04, 5),
	BQSafety_A_OTC		= BQ_ALERT_ENC(0x04, 4),
	BQSafety_A_UTINT		= BQ_ALERT_ENC(0x04, 2),
	BQSafety_A_UTD		= BQ_ALERT_ENC(0x04, 1),
	BQSafety_A_UTC		= BQ_ALERT_ENC(0x04, 0),

	/* Safety Status B */
	BQSafety_F_OTF		= BQ_ALERT_ENC(0x05, 7),
	BQSafety_F_OTINT		= BQ_ALERT_ENC(0x05, 6),
	BQSafety_F_OTD		= BQ_ALERT_ENC(0x05, 5),
	BQSafety_F_OTC		= BQ_ALERT_ENC(0x05, 4),
	BQSafety_F_UTINT		= BQ_ALERT_ENC(0x05, 2),
	BQSafety_F_UTD		= BQ_ALERT_ENC(0x05, 1),
	BQSafety_F_UTC		= BQ_ALERT_ENC(0x05, 0),
	
	/* Safety Alert C */
	BQSafety_A_OCD3		= BQ_ALERT_ENC(0x06, 7),
	BQSafety_A_SCDL		= BQ_ALERT_ENC(0x06, 6),
	BQSafety_A_OCDL		= BQ_ALERT_ENC(0x06, 5),
	BQSafety_A_COVL		= BQ_ALERT_ENC(0x06, 4),
	BQSafety_A_PTOS		= BQ_ALERT_ENC(0x06, 3),

	/* Safety Status C */
	BQSafety_F_OCD3	= BQ_ALERT_ENC(0x07, 7),			
	BQSafety_F_SCDL	= BQ_ALERT_ENC(0x07, 6),
	BQSafety_F_OCDL	= BQ_ALERT_ENC(0x07, 5),
	BQSafety_F_COVL	= BQ_ALERT_ENC(0x07, 4),
	BQSafety_F_PTOS	= BQ_ALERT_ENC(0x07, 3),

	/* PF Alert A */
	BQSafety_A_CUDEP	= BQ_ALERT_ENC(0x0A, 7), // # 17 -- first "permanent fault"
	BQSafety_A_SOTF		= BQ_ALERT_ENC(0x0A, 6),
	BQSafety_A_SOT		= BQ_ALERT_ENC(0x0A, 4),
	BQSafety_A_SOCD		= BQ_ALERT_ENC(0x0A, 3),
	BQSafety_A_SOCC		= BQ_ALERT_ENC(0x0A, 2),
	BQSafety_A_SOV		= BQ_ALERT_ENC(0x0A, 1),
	BQSafety_A_SUV		= BQ_ALERT_ENC(0x0A, 0),
	
	/* PF Status A */
	BQSafety_F_CUDEP	= BQ_ALERT_ENC(0x0B, 7),
	BQSafety_F_SOTF	= BQ_ALERT_ENC(0x0B, 6),
	BQSafety_F_SOT	= BQ_ALERT_ENC(0x0B, 4),
	BQSafety_F_SOCD	= BQ_ALERT_ENC(0x0B, 3),
	BQSafety_F_SOCC	= BQ_ALERT_ENC(0x0B, 2),
	BQSafety_F_SOV	= BQ_ALERT_ENC(0x0B, 1),
	BQSafety_F_SUV	= BQ_ALERT_ENC(0x0B, 0),

	/* PF Alert B */
	BQSafety_A_SCDL_PF	= BQ_ALERT_ENC(0x0C, 7),
	BQSafety_A_VIMA		= BQ_ALERT_ENC(0x0C, 4),
	BQSafety_A_VIMR		= BQ_ALERT_ENC(0x0C, 3),
	BQSafety_A_2LVL		= BQ_ALERT_ENC(0x0C, 2),
	BQSafety_A_DFETF	= BQ_ALERT_ENC(0x0C, 1),
	BQSafety_A_CFETF	= BQ_ALERT_ENC(0x0C, 0),

	/* PF Status B */
	BQSafety_F_SCDL_PF	= BQ_ALERT_ENC(0x0D, 7),
	BQSafety_F_VIMA		= BQ_ALERT_ENC(0x0D, 4),
	BQSafety_F_VIMR		= BQ_ALERT_ENC(0x0D, 3),
	BQSafety_F_2LVL		= BQ_ALERT_ENC(0x0D, 2),
	BQSafety_F_DFETF	= BQ_ALERT_ENC(0x0D, 1),
	BQSafety_F_CFETF	= BQ_ALERT_ENC(0x0D, 0),

	/* PF Alert C */
	BQSafety_A_HWMX		= BQ_ALERT_ENC(0x0E, 6),
	BQSafety_A_VSSF		= BQ_ALERT_ENC(0x0E, 5),
	BQSafety_A_VREF		= BQ_ALERT_ENC(0x0E, 4),
	BQSafety_A_LFOF		= BQ_ALERT_ENC(0x0E, 3),

	/* PF Status C */
	BQSafety_F_HWMX		= BQ_ALERT_ENC(0x0F, 6),
	BQSafety_F_VSSF		= BQ_ALERT_ENC(0x0F, 5),
	BQSafety_F_VREF		= BQ_ALERT_ENC(0x0F, 4),
	BQSafety_F_LFOF		= BQ_ALERT_ENC(0x0F, 3),

	/* PF Alert D */
	BQSafety_A_TOSF		= BQ_ALERT_ENC(0x10, 0),

	/* PF Status D */
	BQSafety_F_TOSF		= BQ_ALERT_ENC(0x11, 0)
};

#define BQ_N_ALERTS 36
#define BQ_ITER_PERMAMENT_FAULT_START 17
#define BQ_ITER_N_PFAULTS (BQ_N_ALERTS - BQ_ITER_PERMAMENT_FAULT_START)

extern BQSafety const BQ_ALERT_ITERABLE[BQ_N_ALERTS];
extern BQSafety const BQ_FAULT_ITERABLE[BQ_N_ALERTS];

class BQSafetyState {
	friend class bq76952;
	byte m_registerBlock[14];
	bool m_readSafetyVal(int reg, int bit) const;
public:
	bool getSafetyFlag(BQSafety flag) const;
	static const char *safetyFlagToString(BQSafety flag);
	int snprintLockouts(char *buffer, size_t size);
};

#define BQ_I2C_DEFAULT_ADDRESS 0x08

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
	int m_bulkWrite(int address, int size, byte *data, bool skipTxSetup=false);
	int m_pollTransferSetup(int address, unsigned short maxWait = 50);

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
	int init(byte alertPin, TwoWire *I2C, bool loud = false, byte address = BQ_I2C_DEFAULT_ADDRESS);
	
	/* Begin (initial config download and initialization) */
	int begin();

	/* Upload config */
	int configUpload(const BQConfig *config);
	
	/* Last config's checksum */
	int configChecksum(void) const;

	/* Send a reset command (this will reset the configuration to OTP) */
	int reset(void);
	
	/* Get primary current: 3 ms update rate by default */
	int getCC2Current(float *out_A);

	/* Get average current  */
	int getCC3Current(float *out_A);

	/* Get CC2 update rate (1.5 or 3 ms depending on config) in seconds */
	float getCC2UpdateRate(void);

	/* Get averaging window (dependent on configuration) in seconds */
	float getCC3Period(void);
	
	/* Get voltage on channel */
	int getVoltage(bq76952_voltages channel, float *o_V);
	
	/* Get all voltages -- faster than reading one by one */
	int getVoltages(BQVoltages_V *out);			
	
	/* Get thermistor value */
	int getThermistorTemp(bq76952_thermistor thermistor, float *o_tempC);

	/* Get all thermistors -- faster than reading one by one */
	int getThermistors(BQTemps_C *out);

	/* Get chip die temp */
	int getDieTemp(float *o_intTempC);

	/* Get primary (FETs) state */
	int getPrimaryState(BQPrimaryState *out);

	/* Get current safety state */
	int getSafetyState(BQSafetyState *ss);

	/* Open both FETs. */
	int requestEnableAll(void);

	/* Shut discharge FET. */
	int requestDisableDischarge(void);

	/* Shut charge FET. */
	int requestDisableCharge(void);

	/* Individuial FET state request */
	int fetRequest(bool chgEnable, bool dsgEnable);

	/* Set message level */
	void setLoudness(bool loudness);

	/* Get accumulated charge */
	int getAccumulatedCharge(float *out_Ah);

	/* Reset charge accumulator */
	int resetAccumulatedCharge(void);
};
