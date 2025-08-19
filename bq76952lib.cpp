/*
* Description :   Source file of BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   Pranjal Joshi, Grisha Revzin @ Flytrex
* Date        :   17/10/2020
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#include "bq76952lib.h"

#include <cfloat>

#include "ftx-esp-common/dbg-macro.h"
#include "ftx-esp-common/crc.h"
#include "ftx-esp-common/util.h"

#define message(M, ...) debug("bq76952: " M, ##__VA_ARGS__)
#define checkbq(A, M, ...) check(A, "bq76952: " M, ##__VA_ARGS__)

// BQ76952 - Address Map
#define CMD_DIR_SUBCMD_LOW            ((byte) 0x3E)
#define CMD_DIR_SUBCMD_HI             ((byte) 0x3F)
#define CMD_DIR_XFER_LEN              ((byte) 0x61)
#define CMD_DIR_XFER_BUF_START        ((byte) 0x40)
#define CMD_DIR_XFER_CHKSUM           ((byte) 0x60)
#define MAX_TRANSFER_SIZE (CMD_DIR_XFER_CHKSUM - CMD_DIR_XFER_BUF_START)

#ifdef DEBUG_PRINTING
static const char *reg2str(int reg);
#endif

typedef enum {
	REG_TYPE_BYTE,
	REG_TYPE_INT16,
	REG_TYPE_INT32,
	REG_TYPE_FLOAT,
} BQRegType;

//// LOW LEVEL FUNCTIONS ////

int bq76952::m_directCommandWrite(byte command, size_t size, int data)
{
  /* Transmit address+w, command address, and data.
   * "size" must match the command in datasheet - that's on the caller.
   * Many commands are read-only - that's also on the caller.
   * return 0 on success, -1 on fail. */
  uint8_t *puData = (uint8_t *) &data;
  int erc = 0;
  checkbq(0 < size && size <= 2, "%s: invalid size %lu", __func__, size);
  m_I2C->beginTransmission(m_I2C_Address);
  checkbq(m_I2C->write(command), "%s: write(%02X) failed", __func__, command);
  checkbq(size == m_I2C->write(puData, size), "%s: write(data=%04X, size=%d) failed", __func__,  data, size);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s: endTransmission unexcepted result %d", __func__, erc);
  if (m_loud) {
    message("%s(command=0x%02X, size=%d, data=0x%08X)", __func__, command, size, data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_directCommandRead(byte command, size_t size, int *o_data)
{
  /* Transmit address+w, command adresss, repeated start, address+r, read data. */
  uint8_t *puData = (uint8_t *) o_data;
  memset(puData, 0, size);
  int erc = 0;
  checkbq(0 < size && size <= 32, "%s: invalid size %lu", __func__, size);
  m_I2C->beginTransmission(m_I2C_Address);
  checkbq(m_I2C->write(command), "%s: write(0x%02X) failed", __func__, command);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s: endTransmission unexcepted result %d", __func__, erc);
  checkbq(size == (erc = m_I2C->requestFrom(m_I2C_Address, size)),
                                          "%s: requestFrom(expected=%d) received %d", __func__, size, erc);
  for (int i = 0; i < size; ++i) {
    puData[i] = m_I2C->read();
  }
  if (m_loud) {
    message("%s(command=0x%02X, size=%d) -> [%08X %08X %08X %08X %08X %08X %08X %08X]",
                                            __func__, command, size,
                                            o_data[0], o_data[1], o_data[2], o_data[3],
                                            o_data[4], o_data[5], o_data[6], o_data[7]);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_writeBulkAddress(int command, size_t size)
{
  byte *puCmd = (byte *) &command;
  checkbq(0 <= size && size <= (CMD_DIR_XFER_CHKSUM - CMD_DIR_XFER_BUF_START), "%s: invalid size %lu", __func__, size);
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write(CMD_DIR_SUBCMD_LOW);
  m_I2C->write(puCmd, 2);
  return 0;

  error:
  return -1;
}

int bq76952::m_subCommandWrite(int command, size_t size, byte *data)
{
  return m_bulkWrite(command, size, data, true);
}

int bq76952::m_pollTransferSetup(int address, unsigned short maxWait)
{
    int erc = 0;
    bool success = false;
    int readback = 0;
    byte *puReadback = (byte *) &readback;
    size_t waitStart = xTaskGetTickCount();
    while (xTaskGetTickCount() - waitStart < maxWait) {
      m_I2C->beginTransmission(m_I2C_Address);
      m_I2C->write((byte) CMD_DIR_SUBCMD_LOW);
      checkbq(!(erc = m_I2C->endTransmission(false)),
              "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
      checkbq(2 == (erc = m_I2C->requestFrom(m_I2C_Address, 2, 1)),
              "%s: requestFrom(expected=%d) received %d", __func__, 2, erc);
      puReadback[0] = m_I2C->read();
      puReadback[1] = m_I2C->read();
      if (readback == address) {
        success = true;
        break;
      }
    }
    checkbq(success, "%s(scmd=0x%04X): failed to verify transfer setup in %d ms", __func__, address, maxWait);
    return 0;

    error:
    return -1;
  }

int bq76952::m_bulkRead(int address, int expectedSize, byte *o_data)
{
  int erc = 0;
  int responseLen = 0;
  byte responseChecksum = 0, ourChecksum = 0;
  byte *puAddress = (byte *) &address;
  checkbq(expectedSize, "%s(scmd=0x%04X, expectedSize=%u): expectedSize cannot be 0", __func__, address, expectedSize);

  /* set r/w adresss on device */
  checkbq(!m_writeBulkAddress(address, expectedSize), "%s(scmd=0x%04X, expectedSize=%u): failed to transmit prologue", __func__, address, expectedSize);
  memset(o_data, 0, expectedSize);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);

  /* wait for transfer setup */
  checkbq(!m_pollTransferSetup(address), "%s(scmd=0x%04X, expectedSize=%u): transfer setup timed out", __func__, address, expectedSize);

  /* read response length */
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write((byte) CMD_DIR_XFER_LEN);
  checkbq(!(erc = m_I2C->endTransmission(false)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
  checkbq(1 == (erc = m_I2C->requestFrom(m_I2C_Address, 1, 0)), "%s: requestFrom(expected=%d) received %d", __func__, expectedSize, erc);
  responseLen = m_I2C->read() - 4;
  checkbq(responseLen == expectedSize, "%s(scmd=0x%04X, expectedSize=%u): responseLen=%u mismatched", __func__, address, expectedSize, responseLen);

  /* read response & calculate our checksum */
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write(CMD_DIR_XFER_BUF_START);
  checkbq(!(erc = m_I2C->endTransmission(false)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
  checkbq(responseLen == (erc = m_I2C->requestFrom(m_I2C_Address, responseLen, 0)), "%s: requestFrom(expected=%d) received %d", __func__, expectedSize, erc);
  for (size_t i = 0; i < responseLen; ++i) {
    o_data[i] = m_I2C->read();
    ourChecksum += o_data[i];
  }
  ourChecksum += (puAddress[0] + puAddress[1]);
  ourChecksum = ~ourChecksum;

  /* reading checksum */
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write(CMD_DIR_XFER_CHKSUM);
  checkbq(!(erc = m_I2C->endTransmission(false)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
  checkbq(1 == (erc = m_I2C->requestFrom(m_I2C_Address, 1, 1)), "%s: requestFrom(expected=%d) received %d", __func__, expectedSize, erc);
  responseChecksum = m_I2C->read();

  /* comparing checksum */
  checkbq(ourChecksum == responseChecksum,
    "%s(scmd=0x%04X, expectedSize=%u): checksum mismatch. Ours = %d theirs = %d", __func__, address, expectedSize, ourChecksum, responseChecksum);

#ifdef DEBUG_PRINTING
  if (m_loud) {
    int *o_data_int = (int *) o_data;
    message("%s(scmd=0x%04X, expectedSize=%d) -> [%08X %08X %08X %08X %08X %08X %08X %08X]",
              __func__, address, expectedSize,
                      o_data_int[0], o_data_int[1], o_data_int[2], o_data_int[3],
                      o_data_int[4], o_data_int[5], o_data_int[6], o_data_int[7]);
  }
#endif

  return 0;

  error:
  return -1;
}

int bq76952::m_subCommandRead(int address, size_t size, byte *o_data)
{
  return m_bulkRead(address, size, o_data);
}

int bq76952::m_memReadI8(int address, byte *o_data)
{
  checkbq(!m_bulkRead(address, 32, m_RAMAccessBuffer), "%s(0x%02X): failed to read", __func__, address);
  *o_data = m_RAMAccessBuffer[0];
  if (m_loud) {
    message("%s(address=0x%04X) -> 0x%02X (%d)", __func__, address, *o_data, *o_data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_memReadI16(int address, short *o_data)
{
  checkbq(!m_bulkRead(address, 32, m_RAMAccessBuffer), "%s(0x%02X): failed to read", __func__, address);
  *o_data = *((short *) m_RAMAccessBuffer);
  if (m_loud) {
    message("%s(address=0x%04X) -> 0x%04X (%d)", __func__, address, *o_data, *o_data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_memReadI32(int address, int *o_data)
{
  checkbq(!m_bulkRead(address, 32, m_RAMAccessBuffer), "%s(0x%02X): failed to read", __func__, address);
  *o_data = *((int32_t *) m_RAMAccessBuffer);
  if (m_loud) {
    message("%s(address=0x%04X) -> 0x%08X (%d)", __func__, address, *o_data, *o_data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_memReadF32(int address, float *o_data)
{
  checkbq(!m_bulkRead(address, 32, m_RAMAccessBuffer), "%s(0x%02X): failed to read", __func__, address);
  *o_data = *((float *) m_RAMAccessBuffer);
  if (m_loud) {
    message("%s(address=0x%04X) -> %f", __func__, address, *o_data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_bulkWrite(int address, int size, byte *data, bool skipTxSetup)
{
  int erc = 0;
  byte checksum = 0;
  byte *puAddress = (byte *) &address;
  /* set r/w adresss on device */
  checkbq(!m_writeBulkAddress(address, size), "%s(scmd=0x%04X, size=%u): failed to transmit prologue", __func__, address, size);
  checkbq(!(erc = m_I2C->endTransmission(true)),
              "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
  if (size == 0) {
    /* nothing else to do here */
    if (m_loud) {
      message("%s(address=0x%04X, size=0) -> [OK]", __func__, address);
    }
    return 0;
  }

  /* wait for transfer setup */
  if (!skipTxSetup) {
    checkbq(!m_pollTransferSetup(address), "%s(scmd=0x%04X, size=%u): transfer setup timed out", __func__, address, size);
    checkbq(!(erc = m_I2C->endTransmission(true)),
                "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
  }

  /* write data to device transfer buffer */
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write(CMD_DIR_XFER_BUF_START);
  for (int i = 0; i < size; ++i) {
    m_I2C->write(data[i]);
    checksum += data[i];
  }
  checksum += (puAddress[0] + puAddress[1]);
  checksum = ~checksum;
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);

  /* write checksum */
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write(CMD_DIR_XFER_CHKSUM);
  m_I2C->write(checksum);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);

  /* write transfer length */
  m_I2C->beginTransmission(m_I2C_Address);
  m_I2C->write(CMD_DIR_XFER_LEN);
  m_I2C->write(size + 4);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s(cmd=0x%04X): endTransmission unexcepted result %d", __func__, address, erc);
#ifdef DEBUG_PRINTING
  if (m_loud) {
    int *data_int = (int *) data;
    message("%s(scmd=0x%04X, size=%d, data = [%08X %08X %08X %08X %08X %08X %08X %08X]) -> [OK]",
            __func__, address,  size, data_int[0], data_int[1], data_int[2], data_int[3],
                                        data_int[4], data_int[5], data_int[6], data_int[7]);
  }
#endif
  return 0;

  error:
  return -1;
}

int bq76952::m_memWriteI8(int address, byte data)
{
  byte readback = 0;
  checkbq(m_inUpdateConfig, "%s(address=0x%04X, data=0x%02X): call enterConfigUpdate first", __func__, address, data);
  m_RAMAccessBuffer[0] = data;
  checkbq(!m_memReadI8(address, &readback), "%s(address=0x%04X, data=0x%02X): readback failed", __func__, address, data);
  checkbq(!m_bulkWrite(address, sizeof(byte), (byte *) &data),
          "%s(address=0x%04X, data=0x%02X): transfer failed", __func__, address, data);
  if (m_loud) {
    message("%s(address=0x%04X, data=0x%02X) -> [OK]", __func__, address, data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_memWriteI16(int address, short data)
{
  short readback = 0;
  checkbq(m_inUpdateConfig, "%s(address=0x%04X, data=0x%04X): call enterConfigUpdate first", __func__, address, data);
  *((short *) m_RAMAccessBuffer) = data;
  checkbq(!m_bulkWrite(address, sizeof(short), (byte *) &data),
          "%s(address=0x%04X, data=0x%04X): transfer failed", __func__, address, data);
  checkbq(!m_memReadI16(address, &readback), "%s(address=0x%04X, data=0x%04X): readback failed", __func__, address, data);
  if (m_loud) {
    message("%s(address=0x%04X, data=0x%04X) -> [OK]", __func__, address, data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_memWriteI32(int address, int data)
{
  int readback = INT_MIN;
  checkbq(m_inUpdateConfig, "%s(address=0x%04X, data=0x%08X): call enterConfigUpdate first", __func__, address, data);
  *((int32_t *) m_RAMAccessBuffer) = data;
  checkbq(!m_bulkWrite(address, sizeof(int32_t), (byte *) &data),
          "%s(address=0x%04X, data=0x%08X): transfer failed", __func__, address, data);
  checkbq(!m_memReadI32(address, &readback), "%s(address=0x%04X, data=0x%08X): readback failed", __func__, address, data);
  checkbq(readback == data, "%s(address=0x%04X, data=0x%08X): failed to verify, got 0x%08X", __func__, address, data, readback);
  if (m_loud) {
    message("%s(address=0x%04X, data=0x%08X) -> [OK]", __func__, address, data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_memWriteF32(int address, float data)
{
  float readback = FLT_MAX;
  checkbq(m_inUpdateConfig, "%s(address=0x%04X, data=%f): call enterConfigUpdate first", __func__, address, data);
  *((int32_t *) m_RAMAccessBuffer) = data;
  checkbq(!m_bulkWrite(address, sizeof(float), (byte *) &data),
          "%s(address=0x%04X, data=%f): transfer failed", __func__, address, data);
  checkbq(!m_memReadF32(address, &readback), "%s(address=0x%04X, data=%f): readback failed", __func__, address, data);
  checkbq(readback == data, "%s(address=0x%04X, data=%f): failed to verify, got %f", __func__, address, data, readback);
  if (m_loud) {
    message("%s(address=0x%04X, data=%f) -> [OK]", __func__, address, data);
  }
  return 0;

  error:
  return -1;
}

int bq76952::m_enterConfigUpdate(void)
{
  if (m_inUpdateConfig) {
    return 0;
  }

  checkbq(!m_subCommandWrite(0x0090), "%s: failed to write subcommand", __func__);
  delay(500);
  m_inUpdateConfig = true;
  if (m_loud) message("entered CONFIG_UPDATE");
  return 0;

  error:
  return -1;
}

int bq76952::m_exitConfigUpdate(void)
{
  if (!m_inUpdateConfig) {
    return 0;
  }

  checkbq(!m_subCommandWrite(0x0092), "%s: failed to write subcommand", __func__);
  delay(500);
  m_inUpdateConfig = false;
  if (m_loud) message("exited CONFIG_UPDATE");
  return 0;

  error:
  return -1;
}

int bq76952::configUpload(const BQConfig *config)
{
  int readbackChecksumMismatched = false;
  int new_checksum = config->CRC32();
  message("%s: config upload started", __func__);
  checkbq(!m_enterConfigUpdate(), "%s: enterConfigUpdate failed", __func__);
  for (size_t i = 0; i < BQ76952_TOT_REGISTERS; ++i) {
    uint16_t address = config->m_registers[i].getAddress();
    int type = config->m_registers[i].getType();
    if (m_loud) {
      message("%s: [%3d/" XSTR(BQ76952_TOT_REGISTERS) "] uploading reg 0x%04X (%s) type %s",
              __func__, i + 1, address, config->m_registers[i].getDescription(), reg2str(type));
    }
    switch (type) {
      case REG_TYPE_BYTE: {
        byte val = config->m_registers[i].getI8();
        checkbq(!m_memWriteI8(address, val),
              "%s: failed to write reg 0x%04X: memWrite_I8 failed", __func__, address);
        break;
      }
      case REG_TYPE_FLOAT: {
        float val = config->m_registers[i].getF32();
        checkbq(!m_memWriteF32(address, val),
              "%s: failed to write reg 0x%04X: memWrite_F32 failed", __func__, address);
        break;
      }
      case REG_TYPE_INT16: {
        unsigned int val = config->m_registers[i].getI16();
        checkbq(!m_memWriteI16(address, val),
              "%s: failed to write reg 0x%04X: memWrite_I16 failed", __func__, address);
        break;
      }
      case REG_TYPE_INT32: {
        int32_t val = config->m_registers[i].getI32();
        checkbq(!m_memWriteI32(address, val),
              "%s: failed to write reg 0x%04X: memWrite_I32 failed", __func__, address);
        break;
      }
      default:
        check_fatal(0, "%s: invalid type %d on register 0x%04X", __func__, type, address);
      }
  }
  checkbq(!m_exitConfigUpdate(), "%s: enterConfigUpdate failed", __func__);
  message("%s: config upload complete, reading back and verifying", __func__);
  checkbq(!m_configDownload(&m_currentConfig), "%s: configDownload failed", __func__);
  m_updateUnits();
  if (m_currentConfig.CRC32() != new_checksum) {
    readbackChecksumMismatched = true;
    checkbq(0,
        "%s: config readback checksum does not match: sent: %08X, got: %08X",
        __func__, m_currentConfig.CRC32(), new_checksum);
  }

  message("%s: config uploaded and verified, checksum %08X", __func__, new_checksum);
  return 0;

  error:
  m_exitConfigUpdate();
  if (!readbackChecksumMismatched) {
    return -1;
  }
  else {
    return -2;
  }
}

int bq76952::m_configDownload(BQConfig *config)
{
  message("%s: config download started", __func__);
  for (size_t i = 0; i < BQ76952_TOT_REGISTERS; ++i) {
    uint16_t address = config->m_registers[i].getAddress();
    int type = config->m_registers[i].getType();
    if (m_loud) {
      message("%s: [%3d/" XSTR(BQ76952_TOT_REGISTERS) "] downloading reg 0x%04X (%s) type %s",
              __func__, i + 1, address, config->m_registers[i].getDescription(), reg2str(type));
    }
    switch (type) {
      case REG_TYPE_BYTE: {
        byte val = 0;
        checkbq(!m_memReadI8(address, &val),
              "%s: failed to read reg 0x%04X: memRead_I8 failed", __func__, address);
        config->m_registers[i].setI8(val);
        break;
      }
      case REG_TYPE_FLOAT: {
        float val = 0;
        checkbq(!m_memReadF32(address, &val),
              "%s: failed to read reg 0x%04X: memRead_F32 failed", __func__, address);
        config->m_registers[i].setF32(val);
        break;
      }
      case REG_TYPE_INT16: {
        short val = 0;
        checkbq(!m_memReadI16(address, &val),
              "%s: failed to read reg 0x%04X: memRead_I16 failed", __func__, address);
        config->m_registers[i].setI16(val);
        break;
      }
      case REG_TYPE_INT32: {
        int val = 0;
        checkbq(!m_memReadI32(address, &val),
              "%s: failed to read reg 0x%04X: memRead_I32 failed", __func__, address);
        config->m_registers[i].setI32(val);
        break;
      }
      default:
        check_fatal(0, "%s: invalid type %d on register 0x%04X", __func__, type, address);
      }
  }
  checkbq(!m_exitConfigUpdate(), "%s: failed to sent exitConfigUpdate", __func__);
  message("%s: config downloaded, crc %08X (default is %08X)",
              __func__, m_currentConfig.CRC32(), m_defaultConfigCRC);
  return 0;

  error:
  m_exitConfigUpdate();
  return -1;
}

/////// API FUNCTIONS ///////

int bq76952::init(TwoWire *i2c, bool loud, byte address)
{
  m_loud = loud;
  m_I2C_Address = address;
  m_I2C = i2c;
  return 0;
}

int bq76952::begin()
{
  checkbq(m_I2C->begin(), "%s: failed to init I2C", __func__);
  m_I2C->setTimeOut(2);
  BQConfig::getDefaultConfig(&m_currentConfig);
  m_defaultConfigCRC = m_currentConfig.CRC32();
  checkbq(!m_configDownload(&m_currentConfig), "%s(address=0x%02X): failed to download config", __func__, m_I2C_Address);
  m_updateUnits();
  return 0;

  error:
  return -1;
}

int bq76952::configChecksum(void) const
{
  return m_currentConfig.CRC32();
}

int bq76952::reset(void)
{
  checkbq(!m_subCommandWrite(0x0012), "%s: subCommandWrite failed", __func__);
  vTaskDelay(1000);
  checkbq(!m_configDownload(&m_currentConfig), "%s: failed to download config", __func__);
  m_updateUnits();
  return 0;

  error:
  return -1;
}

int bq76952::getVoltage(bq76952_voltages channel, float *o_V)
{
  checkbq(BQ_VOLTAGE_CELL1 <= channel && channel <= BQ_VOLTAGE_LD && channel % 2 == 0,
        "%s: invalid channel %d", __func__, channel);
  uint16_t raw;
  checkbq(!m_directCommandRead((uint8_t) channel, sizeof(uint16_t), (int *) &raw),
        "%s: directCommandRead failed", __func__);
  if (channel <= BQ_VOLTAGE_CELL16) {
    return raw / 1e3;
  }
  else {
    return raw * m_userV_volts;
  }
  return 0;

  error:
  return -1;
}

struct raw_voltages {
	int16_t cells[16];
	int16_t stack, pack, ld;
} __attribute__((packed));

int bq76952::getVoltages(BQVoltages_V *out)
{
  raw_voltages raw = {0};
  uint8_t *puSecondRead = ((uint8_t *) &raw) + 32;
  size_t remainder = sizeof(raw_voltages) % 32;
  checkbq(!m_directCommandRead(BQ_VOLTAGE_CELL1, 32, (int *) &raw),
        "%s: directCommandRead failed", __func__);
  checkbq(!m_directCommandRead(BQ_VOLTAGE_CELL1 + 32, remainder, (int *) puSecondRead),
        "%s: directCommandRead failed", __func__);
  for (size_t i = 0; i < BQ_N_CELLS; ++i) {
    out->cells[i] = raw.cells[i] / 1e3;
  }
  out->STACK = raw.stack * m_userV_volts;
  out->PACK = raw.pack * m_userV_volts;
  out->LD = raw.ld * m_userV_volts;
  return 0;

  error:
  return -1;
}

int bq76952::getCC2Current(float *current_amps)
{
  int16_t val = 0;
  checkbq(!m_directCommandRead(0x3A, 2, (int *) &val),  "%s: directCommandRead failed", __func__);
  *current_amps = m_userA_amps * val;
  return 0;

  error:
  return -1;
}

struct temperatures_raw {
	uint16_t int_;
  uint16_t cfetoff;
  uint16_t dfetoff;
  uint16_t alert;
  uint16_t ts1;
  uint16_t ts2;
  uint16_t ts3;
  uint16_t hdq;
  uint16_t dchg;
  uint16_t ddsg;
} __attribute__((packed));

static float convert_temp(uint16_t raw)
{
  return raw / 10.0 - 273.15;
}

int bq76952::getThermistors(BQTemps_C *out)
{
  temperatures_raw raw = {0};
  checkbq(!m_directCommandRead(BQ_THERMISTOR_INT, sizeof(temperatures_raw), (int *) &raw),
        "%s: directCommandRead failed", __func__);
  out->int_ = convert_temp(raw.int_);
  out->cfetoff = convert_temp(raw.cfetoff);
  out->dfetoff = convert_temp(raw.dfetoff);
  out->alert = convert_temp(raw.alert);
  out->ts1 = convert_temp(raw.ts1);
  out->ts2 = convert_temp(raw.ts2);
  out->ts3 = convert_temp(raw.ts3);
  out->hdq = convert_temp(raw.hdq);
  out->dchg = convert_temp(raw.dchg);
  out->ddsg = convert_temp(raw.ddsg);
  return 0;

  error:
  return -1;
}

int bq76952::getDieTemp(float *o_intTempC)
{
  int raw = 0;
  checkbq(!m_directCommandRead(0x68, sizeof(uint16_t), (int *) raw), "%s: directCommandRead failed", __func__);
  *o_intTempC = convert_temp(raw);
  return 0;

  error:
  return -1;
}

int bq76952::getThermistorTemp(bq76952_thermistor thermistor, float *o_tempC)
{
  int raw = 0;
  checkbq(BQ_THERMISTOR_TS1 <= thermistor && thermistor <= BQ_THERMISTOR_DDSG,
        "%s: invalid thermistor channel %d", __func__, thermistor);
  checkbq(!m_directCommandRead((byte) thermistor, sizeof(uint16_t), (int *) raw),
        "%s: directCommandRead failed", __func__);
  *o_tempC = convert_temp(raw);
  return 0;

  error:
  return -1;
}

void bq76952::m_updateUnits(void)
{
  m_userA_amps = m_currentConfig.getUserAScaling();
  m_userV_volts = m_currentConfig.getUserVScaling();
}

int bq76952::getPrimaryState(BQPrimaryState *out)
{
  byte reg = 0;
  checkbq(!m_directCommandRead(0x7F, 1, (int *) &reg), "%s: m_directCommandRead failed", __func__);
  out->CFETclosed = reg & (1 << 0);
  out->DFETclosed = reg & (1 << 2);
  out->PCFETclosed = reg & (1 << 1);
  out->PDFETclosed = reg & (1 << 3);
  out->alertPin = reg & (1 << 6);
  return 0;

  error:
  return -1;
}

int bq76952::getSafetyState(BQSafetyState *ss)
{
  checkbq(!m_directCommandRead(0x02, 6, (int *) (ss->m_registerBlock)), "%s: directCommandRead failed", __func__);
  checkbq(!m_directCommandRead(0x0A, 8, (int *) (ss->m_registerBlock + 6)), "%s: directCommandRead failed", __func__);

  return 0;

  error:
  return -1;
}

void bq76952::setLoudness(bool loud)
{
  m_loud = loud;
}

int bq76952::requestEnableAll(void)
{
  return m_subCommandWrite(0x0096);
}

int bq76952::requestDisableDischarge(void)
{
  return m_subCommandWrite(0x0093);
}

int bq76952::requestDisableCharge(void)
{
  return m_subCommandWrite(0x0094);
}

struct DASTATUS6 {
  int32_t intPortion;
  uint32_t fracPortion;
  uint32_t accumulatedTime;
  uint32_t CFETOFF_counts;
  uint32_t DFETOFF_counts;
  uint32_t ALERT_counts;
  uint32_t TS1_counts;
  uint32_t TS2_counts;
} __attribute__((packed));

int bq76952::getAccumulatedCharge(float *out)
{
  DASTATUS6 buffer;
  checkbq(!m_subCommandRead(0x0076, sizeof(buffer), (byte *) &buffer), "%s: m_subCommandRead failed", __func__);
  *out = ((float) buffer.fracPortion / ((float) UINT32_MAX) - 0.5 + (float) buffer.intPortion) * m_userA_amps;
  return 0;

  error:
  return -1;
}

int bq76952::resetAccumulatedCharge(void)
{
  return m_subCommandWrite(0x0082);
}

float bq76952::getCC2UpdateRate(void)
{
   /* Settings.Configuration.Power Config[FASTADC] */
    for (size_t i = 0; i < BQ76952_TOT_REGISTERS; ++i) {
    if (0x9234 == m_currentConfig.m_registers[i].getAddress()) {
      if ((1 << 6) & m_currentConfig.m_registers[i].getI16()) {
        return 1.5e-3;
      }
      else {
        return 3e-3;
      }
    }
  }
  check_fatal(0, "%s: this is not a valid BQ configuration", __func__);
}

struct DASTATUS5 {
  int16_t VREG18;
  int16_t VSS;
  int16_t maxCellVoltage;
  int16_t minCellVoltage;
  int16_t batteryVoltageSum;
  int16_t cellTemperature;
  int16_t fetTemperature;
  int16_t maxCellTemperature;
  int16_t minCellTemperature;
  int16_t avgCellTemperature;
  int16_t cc3;
  int16_t cc1;
  int32_t cc2Counts;
  int32_t cc3Counts;
} __attribute__((packed));

int bq76952::getCC3Current(float *outA, int *outADC)
{
  DASTATUS5 buffer;
  checkbq(!m_subCommandRead(0x0075, sizeof(buffer), (byte *) &buffer), "%s: m_subCommandRead failed", __func__);
  *outA = buffer.cc3 * m_userA_amps;
  if (outADC) {
    *outADC = buffer.cc3Counts;
  }
  return 0;

  error:
  return -1;
}

float bq76952::getCC3Period(void)
{
   /* Settings.Configuration.CC3 Samples */
    for (size_t i = 0; i < BQ76952_TOT_REGISTERS; ++i) {
    if (0x9307 == m_currentConfig.m_registers[i].getAddress()) {
      byte nSamples = (byte) m_currentConfig.m_registers[i].getI8();
      return nSamples * getCC2UpdateRate();
    }
  }
  check_fatal(0, "%s: this is not a valid BQ configuration", __func__);
}

int bq76952::fetRequest(bool chgEnable, bool dsgEnable)
{
  uint8_t buf = 0;
  if (dsgEnable) {
    buf |= (1 << 0);  // DSG_FET
    buf |= (1 << 1);  // PDSG_FET
  }
  if (chgEnable) {
    buf |= (1 << 2);  // CHG_FET
    buf |= (1 << 3);  // PCGH_FET
  }
  /* 12.5.6 FET Control: low if FET allowed to turn on */
  buf = ~buf;
  check(!m_subCommandWrite(0x0097, 1, &buf), "%s: m_subCommandWrite failed", __func__);
  return 0;

  error:
  return -1;
}

struct ReadCal1_raw {
  int16_t cdc;
  int32_t cc2;
  uint16_t packPin;
  uint16_t tos;
  uint16_t ldPin;
  int16_t dummy; /* For some reason the response length is 14, while the datasheets specify 12. */
} __attribute__((packed));

int bq76952::getVCalibADCCounts(BQRawCalibCounts *out)
{
  ReadCal1_raw raw;
  check(!m_subCommandRead(0xf081, sizeof(ReadCal1_raw), (byte *) &raw), "%s: m_subCommandRead failed", __func__);
  out->ldPin = raw.ldPin;
  out->packPin = raw.packPin;
  out->topOfStack = raw.tos;
  out->cc2counts = ((raw.cc2 & 0x00FFFF00) >> 8) - 0xFFFF;
  /* The datasheet off-handedly mentions using only the second and third bytes for calibration */
  return 0;

  error:
  return -1;
}

/* Sleep mode allow/deny */
int bq76952::sleepMode(bool allowed)
{
  int subcommand = allowed ? 0x0099 : 0x009A;
  check(!m_subCommandWrite(subcommand), "%s: m_subCommandWrite failed", __func__);
  return 0;

  error:
  return -1;
}

/* Deepsleep mode enter/exit */
int bq76952::deepSleep(bool enter)
{
  if (enter) {
    /* Must be sent two times as per the datasheet */
    check(!m_subCommandWrite(0x000F), "%s: m_subCommandWrite failed", __func__);
    check(!m_subCommandWrite(0x000F), "%s: m_subCommandWrite failed", __func__);
    vTaskDelay(500);
    uint16_t csr = 0;
    check(!m_directCommandRead(0x00, 2, (int *) &csr), "%s: m_directCommandRead failed", __func__);
    check(csr & (1 << 2), "%s: device failed to enter DEEPSLEEP mode", __func__);
  }
  else {
    check(!m_subCommandWrite(0x000E), "%s: m_subCommandWrite failed", __func__);
    vTaskDelay(500);
    uint16_t csr = 0;
    check(!m_directCommandRead(0x00, 2, (int *) &csr), "%s: m_directCommandRead failed", __func__);
    check(!(csr & (1 << 0)), "%s: device failed to exit DEEPSLEEP mode", __func__);
  }
  info("%s: BQ76952 DEEPSLEEP mode change successful: %d", __func__, enter);
  return 0;

  error:
  return -1;
}

int bq76952::getCellBalancingMask(int *o_mask)
{
  uint16_t bmask = 0;
  check(!m_subCommandRead(0x0083, 2, (uint8_t *) &bmask), "%s: m_subCommandRead failed", __func__);
  *o_mask = bmask;
  return 0;

  error:
  return -1;
}

int bq76952::getAlarmStatus(int *o_alarmStatus)
{
  *o_alarmStatus = 0;
  check(!m_directCommandRead(0x64, 2, o_alarmStatus), "%s: m_directCommandRead failed", __func__);
  return 0;

  error:
  return -1;
}

int bq76952::clearAlarmStatus(void)
{
  return m_directCommandWrite(0x62, 2, 0xFFFF);
}

#define SAFETY_REG(x) (x >> 8)
#define SAFETY_BIT(x) (x & 0xF)

bool BQSafetyState::getSafetyFlag(BQSafety flag) const
{
  return m_readSafetyVal(SAFETY_REG(flag), SAFETY_BIT(flag)) > 0;
}

#define SF_BLOCK0_START SAFETY_REG(BQSafety_A_SCD)
#define SF_BLOCK1_START SAFETY_REG(BQSafety_A_CUDEP)

bool BQSafetyState::m_readSafetyVal(int reg, int bit) const
{
  check_fatal(SAFETY_REG(BQSafety_A_SCD) <= reg && reg <= SAFETY_REG(BQSafety_F_TOSF),
            "%s: register 0x%02X out of bounds [0x%02X, 0x%02X]",
             __func__, reg, SAFETY_REG(BQSafety_A_SCD), SAFETY_REG(BQSafety_F_TOSF));
  check_fatal(0 <= bit && bit < 8, "%s: bit number %d out of bounds", __func__, bit);
  size_t index = 0;
  if (reg >= SF_BLOCK1_START) {
    index = 6 + reg - SF_BLOCK1_START;
  }
  else {
    index = reg - SF_BLOCK0_START;
  }
  return (m_registerBlock[index] & (1 << bit));
}

 BQSafety const BQ_ALERT_ITERABLE[BQ_N_ALERTS] = {
	BQSafety_A_SCD, BQSafety_A_OCD2, BQSafety_A_OCD1, BQSafety_A_OCC, BQSafety_A_COV, BQSafety_A_CUV,
	BQSafety_A_OTF, BQSafety_A_OTINT, BQSafety_A_OTD, BQSafety_A_OTC, BQSafety_A_UTINT, BQSafety_A_UTD, BQSafety_A_UTC,
	BQSafety_A_OCD3, BQSafety_A_SCDL, BQSafety_A_OCDL, BQSafety_A_COVL, BQSafety_A_PTOS,
	BQSafety_A_CUDEP, BQSafety_A_SOTF, BQSafety_A_SOT, BQSafety_A_SOCD, BQSafety_A_SOCC, BQSafety_A_SOV, BQSafety_A_SUV,
	BQSafety_A_SCDL_PF, BQSafety_A_VIMA, BQSafety_A_VIMR, BQSafety_A_2LVL, BQSafety_A_DFETF, BQSafety_A_CFETF,
	BQSafety_A_HWMX, BQSafety_A_VSSF, BQSafety_A_VREF, BQSafety_A_LFOF,
	BQSafety_A_TOSF
};

 BQSafety const BQ_FAULT_ITERABLE[BQ_N_ALERTS] = {
	BQSafety_F_SCD, BQSafety_F_OCD2, BQSafety_F_OCD1, BQSafety_F_OCC, BQSafety_F_COV, BQSafety_F_CUV,
	BQSafety_F_OTF, BQSafety_F_OTINT, BQSafety_F_OTD, BQSafety_F_OTC, BQSafety_F_UTINT, BQSafety_F_UTD, BQSafety_F_UTC,
	BQSafety_F_OCD3, BQSafety_F_SCDL, BQSafety_F_OCDL, BQSafety_F_COVL, BQSafety_F_PTOS,
	BQSafety_F_CUDEP, BQSafety_F_SOTF, BQSafety_F_SOT, BQSafety_F_SOCD, BQSafety_F_SOCC, BQSafety_F_SOV, BQSafety_F_SUV,
	BQSafety_F_SCDL_PF, BQSafety_F_VIMA, BQSafety_F_VIMR, BQSafety_F_2LVL, BQSafety_F_DFETF, BQSafety_F_CFETF,
	BQSafety_F_HWMX, BQSafety_F_VSSF, BQSafety_F_VREF, BQSafety_F_LFOF,
	BQSafety_F_TOSF
};

const char *BQSafetyState::safetyFlagToString(BQSafety flag)
{
  switch (flag) {
  case BQSafety_A_SCD:  case BQSafety_F_SCD:    return "SCD";   case BQSafety_A_OCD2:   case BQSafety_F_OCD2:   return "OCD2";
  case BQSafety_A_OCD1: case BQSafety_F_OCD1:   return "OCD1";  case BQSafety_A_OCC:    case BQSafety_F_OCC:    return "OCC";
  case BQSafety_A_COV:  case BQSafety_F_COV:    return "COV";   case BQSafety_A_CUV:    case BQSafety_F_CUV:    return "CUV";
  case BQSafety_A_OTF:  case BQSafety_F_OTF:    return "OTF";   case BQSafety_A_OTINT:  case BQSafety_F_OTINT:  return "OTINT";
  case BQSafety_A_OTD:  case BQSafety_F_OTD:    return "OTD";   case BQSafety_A_OTC:    case BQSafety_F_OTC:    return "OTC";
  case BQSafety_A_UTINT: case BQSafety_F_UTINT: return "UTINT"; case BQSafety_A_UTD:    case BQSafety_F_UTD:    return "UTD";
  case BQSafety_A_UTC:  case BQSafety_F_UTC:    return "UTC";   case BQSafety_A_OCD3:   case BQSafety_F_OCD3:   return "OCD3";
  case BQSafety_A_SCDL: case BQSafety_F_SCDL:   return "SCDL";  case BQSafety_A_OCDL:   case BQSafety_F_OCDL:   return "OCDL";
  case BQSafety_A_COVL: case BQSafety_F_COVL:   return "COVL";  case BQSafety_A_PTOS:   case BQSafety_F_PTOS:   return "PTOS";
  case BQSafety_A_CUDEP: case BQSafety_F_CUDEP: return "CUDEP"; case BQSafety_A_SOTF:   case BQSafety_F_SOTF:   return "SOTF";
  case BQSafety_A_SOT:  case BQSafety_F_SOT:    return "SOT";   case BQSafety_A_SOCD:   case BQSafety_F_SOCD:   return "SOCD";
  case BQSafety_A_SOCC: case BQSafety_F_SOCC:   return "SOCC";  case BQSafety_A_SOV:    case BQSafety_F_SOV:    return "SOV";
  case BQSafety_A_SUV: case BQSafety_F_SUV:     return "SUV";   case BQSafety_A_SCDL_PF: case BQSafety_F_SCDL_PF: return "PF";
  case BQSafety_A_VIMA: case BQSafety_F_VIMA:   return "VIMA";  case BQSafety_A_VIMR:   case BQSafety_F_VIMR:    return "VIMR";
  case BQSafety_A_2LVL: case BQSafety_F_2LVL:   return "2LVL";  case BQSafety_A_DFETF:  case BQSafety_F_DFETF:   return "DFETF";
  case BQSafety_A_CFETF: case BQSafety_F_CFETF: return "CFETF"; case BQSafety_A_HWMX:   case BQSafety_F_HWMX:    return "HWMX";
  case BQSafety_A_VSSF: case BQSafety_F_VSSF:   return "VSSF";  case BQSafety_A_VREF:   case BQSafety_F_VREF:    return "VREF";
  case BQSafety_A_LFOF: case BQSafety_F_LFOF:   return "LFOF";  case BQSafety_A_TOSF:   case BQSafety_F_TOSF:    return "TOSF";
  default: return "INVALID";
  }
}

int BQSafetyState::snprintLockouts(char *buffer, size_t bufSize)
{
    char *pLast = buffer;
    size_t nActive = 0;
    for (size_t i = 0; i < BQ_N_ALERTS; ++i) {
        BQSafety alert = BQ_FAULT_ITERABLE[i];
        bool active = getSafetyFlag(alert);
        if (active) {
            pLast += snprintf(pLast, bufSize - (pLast - buffer), "%s ", safetyFlagToString(alert));
            nActive++;
        }
    }
    return nActive;
}

int BQConfig::CRC32(void) const
{
  uint32_t checksum = UINT32_MAX;
  for (size_t i = 0; i < BQ76952_TOT_REGISTERS; ++i) {
    checksum = ftx_util::CRC32(m_registers[i].m_value, m_registers[i].getSize(), checksum);
  }
  return checksum;
}

void BQConfig::setRegister(size_t i, const BQRegister &reg)
{
  check_fatal(i < BQ76952_TOT_REGISTERS,
              "%s: invalid access index %u (max" STR(BQ76952_TOT_REGISTERS) ")", __func__, i);
  m_registers[i] = reg;
}

void BQRegister::setI8(byte val)
{
  m_type = REG_TYPE_BYTE;
  m_value[0] = val;
}

void BQRegister::setF32(float val)
{
  m_type = REG_TYPE_FLOAT;
  *((float *) m_value) = val;
}

void BQRegister::setI16(short val)
{
  m_type = REG_TYPE_INT16;
  *((short *) m_value) = val;
}

void BQRegister::setI32(int32_t val)
{
  m_type = REG_TYPE_INT32;
  *((int32_t *) m_value) = val;
}

byte BQRegister::getI8(void) const
{
  return m_value[0];
}

float BQRegister::getF32(void) const
{
  return *((float *) m_value);
}

short BQRegister::getI16(void) const
{
  return *((short *) m_value);
}

int32_t BQRegister::getI32(void) const
{
  return *((int32_t *) m_value);
}

BQRegister::BQRegister(void)
{
  m_descr = nullptr;
}

BQRegister::BQRegister(int address, byte val, const char *descr)
{
  m_descr = descr;
  m_address = (short) address % 0xFFFF;
  setI8(val);
}

BQRegister::BQRegister(int address, float val, const char *descr)
{
  m_descr = descr;
  m_address = (short) address % 0xFFFF;
  setF32(val);
}

BQRegister::BQRegister(int address, uint16_t val, const char *descr)
{
  m_descr = descr;
  m_address = (short) address % 0xFFFF;
  setI16(val);
}

BQRegister::BQRegister(int address, uint32_t val, const char *descr)
{
  m_descr = descr;
  m_address = (short) address % 0xFFFF;
  setI32(val);
}

size_t BQRegister::getSize(void) const
{
  switch (m_type) {
    default:
    case REG_TYPE_FLOAT:
    case REG_TYPE_INT32:
      return 4;
    case REG_TYPE_INT16:
      return 2;
    case REG_TYPE_BYTE:
      return 1;
  }
}

#ifdef DEBUG_PRINTING
static const char *reg2str(int reg)
{
  switch (reg) {
  case REG_TYPE_FLOAT:
    return "FLOAT32";
  case REG_TYPE_BYTE:
    return "INT8";
  case REG_TYPE_INT16:
    return "INT16";
  case REG_TYPE_INT32:
    return "INT32";
  default:
    return "INVALID";
  }
}
#endif

int BQRegister::getType(void) const
{
  return m_type;
}

int BQRegister::getAddress(void) const
{
  return m_address;
}

const char *BQRegister::getDescription(void) const
{
  if (m_descr)
    return m_descr;
  else
    return "Empty";
}

BQRegister *BQConfig::m_regByAddress(int address) const
{
  for (size_t i = 0; i < BQ76952_TOT_REGISTERS; ++i) {
    if (address == m_registers[i].m_address) {
      return (BQRegister *) &m_registers[i];
    }
  }
  return NULL;
}

float BQConfig::getUserAScaling(void) const
{
  int config;
  BQRegister *reg = m_regByAddress(0x9303);
  check_fatal(reg, "%s: this is not a valid BQ configuration", __func__);
  config = 0x3 & reg->m_value[0];
  switch (config) {
  default:
  case 0:
    return 1e-4;
  case 1:
    return 1e-3;
  case 2:
    return 1e-2;
  case 3:
    return 1e-1;
  }
}

float BQConfig::getUserVScaling(void) const
{
  int config;
  BQRegister *reg = m_regByAddress(0x9303);
  check_fatal(reg, "%s: this is not a valid BQ configuration", __func__);
  config = (1 << 2) & reg->m_value[0];
  if (config) {
    return 1e-2;
  }
  else {
    return 1e-3;
  }
}

const static float C_CCGAIN_NUMERATOR = 7.4768;         /* 13.2.2.1 Calibration:Current:CC Gain */
const static float C_CAPGAIN_MULTIPLIER = 298261.6178;  /* 13.2.2.2 Calibration:Current:Capacity Gain */

float BQconfigAdjustments::getSenseResistorFromGain(float gain)
{
  return C_CCGAIN_NUMERATOR / gain;
}

void BQConfig::applyAdjustments(const BQconfigAdjustments &adj)
{
  /* first 16 registers are luckily exactly the cell gain */
  for (size_t i = 0; i < BQ_N_CELLS; ++i) {
    m_registers[i].setI16(adj.cellGain[i]);
  }
  m_registers[16].setI16(adj.packGain);
  m_registers[17].setI16(adj.tosGain);
  m_registers[18].setI16(adj.ldGain);
  m_registers[25].setI16(adj.currentOffset);

  /* 13.2.2.1 Calibration:Current:CC Gain */
  float ccGainValue = ftx_util::clamp(1e-2f, adj.currentSenseGain, 10e2f);
  m_registers[20].setF32(ccGainValue);
  /* 13.2.2.2 Calibration:Current:Capacity Gain */
  m_registers[21].setF32(C_CAPGAIN_MULTIPLIER * ccGainValue);
  /* 13.3.2.19 Settings:Configuration:Vcell Mode */
  m_registers[94].setI16(adj.activeCellsMask);
  /* 13.6.2.1 Protections:COV:Threshold */
  m_registers[176].setI8(ceil(adj.COVThresholdV * (1000.0f / 50.6f)));
  /* 13.7.4.1 Permanent Fail:TOS:Threshold */
  m_registers[239].setI16(adj.tosfThresholdV * 1000 / BQ_N_CELLS);
  /* 13.6.13.1 Protections:OTD:Threshold */
  m_registers[207].setI8(floor(adj.emergencyHighTempC));
  /* 13.6.12.1 Protections:OTC:Threshold */
  m_registers[204].setI8(floor(adj.chargingLimitHighTempC));
  /* 13.6.17.1 Protections:UTD:Threshold */
  m_registers[219].setI8(ceil(adj.emergencyLowTempC));
  /* 13.6.16.1 Protections:UTC:Threshold */
  m_registers[216].setI8(ceil(adj.chargingLimitLowTempC));

  /* 13.3.11.10 Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax) */
  m_registers[151].setI16(ceil(adj.minAutoBalancingVoltageV * 1000.0f));

  /* 13.3.11.7 Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge) */
  m_registers[154].setI16(ceil(adj.minAutoBalancingVoltageV * 1000.0f));
}

void BQConfig::readAdjustmetns(BQconfigAdjustments &adj)
{
  /* first 16 registers are luckily exactly the cell gains */
  for (size_t i = 0; i < BQ_N_CELLS; ++i) {
    adj.cellGain[i] = m_registers[i].getI16();
  }
  adj.packGain = m_registers[16].getI16();
  adj.tosGain = m_registers[17].getI16();
  adj.ldGain = m_registers[18].getI16();
  adj.currentOffset = m_registers[25].getI16();
  adj.currentSenseGain = m_registers[20].getF32();
}

float BQConfig::getMaxChargeTemp(void) const
{
  return 1.0f * m_regByAddress(0x929A)->getI8();
}

void BQConfig::getDefaultConfig(BQConfig *out)
{
  out->setRegister(  0,	BQRegister(0x9180, (uint16_t) 	 0x00000000));
	out->setRegister(  1,	BQRegister(0x9182, (uint16_t) 	 0x00000000));
	out->setRegister(  2,	BQRegister(0x9184, (uint16_t) 	 0x00000000));
	out->setRegister(  3,	BQRegister(0x9186, (uint16_t) 	 0x00000000));
	out->setRegister(  4,	BQRegister(0x9188, (uint16_t) 	 0x00000000));
	out->setRegister(  5,	BQRegister(0x918A, (uint16_t) 	 0x00000000));
	out->setRegister(  6,	BQRegister(0x918C, (uint16_t) 	 0x00000000));
	out->setRegister(  7,	BQRegister(0x918E, (uint16_t) 	 0x00000000));
	out->setRegister(  8,	BQRegister(0x9190, (uint16_t) 	 0x00000000));
	out->setRegister(  9,	BQRegister(0x9192, (uint16_t) 	 0x00000000));
	out->setRegister( 10,	BQRegister(0x9194, (uint16_t) 	 0x00000000));
	out->setRegister( 11,	BQRegister(0x9196, (uint16_t) 	 0x00000000));
	out->setRegister( 12,	BQRegister(0x9198, (uint16_t) 	 0x00000000));
	out->setRegister( 13,	BQRegister(0x919A, (uint16_t) 	 0x00000000));
	out->setRegister( 14,	BQRegister(0x919C, (uint16_t) 	 0x00000000));
	out->setRegister( 15,	BQRegister(0x919E, (uint16_t) 	 0x00000000));
	out->setRegister( 16,	BQRegister(0x91A0, (uint16_t) 	 0x00000000));
	out->setRegister( 17,	BQRegister(0x91A2, (uint16_t) 	 0x00000000));
	out->setRegister( 18,	BQRegister(0x91A4, (uint16_t) 	 0x00000000));
	out->setRegister( 19,	BQRegister(0x91A6, (uint16_t) 	 0x00000000));
	out->setRegister( 20,	BQRegister(0x91A8, (float) 		 7.476800));
	out->setRegister( 21,	BQRegister(0x91AC, (float) 		 2230042.463000));
	out->setRegister( 22,	BQRegister(0x91B0, (uint16_t) 	 0x00000000));
	out->setRegister( 23,	BQRegister(0x91B2, (uint16_t) 	 0x00000000));
	out->setRegister( 24,	BQRegister(0x91C6, (uint16_t) 	 0x00000040));
	out->setRegister( 25,	BQRegister(0x91C8, (uint16_t) 	 0x00000000));
	out->setRegister( 26,	BQRegister(0x91CA, (uint8_t) 	 0x00000000));
	out->setRegister( 27,	BQRegister(0x91CB, (uint8_t) 	 0x00000000));
	out->setRegister( 28,	BQRegister(0x91CC, (uint8_t) 	 0x00000000));
	out->setRegister( 29,	BQRegister(0x91CD, (uint8_t) 	 0x00000000));
	out->setRegister( 30,	BQRegister(0x91CE, (uint8_t) 	 0x00000000));
	out->setRegister( 31,	BQRegister(0x91CF, (uint8_t) 	 0x00000000));
	out->setRegister( 32,	BQRegister(0x91D0, (uint8_t) 	 0x00000000));
	out->setRegister( 33,	BQRegister(0x91D1, (uint8_t) 	 0x00000000));
	out->setRegister( 34,	BQRegister(0x91D2, (uint8_t) 	 0x00000000));
	out->setRegister( 35,	BQRegister(0x91D3, (uint8_t) 	 0x00000000));
	out->setRegister( 36,	BQRegister(0x91E2, (uint16_t) 	 0x0000632E));
	out->setRegister( 37,	BQRegister(0x91E4, (uint16_t) 	 0x00000BD8));
	out->setRegister( 38,	BQRegister(0x91E6, (uint16_t) 	 0x00003FFF));
	out->setRegister( 39,	BQRegister(0x91E8, (uint16_t) 	 0x000018EB));
	out->setRegister( 40,	BQRegister(0x91EA, (uint16_t) 	 0xFFFFC35C));
	out->setRegister( 41,	BQRegister(0x91EC, (uint16_t) 	 0x00006737));
	out->setRegister( 42,	BQRegister(0x91EE, (uint16_t) 	 0xFFFFA778));
	out->setRegister( 43,	BQRegister(0x91F0, (uint16_t) 	 0x000070A2));
	out->setRegister( 44,	BQRegister(0x91F2, (uint16_t) 	 0x000002A0));
	out->setRegister( 45,	BQRegister(0x91F4, (uint16_t) 	 0xFFFFFE8D));
	out->setRegister( 46,	BQRegister(0x91F6, (uint16_t) 	 0x000002C4));
	out->setRegister( 47,	BQRegister(0x91F8, (uint16_t) 	 0xFFFFF256));
	out->setRegister( 48,	BQRegister(0x91FA, (uint16_t) 	 0x000013BB));
	out->setRegister( 49,	BQRegister(0x91FE, (uint16_t) 	 0x00002DB7));
	out->setRegister( 50,	BQRegister(0x9200, (uint16_t) 	 0xFFFFBB97));
	out->setRegister( 51,	BQRegister(0x9202, (uint16_t) 	 0x0000649F));
	out->setRegister( 52,	BQRegister(0x9204, (uint16_t) 	 0xFFFFA3D7));
	out->setRegister( 53,	BQRegister(0x9206, (uint16_t) 	 0x00007DAF));
	out->setRegister( 54,	BQRegister(0x9208, (uint16_t) 	 0x0000082A));
	out->setRegister( 55,	BQRegister(0x920A, (uint16_t) 	 0xFFFFF7F9));
	out->setRegister( 56,	BQRegister(0x920C, (uint16_t) 	 0x00000B8B));
	out->setRegister( 57,	BQRegister(0x920E, (uint16_t) 	 0xFFFFF29D));
	out->setRegister( 58,	BQRegister(0x9210, (uint16_t) 	 0x00001121));
	out->setRegister( 59,	BQRegister(0x9214, (uint16_t) 	 0x0000435E));
	out->setRegister( 60,	BQRegister(0x9216, (uint16_t) 	 0x00000000));
	out->setRegister( 61,	BQRegister(0x9218, (uint16_t) 	 0x00000000));
	out->setRegister( 62,	BQRegister(0x921A, (uint16_t) 	 0x00000000));
	out->setRegister( 63,	BQRegister(0x921C, (uint16_t) 	 0x00000000));
	out->setRegister( 64,	BQRegister(0x921E, (uint16_t) 	 0x00000000));
	out->setRegister( 65,	BQRegister(0x9220, (uint16_t) 	 0x00000000));
	out->setRegister( 66,	BQRegister(0x9222, (uint16_t) 	 0x00000000));
	out->setRegister( 67,	BQRegister(0x9224, (uint16_t) 	 0x00000000));
	out->setRegister( 68,	BQRegister(0x9226, (uint16_t) 	 0x00000000));
	out->setRegister( 69,	BQRegister(0x9228, (uint16_t) 	 0x00000000));
	out->setRegister( 70,	BQRegister(0x922A, (uint16_t) 	 0x00000000));
	out->setRegister( 71,	BQRegister(0x922D, (uint8_t) 	 0x00000009));
	out->setRegister( 72,	BQRegister(0x91D4, (uint16_t) 	 0x0000FFFF));
	out->setRegister( 73,	BQRegister(0x91D6, (uint16_t) 	 0x0000FFFF));
	out->setRegister( 74,	BQRegister(0x9231, (uint16_t) 	 0x000001F4));
	out->setRegister( 75,	BQRegister(0x9233, (uint8_t) 	 0x0000001E));
	out->setRegister( 76,	BQRegister(0x9234, (uint16_t) 	 0x00002982));
	out->setRegister( 77,	BQRegister(0x9236, (uint8_t) 	 0x00000000));
	out->setRegister( 78,	BQRegister(0x9237, (uint8_t) 	 0x00000000));
	out->setRegister( 79,	BQRegister(0x9238, (uint8_t) 	 0x00000000));
	out->setRegister( 80,	BQRegister(0x9239, (uint8_t) 	 0x00000000));
	out->setRegister( 81,	BQRegister(0x923A, (uint8_t) 	 0x00000000));
	out->setRegister( 82,	BQRegister(0x923C, (uint8_t) 	 0x00000020));
	out->setRegister( 83,	BQRegister(0x923D, (uint8_t) 	 0x00000000));
	out->setRegister( 84,	BQRegister(0x92FA, (uint8_t) 	 0x00000000));
	out->setRegister( 85,	BQRegister(0x92FB, (uint8_t) 	 0x00000000));
	out->setRegister( 86,	BQRegister(0x92FC, (uint8_t) 	 0x00000000));
	out->setRegister( 87,	BQRegister(0x92FD, (uint8_t) 	 0x00000007));
	out->setRegister( 88,	BQRegister(0x92FE, (uint8_t) 	 0x00000000));
	out->setRegister( 89,	BQRegister(0x92FF, (uint8_t) 	 0x00000000));
	out->setRegister( 90,	BQRegister(0x9300, (uint8_t) 	 0x00000000));
	out->setRegister( 91,	BQRegister(0x9301, (uint8_t) 	 0x00000000));
	out->setRegister( 92,	BQRegister(0x9302, (uint8_t) 	 0x00000000));
	out->setRegister( 93,	BQRegister(0x9303, (uint8_t) 	 0x00000005));
	out->setRegister( 94,	BQRegister(0x9304, (uint16_t) 	 0x00000000));
	out->setRegister( 95,	BQRegister(0x9307, (uint8_t) 	 0x00000050));
	out->setRegister( 96,	BQRegister(0x925F, (uint16_t) 	 0x00000002));
	out->setRegister( 97,	BQRegister(0x9261, (uint8_t) 	 0x00000088));
	out->setRegister( 98,	BQRegister(0x9262, (uint8_t) 	 0x00000000));
	out->setRegister( 99,	BQRegister(0x9263, (uint8_t) 	 0x00000000));
	out->setRegister(100,	BQRegister(0x9265, (uint8_t) 	 0x00000098));
	out->setRegister(101,	BQRegister(0x9266, (uint8_t) 	 0x000000D5));
	out->setRegister(102,	BQRegister(0x9267, (uint8_t) 	 0x00000056));
	out->setRegister(103,	BQRegister(0x9269, (uint8_t) 	 0x000000E4));
	out->setRegister(104,	BQRegister(0x926A, (uint8_t) 	 0x000000E6));
	out->setRegister(105,	BQRegister(0x926B, (uint8_t) 	 0x000000E2));
	out->setRegister(106,	BQRegister(0x9273, (uint16_t) 	 0x00000032));
	out->setRegister(107,	BQRegister(0x926D, (uint16_t) 	 0x0000F800));
	out->setRegister(108,	BQRegister(0x926F, (uint8_t) 	 0x000000FC));
	out->setRegister(109,	BQRegister(0x9270, (uint8_t) 	 0x000000F7));
	out->setRegister(110,	BQRegister(0x9271, (uint8_t) 	 0x000000F4));
	out->setRegister(111,	BQRegister(0x92C4, (uint8_t) 	 0x0000005F));
	out->setRegister(112,	BQRegister(0x92C5, (uint8_t) 	 0x0000009F));
	out->setRegister(113,	BQRegister(0x92C6, (uint8_t) 	 0x00000000));
	out->setRegister(114,	BQRegister(0x92C7, (uint8_t) 	 0x00000000));
	out->setRegister(115,	BQRegister(0x92C0, (uint8_t) 	 0x00000000));
	out->setRegister(116,	BQRegister(0x92C1, (uint8_t) 	 0x00000000));
	out->setRegister(117,	BQRegister(0x92C2, (uint8_t) 	 0x00000007));
	out->setRegister(118,	BQRegister(0x92C3, (uint8_t) 	 0x00000000));
	out->setRegister(119,	BQRegister(0x9308, (uint8_t) 	 0x0000000D));
	out->setRegister(120,	BQRegister(0x9309, (uint8_t) 	 0x00000001));
	out->setRegister(121,	BQRegister(0x930A, (uint16_t) 	 0x00000000));
	out->setRegister(122,	BQRegister(0x930C, (uint16_t) 	 0x00000000));
	out->setRegister(123,	BQRegister(0x930E, (uint8_t) 	 0x00000005));
	out->setRegister(124,	BQRegister(0x930F, (uint8_t) 	 0x00000032));
	out->setRegister(125,	BQRegister(0x9310, (uint16_t) 	 0x00000064));
	out->setRegister(126,	BQRegister(0x9312, (uint16_t) 	 0x00000032));
	out->setRegister(127,	BQRegister(0x9314, (uint8_t) 	 0x00000005));
	out->setRegister(128,	BQRegister(0x9315, (uint16_t) 	 0x00000000));
	out->setRegister(129,	BQRegister(0x9317, (uint16_t) 	 0x00000000));
	out->setRegister(130,	BQRegister(0x9319, (uint16_t) 	 0x00000000));
	out->setRegister(131,	BQRegister(0x931B, (uint16_t) 	 0x00000000));
	out->setRegister(132,	BQRegister(0x931D, (uint16_t) 	 0x00000000));
	out->setRegister(133,	BQRegister(0x931F, (uint16_t) 	 0x00000000));
	out->setRegister(134,	BQRegister(0x9321, (uint16_t) 	 0x00000000));
	out->setRegister(135,	BQRegister(0x9323, (uint16_t) 	 0x00000000));
	out->setRegister(136,	BQRegister(0x9325, (uint16_t) 	 0x00000000));
	out->setRegister(137,	BQRegister(0x9327, (uint16_t) 	 0x00000000));
	out->setRegister(138,	BQRegister(0x9329, (uint16_t) 	 0x00000000));
	out->setRegister(139,	BQRegister(0x932B, (uint16_t) 	 0x00000000));
	out->setRegister(140,	BQRegister(0x932D, (uint16_t) 	 0x00000000));
	out->setRegister(141,	BQRegister(0x932F, (uint16_t) 	 0x00000000));
	out->setRegister(142,	BQRegister(0x9331, (uint16_t) 	 0x00000000));
	out->setRegister(143,	BQRegister(0x9333, (uint16_t) 	 0x00000000));
	out->setRegister(144,	BQRegister(0x9343, (uint16_t) 	 0x00000040));
	out->setRegister(145,	BQRegister(0x9335, (uint8_t) 	 0x00000000));
	out->setRegister(146,	BQRegister(0x9336, (uint8_t) 	 0xFFFFFFEC));
	out->setRegister(147,	BQRegister(0x9337, (uint8_t) 	 0x0000003C));
	out->setRegister(148,	BQRegister(0x9338, (uint8_t) 	 0x00000046));
	out->setRegister(149,	BQRegister(0x9339, (uint8_t) 	 0x00000014));
	out->setRegister(150,	BQRegister(0x933A, (uint8_t) 	 0x00000001));
	out->setRegister(151,	BQRegister(0x933B, (uint16_t) 	 0x00000F3C));
	out->setRegister(152,	BQRegister(0x933D, (uint8_t) 	 0x00000028));
	out->setRegister(153,	BQRegister(0x933E, (uint8_t) 	 0x00000014));
	out->setRegister(154,	BQRegister(0x933F, (uint16_t) 	 0x00000F3C));
	out->setRegister(155,	BQRegister(0x9341, (uint8_t) 	 0x00000028));
	out->setRegister(156,	BQRegister(0x9342, (uint8_t) 	 0x00000014));
	out->setRegister(157,	BQRegister(0x923F, (uint16_t) 	 0x00000000));
	out->setRegister(158,	BQRegister(0x9241, (uint16_t) 	 0x00000258));
	out->setRegister(159,	BQRegister(0x9243, (uint8_t) 	 0x00000001));
	out->setRegister(160,	BQRegister(0x9244, (uint8_t) 	 0x00000055));
	out->setRegister(161,	BQRegister(0x9245, (uint8_t) 	 0x00000005));
	out->setRegister(162,	BQRegister(0x9252, (uint8_t) 	 0x00000000));
	out->setRegister(163,	BQRegister(0x9253, (uint8_t) 	 0x00000000));
	out->setRegister(164,	BQRegister(0x9254, (uint8_t) 	 0x00000000));
	out->setRegister(165,	BQRegister(0x9255, (uint8_t) 	 0x00000005));
	out->setRegister(166,	BQRegister(0x9248, (uint16_t) 	 0x00000014));
	out->setRegister(167,	BQRegister(0x924A, (uint8_t) 	 0x00000005));
	out->setRegister(168,	BQRegister(0x924B, (uint16_t) 	 0x000001F4));
	out->setRegister(169,	BQRegister(0x924D, (uint8_t) 	 0x0000000A));
	out->setRegister(170,	BQRegister(0x924E, (uint16_t) 	 0x000007D0));
	out->setRegister(171,	BQRegister(0x9250, (uint16_t) 	 0x000000C8));
	out->setRegister(172,	BQRegister(0x91E0, (uint16_t) 	 0x00000000));
	out->setRegister(173,	BQRegister(0x9275, (uint8_t) 	 0x00000032));
	out->setRegister(174,	BQRegister(0x9276, (uint16_t) 	 0x0000004A));
	out->setRegister(175,	BQRegister(0x927B, (uint8_t) 	 0x00000002));
	out->setRegister(176,	BQRegister(0x9278, (uint8_t) 	 0x00000056));
	out->setRegister(177,	BQRegister(0x9279, (uint16_t) 	 0x0000004A));
	out->setRegister(178,	BQRegister(0x927C, (uint8_t) 	 0x00000002));
	out->setRegister(179,	BQRegister(0x927D, (uint8_t) 	 0x00000000));
	out->setRegister(180,	BQRegister(0x927E, (uint8_t) 	 0x0000000A));
	out->setRegister(181,	BQRegister(0x927F, (uint8_t) 	 0x0000000F));
	out->setRegister(182,	BQRegister(0x9280, (uint8_t) 	 0x00000002));
	out->setRegister(183,	BQRegister(0x9281, (uint8_t) 	 0x00000004));
	out->setRegister(184,	BQRegister(0x9288, (uint16_t) 	 0xFFFFFF38));
	out->setRegister(185,	BQRegister(0x92B0, (uint16_t) 	 0x000000C8));
	out->setRegister(186,	BQRegister(0x9282, (uint8_t) 	 0x00000004));
	out->setRegister(187,	BQRegister(0x9283, (uint8_t) 	 0x00000001));
	out->setRegister(188,	BQRegister(0x9284, (uint8_t) 	 0x00000003));
	out->setRegister(189,	BQRegister(0x9285, (uint8_t) 	 0x00000007));
	out->setRegister(190,	BQRegister(0x9286, (uint8_t) 	 0x00000000));
	out->setRegister(191,	BQRegister(0x9287, (uint8_t) 	 0x00000002));
	out->setRegister(192,	BQRegister(0x9294, (uint8_t) 	 0x00000005));
	out->setRegister(193,	BQRegister(0x928A, (uint16_t) 	 0xFFFFF060));
	out->setRegister(194,	BQRegister(0x928C, (uint8_t) 	 0x00000002));
	out->setRegister(195,	BQRegister(0x928D, (uint16_t) 	 0x000000C8));
	out->setRegister(196,	BQRegister(0x928F, (uint8_t) 	 0x00000000));
	out->setRegister(197,	BQRegister(0x9290, (uint8_t) 	 0x0000000A));
	out->setRegister(198,	BQRegister(0x9291, (uint8_t) 	 0x0000000F));
	out->setRegister(199,	BQRegister(0x9292, (uint16_t) 	 0x000000C8));
	out->setRegister(200,	BQRegister(0x9295, (uint8_t) 	 0x00000000));
	out->setRegister(201,	BQRegister(0x9296, (uint8_t) 	 0x0000000A));
	out->setRegister(202,	BQRegister(0x9297, (uint8_t) 	 0x0000000F));
	out->setRegister(203,	BQRegister(0x9298, (uint16_t) 	 0x000000C8));
	out->setRegister(204,	BQRegister(0x929A, (uint8_t) 	 0x00000037));
	out->setRegister(205,	BQRegister(0x929B, (uint8_t) 	 0x00000002));
	out->setRegister(206,	BQRegister(0x929C, (uint8_t) 	 0x00000032));
	out->setRegister(207,	BQRegister(0x929D, (uint8_t) 	 0x0000003C));
	out->setRegister(208,	BQRegister(0x929E, (uint8_t) 	 0x00000002));
	out->setRegister(209,	BQRegister(0x929F, (uint8_t) 	 0x00000037));
	out->setRegister(210,	BQRegister(0x92A0, (uint8_t) 	 0x00000050));
	out->setRegister(211,	BQRegister(0x92A1, (uint8_t) 	 0x00000002));
	out->setRegister(212,	BQRegister(0x92A2, (uint8_t) 	 0x00000041));
	out->setRegister(213,	BQRegister(0x92A3, (uint8_t) 	 0x00000055));
	out->setRegister(214,	BQRegister(0x92A4, (uint8_t) 	 0x00000002));
	out->setRegister(215,	BQRegister(0x92A5, (uint8_t) 	 0x00000050));
	out->setRegister(216,	BQRegister(0x92A6, (uint8_t) 	 0x00000000));
	out->setRegister(217,	BQRegister(0x92A7, (uint8_t) 	 0x00000002));
	out->setRegister(218,	BQRegister(0x92A8, (uint8_t) 	 0x00000005));
	out->setRegister(219,	BQRegister(0x92A9, (uint8_t) 	 0x00000000));
	out->setRegister(220,	BQRegister(0x92AA, (uint8_t) 	 0x00000002));
	out->setRegister(221,	BQRegister(0x92AB, (uint8_t) 	 0x00000005));
	out->setRegister(222,	BQRegister(0x92AC, (uint8_t) 	 0xFFFFFFEC));
	out->setRegister(223,	BQRegister(0x92AD, (uint8_t) 	 0x00000002));
	out->setRegister(224,	BQRegister(0x92AE, (uint8_t) 	 0xFFFFFFF1));
	out->setRegister(225,	BQRegister(0x92AF, (uint8_t) 	 0x00000003));
	out->setRegister(226,	BQRegister(0x92B2, (uint16_t) 	 0x0000003C));
	out->setRegister(227,	BQRegister(0x92B4, (uint8_t) 	 0x00000000));
	out->setRegister(228,	BQRegister(0x92B5, (uint8_t) 	 0x00000032));
	out->setRegister(229,	BQRegister(0x92B6, (uint16_t) 	 0x00000001));
	out->setRegister(230,	BQRegister(0x92BA, (uint16_t) 	 0x000000FA));
	out->setRegister(231,	BQRegister(0x92BC, (uint16_t) 	 0x00000708));
	out->setRegister(232,	BQRegister(0x92BE, (uint16_t) 	 0x00000002));
	out->setRegister(233,	BQRegister(0x92C8, (uint16_t) 	 0x000005DC));
	out->setRegister(234,	BQRegister(0x92CA, (uint8_t) 	 0x00000002));
	out->setRegister(235,	BQRegister(0x92CB, (uint16_t) 	 0x00000898));
	out->setRegister(236,	BQRegister(0x92CD, (uint8_t) 	 0x00000005));
	out->setRegister(237,	BQRegister(0x92CE, (uint16_t) 	 0x00001194));
	out->setRegister(238,	BQRegister(0x92D0, (uint8_t) 	 0x00000005));
	out->setRegister(239,	BQRegister(0x92D1, (uint16_t) 	 0x000001F4));
	out->setRegister(240,	BQRegister(0x92D3, (uint8_t) 	 0x00000005));
	out->setRegister(241,	BQRegister(0x92D4, (uint16_t) 	 0x00002710));
	out->setRegister(242,	BQRegister(0x92D6, (uint8_t) 	 0x00000005));
	out->setRegister(243,	BQRegister(0x92D7, (uint16_t) 	 0xFFFF8300));
	out->setRegister(244,	BQRegister(0x92D9, (uint8_t) 	 0x00000005));
	out->setRegister(245,	BQRegister(0x92DA, (uint8_t) 	 0x00000041));
	out->setRegister(246,	BQRegister(0x92DB, (uint8_t) 	 0x00000005));
	out->setRegister(247,	BQRegister(0x92DC, (uint8_t) 	 0x00000055));
	out->setRegister(248,	BQRegister(0x92DD, (uint8_t) 	 0x00000005));
	out->setRegister(249,	BQRegister(0x92DE, (uint16_t) 	 0x00000DAC));
	out->setRegister(250,	BQRegister(0x92E0, (uint16_t) 	 0x0000000A));
	out->setRegister(251,	BQRegister(0x92E2, (uint16_t) 	 0x000001F4));
	out->setRegister(252,	BQRegister(0x92E4, (uint8_t) 	 0x00000005));
	out->setRegister(253,	BQRegister(0x92E5, (uint16_t) 	 0x00000064));
	out->setRegister(254,	BQRegister(0x92E7, (uint16_t) 	 0x00000E74));
	out->setRegister(255,	BQRegister(0x92E9, (uint16_t) 	 0x00000032));
	out->setRegister(256,	BQRegister(0x92EB, (uint16_t) 	 0x000000C8));
	out->setRegister(257,	BQRegister(0x92ED, (uint8_t) 	 0x00000005));
	out->setRegister(258,	BQRegister(0x92EE, (uint16_t) 	 0x00000014));
	out->setRegister(259,	BQRegister(0x92F0, (uint8_t) 	 0x00000005));
	out->setRegister(260,	BQRegister(0x92F1, (uint16_t) 	 0xFFFFFFEC));
	out->setRegister(261,	BQRegister(0x92F3, (uint8_t) 	 0x00000005));
	out->setRegister(262,	BQRegister(0x92F4, (uint16_t) 	 0x00000064));
	out->setRegister(263,	BQRegister(0x92F6, (uint8_t) 	 0x00000005));
	out->setRegister(264,	BQRegister(0x92F7, (uint8_t) 	 0x00000005));
	out->setRegister(265,	BQRegister(0x92F8, (uint8_t) 	 0x00000005));
	out->setRegister(266,	BQRegister(0x92F9, (uint8_t) 	 0x00000005));
	out->setRegister(267,	BQRegister(0x9256, (uint8_t) 	 0x00000000));
	out->setRegister(268,	BQRegister(0x9257, (uint16_t) 	 0x00000414));
	out->setRegister(269,	BQRegister(0x9259, (uint16_t) 	 0x00003672));
	out->setRegister(270,	BQRegister(0x925B, (uint16_t) 	 0x0000FFFF));
	out->setRegister(271,	BQRegister(0x925D, (uint16_t) 	 0x0000FFFF));
}
