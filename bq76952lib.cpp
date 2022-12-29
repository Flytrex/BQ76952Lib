/*
* Description :   Source file of BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   Pranjal Joshi
* Date        :   17/10/2020 
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#include "bq76952lib.h"

#include "util/dbg-macro.h"

#define message(M, ...) debug("bq76952: " M, ##__VA_ARGS__)
#define checkbq(A, M, ...) check(A, "bq76952: " M, ##__VA_ARGS__)

// Library config
#define DBG_BAUD            115200
#define BQ_I2C_ADDR   0x08

// BQ76952 - Address Map
#define CMD_DIR_SUBCMD_LOW            0x3E
#define CMD_DIR_SUBCMD_HI             0x3F
#define CMD_DIR_RESP_LEN              0x61
#define CMD_DIR_RESP_START            0x40
#define CMD_DIR_RESP_CHKSUM           0x60

// BQ76952 - Voltage measurement commands
#define CMD_READ_VOLTAGE_CELL_1   0x14
#define CMD_READ_VOLTAGE_CELL_2   0x16
#define CMD_READ_VOLTAGE_CELL_3   0x18
#define CMD_READ_VOLTAGE_CELL_4   0x1A
#define CMD_READ_VOLTAGE_CELL_5   0x1C
#define CMD_READ_VOLTAGE_CELL_6   0x1E
#define CMD_READ_VOLTAGE_CELL_7   0x20
#define CMD_READ_VOLTAGE_CELL_8   0x22
#define CMD_READ_VOLTAGE_CELL_9   0x24
#define CMD_READ_VOLTAGE_CELL_10  0x26
#define CMD_READ_VOLTAGE_CELL_11  0x28
#define CMD_READ_VOLTAGE_CELL_12  0x2A
#define CMD_READ_VOLTAGE_CELL_13  0x2C
#define CMD_READ_VOLTAGE_CELL_14  0x2E
#define CMD_READ_VOLTAGE_CELL_15  0x30
#define CMD_READ_VOLTAGE_CELL_16  0x32
#define CMD_READ_VOLTAGE_STACK    0x34
#define CMD_READ_VOLTAGE_PACK     0x36

// BQ76952 - Direct Commands
#define CMD_DIR_SPROTEC           0x02
#define CMD_DIR_FPROTEC           0x03
#define CMD_DIR_STEMP             0x04
#define CMD_DIR_FTEMP             0x05
#define CMD_DIR_SFET              0x06
#define CMD_DIR_FFET              0x07
#define CMD_DIR_VCELL_1           0x14
#define CMD_DIR_INT_TEMP          0x68
#define CMD_DIR_CC2_CUR           0x3A
#define CMD_DIR_FET_STAT          0x7F

// Alert Bits in BQ76952 registers
#define BIT_SA_SC_DCHG            7
#define BIT_SA_OC2_DCHG           6
#define BIT_SA_OC1_DCHG           5
#define BIT_SA_OC_CHG             4
#define BIT_SA_CELL_OV            3
#define BIT_SA_CELL_UV            2

#define BIT_SB_OTF                7
#define BIT_SB_OTINT              6
#define BIT_SB_OTD                5
#define BIT_SB_OTC                4
#define BIT_SB_UTINT              2
#define BIT_SB_UTD                1
#define BIT_SB_UTC                0

// Inline functions
#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))
#define LOW_BYTE(data) (byte)(data & 0x00FF)
#define HIGH_BYTE(data) (byte)((data >> 8) & 0x00FF)

//// LOW LEVEL FUNCTIONS ////

int bq76952::directCommandWrite(byte command, size_t size, int data)
{
  /* Transmit address+w, command address, and data.
   * "size" must match the command in datasheet - that's on the caller.
   * Many commands are read-only - that's also on the caller. 
   * return 0 on success, -1 on fail. */
  uint8_t *puData = (uint8_t *) data; 
  int erc = 0;
  checkbq(size <= 4, "%s: invalid size %lu", __func__, size);
  m_I2C->beginTransmission(BQ_I2C_ADDR);
  checkbq(m_I2C->write(command), "%s: write(%02X) failed", __func__, command);
  checkbq(size == m_I2C->write(puData, size), "%s: write(data=%04X, size=%d) failed", __func__,  data, size);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s: endTransmission unexcepted result %d", erc);
  return 0;

  error:
  return -1;
}

int bq76952::directCommandRead(byte command, size_t size, int *o_data)
{
  /* Transmit address+w, command adresss, repeated start, address+r, read data. */
  uint8_t *puData = (uint8_t *) o_data; 
  int erc = 0;
  checkbq(size <= 4, "%s: invalid size %lu", __func__, size);
  m_I2C->beginTransmission(BQ_I2C_ADDR);
  checkbq(m_I2C->write(command), "%s: write(%02X) failed", __func__, command);
  checkbq(!(erc = m_I2C->endTransmission(true)), "%s: endTransmission unexcepted result %d", __func__, erc);
  checkbq(size == (erc = m_I2C->requestFrom(BQ_I2C_ADDR, size)), 
                                          "%s: requestFrom(expected=%d) received %d", __func__, size, erc);
  for (int i = 0; i < size; ++i) {
    o_data[i] = m_I2C->read();
  }
  return 0;

  error:
  return -1;
}

/*
The direct commands are accessed using a 7-bit command address that is sent from
a host through the device serial communications interface and either triggers an
action, or provides a data value to be written to the device, or instructs the
device to report data back to the host.
*/
// Send Direct command
int bq76952::directCommand(byte command, uint16_t *o_response) 
{
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(command);
  Wire.endTransmission();
  byte lsb, msb;

  checkbq(2 == Wire.requestFrom(BQ_I2C_ADDR, 2), "command %02X timed out", command); 
  // buggy - synchronous and waits for 1000ms https://github.com/espressif/esp-idf/issues/4999
  // See also: https://github.com/ci4rail/esp-idf/commit/bb2e8831bf44286181d86abd3156b70b65363782   
  lsb = Wire.read();
  msb = Wire.read();

  *o_response = (msb << 8 | lsb);
  message("[+] Direct Cmd SENT -> 0x%02X", command);
  message("[+] Direct Cmd RESP <- 0x%04X", *o_response);
  return 0;

  error:
  return -1;
}

/*
https://www.ti.com/document-viewer/lit/html/SLUUBY2B/GUID-725A7D59-C6EA-4DE7-8CD2-9B117C75A18D#TITLE-SLUUBY2T6140828-4B

Subcommands are additional commands that are accessed indirectly using the 7-bit
command address space and provide the capability for block data transfers. When
a subcommand is initiated, a 16-bit subcommand address is first written to the
7-bit command addresses 0x3E (lower byte) and 0x3F (upper byte). The device
initially assumes a read-back of data may be needed, and auto-populates existing
data into the 32-byte transfer buffer (which uses 7-bit command addresses
0x40–0x5F), and writes the checksum for this data into address 0x60. If the host
instead intends to write data into the device, the host will overwrite the new
data into the transfer buffer, a checksum for the data into address 0x60, and
the data length into address 0x61. As soon as address 0x61 is written, the
device checks the checksum written into 0x60 with the data written into
0x40-0x5F, and if this is correct, it proceeds to transfer the data from the
transfer buffer into the device's memory.

The checksum is the 8-bit sum of the subcommand bytes (0x3E and 0x3F) plus the
number of bytes used in the transfer buffer, then the result is bitwise
inverted. The verification cannot take place until the data length is written,
so the device realizes how many bytes in the transfer buffer are included. The
checksum and data length must be written together as a word in order to be
valid. The data length includes the two bytes in 0x3E and 0x3F, the two bytes in
0x60 and 0x61, and the length of the transfer buffer. Therefore, if the entire
32-byte transfer buffer is used, the data length will be 0x24.


Some subcommands are only used to initiate an action and do not involve sending
or receiving data. In these cases, the host can simply write the subcommand into
0x3E and 0x3F, it is not necessary to write the length and checksum or any
further data.
*/
// Send Sub-command
void bq76952::subCommand(unsigned int data) 
{
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write((byte *) &data, 2);
  Wire.endTransmission();
  message("[+] Sub Cmd SENT to 0x3E -> 0x%02X", data);
}

/*
  1. Write lower byte of subcommand to 0x3E.
  2. Write upper byte of subcommand to 0x3F.
  3. Read 0x3E and 0x3F. If this returns 0xFF, this indicates the subcommand has
    not completed operation yet. When the subcommand has completed, the readback
    will return what was originally written. Continue reading 0x3E and 0x3F until
    it returns what was written originally. Note: this response only applies to
    subcommands that return data to be read back. 
  4. Read the length of response from 0x61. 
  5. Read buffer starting at 0x40 for the expected length. 
  6. Read the checksum at 0x60 and verify it matches the data read.

  Note: 0x61 provides the length of the buffer data plus 4 (that is, length of the buffer data plus the length of 0x3E and 0x3F plus the length of 0x60 and 0x61).

  The checksum is calculated over (the content of) 0x3E, 0x3F, and the buffer data, it does not include the checksum or length in 0x60 and 0x61.
*/
// FTX: tested
bool reqestResponse(uint8_t reg, uint8_t size, uint8_t* buff) 
{
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)BQ_I2C_ADDR, size);
  if (Wire.available() < size) {
    return false;
  }

  for(int i=0; i<size; ++i) {
    buff[i] = Wire.read();
  }

  return true;
}

// FTX: tested
bool bq76952::subCommandWithResponse(uint16_t subCommand, uint8_t* buffer, uint8_t* buffer_len) 
{
  // 1+2. Write subCommand code to 0x3e
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write((byte *) &subCommand, 2);
  Wire.endTransmission();

  byte replyLen = 0;
  byte checksumFromDevice = 0;
  byte chksum = 0;

  // 3. Readback subcommand code to veryfy completetion
  byte retrys = 10;
  do {
    uint16_t subCommandReadback;
    checkbq(reqestResponse(CMD_DIR_SUBCMD_LOW, sizeof(subCommandReadback), (byte *) &subCommandReadback), "subcommand timeout");
    retrys--;
    checkbq(retrys > 0, "timeout waitng for subcommand");
    if(subCommandReadback == 0xffff || subCommandReadback == 0x1111) {
      message("not ready -> expect %x recv %x %d", subCommand, subCommandReadback, retrys);
      delay(1);
      continue;
    } 
    else {
      check(subCommandReadback == subCommand, "Bad Readback -> expect %x recv %x", subCommand, subCommandReadback);
    }
  } while (true);

  //4. Read the length of response from 0x61. 
  checkbq(reqestResponse(CMD_DIR_RESP_LEN, 1, &replyLen), "failed to read response")
  replyLen -= 4;

  //5. Read buffer starting at 0x40 for the expected length. 
  checkbq(replyLen < *buffer, "reply len =")
  if (replyLen > *buffer_len) {
    return false;
  }

  checkbq(reqestResponse(CMD_DIR_RESP_START, replyLen, buffer), "subcommand %x reply timeout", subCommand);
  *buffer_len = replyLen;

  //6. Read the checksum at 0x60 and verify it matches the data read.
  check(reqestResponse(CMD_DIR_RESP_CHKSUM, 1, &checksumFromDevice), "sub CRC timeout");

  chksum = computeChecksum(chksum, LOW_BYTE(subCommand), true);
  chksum = computeChecksum(chksum, HIGH_BYTE(subCommand));
  for(int i=0; i < replyLen; ++i) {
    chksum = computeChecksum(chksum, buffer[i]);
  }
  checkbq(chksum == checksumFromDevice, "checksum err: calc %x recv %x\n", chksum, checksumFromDevice); 
  return true;

  error:
  return false;
}

// Enter config update mode
void bq76952::enterConfigUpdate(void) 
{
  subCommand(0x0090);
  delay(500);
}

// Exit config update mode
void bq76952::exitConfigUpdate(void) 
{
  subCommand(0x0092);
  delay(500);
}

/*
  https://www.ti.com/document-viewer/lit/html/SLUUBY2B/GUID-9ED2EB91-EA65-49C7-9667-53B029541C4D#TITLE-SLUUBY2T5345357-60

  1. Place the device into CONFIG_UPDATE mode by sending the 0x0090 ENTER_CFG_UPDATE() subcommand. 
      The device will then automatically disable the protection FETs if they are enabled.
	2. Wait for the 0x12 Battery Status()[CFGUPDATE] flag to set.
	3. Modify settings as needed by writing updated data memory settings (for more information, see Data Memory Access).
	4. Send the 0x0092 EXIT_CFG_UPDATE() command to resume firmware operation
*/

// Write Byte to Data memory of BQ76952
void bq76952::writeDataMemory(unsigned int addr, unsigned int data, byte noOfBytes) 
{
  byte chksum = 0;
  chksum = computeChecksum(chksum, BQ_I2C_ADDR);
  chksum = computeChecksum(chksum, CMD_DIR_SUBCMD_LOW);
  chksum = computeChecksum(chksum, LOW_BYTE(addr));
  chksum = computeChecksum(chksum, HIGH_BYTE(addr));
  chksum = computeChecksum(chksum, data);

  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write(LOW_BYTE(addr));
  Wire.write(HIGH_BYTE(addr));
  Wire.write(LOW_BYTE(data));
  if(noOfBytes == 2)                                /* FIXME */
    Wire.write(HIGH_BYTE(data));
  Wire.endTransmission();

  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_RESP_CHKSUM);
  Wire.write(chksum);
  Wire.write(0x05);   // size ? what about the case where noOfBytes = 2 ?
  Wire.endTransmission();
}

// Read Byte from Data memory of BQ76952
byte bq76952::readDataMemory(unsigned int addr) 
{
  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write(LOW_BYTE(addr));
  Wire.write(HIGH_BYTE(addr));
  Wire.endTransmission();

  Wire.beginTransmission(BQ_I2C_ADDR);
  Wire.write(CMD_DIR_RESP_START);
  Wire.endTransmission();

  Wire.requestFrom(BQ_I2C_ADDR, 1);
  while(!Wire.available());         /* FIXME */
  return (byte) Wire.read();
}

// Compute checksum = ~(sum of all bytes)
byte bq76952::computeChecksum(byte oldChecksum, byte data, bool reset) {
  if(reset)
    oldChecksum = data;
  else
    oldChecksum = ~(oldChecksum) + data;
  return ~(oldChecksum);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////


/////// API FUNCTIONS ///////

int bq76952::begin(byte alertPin, TwoWire *i2c) 
{
  checkbq(0 <= alertPin && alertPin <= 22, "invalid pin number %d", alertPin);
  m_alertPin = alertPin;
  pinMode(alertPin, INPUT);
  m_I2C = i2c;
  return 0;

  error:
  return -1;
}
#if 0
// FTX: tested
bool bq76952::isConnected(void) {
  Wire.beginTransmission(BQ_I2C_ADDR);
  if(Wire.endTransmission() == 0) {
    message("[+] BQ76592 -> Connected on I2C");
    return true;
  }
  else {
    message("[+] BQ76592 -> Not Detected on I2C");
    return false;
  }
}

// Reset the BQ chip
void bq76952::reset(void) {
  subCommand(0x0012);
  debugPrintln("[+] Resetting BQ76952...");
}

// FTX: tested
// Read single cell voltage
unsigned int bq76952::getCellVoltage(byte cellNumber) {
  return directCommand(CELL_NO_TO_ADDR(cellNumber));
}

// Read All cell voltages in given array - Call like readAllCellVoltages(&myArray)
void bq76952::getAllCellVoltages(unsigned int* cellArray) {
  for(byte x=1;x<17;x++)
    cellArray[x] = getCellVoltage(x);
}

// Measure CC2 current
unsigned int bq76952::getCurrent(void) {
  return directCommand(CMD_DIR_CC2_CUR);
}

// FTX: tested
// Measure chip temperature in °C
float bq76952::getInternalTemp(void) {
  float raw = directCommand(CMD_DIR_INT_TEMP)/10.0;
  return (raw - 273.15);
}

// Measure thermistor temperature in °C
float bq76952::getThermistorTemp(bq76952_thermistor thermistor) {
  byte cmd;
  switch(thermistor) {
    case TS1:
      cmd = 0x70;
      break;
    case TS2:
      cmd = 0x72;
      break;
    case TS3:
      cmd = 0x74;
      break;
    case HDQ:
      cmd = 0x76;
      break;
    case DCHG:
      cmd = 0x78;
      break;
    case DDSG:
      cmd = 0x7A;
      break;
  }
  float raw = directCommand(cmd)/10.0;
  return (raw - 273.15);
}

// Check Primary Protection status
bq76952_protection_t bq76952::getProtectionStatus(void) {
  bq76952_protection_t prot;
  byte regData = (byte)directCommand(CMD_DIR_FPROTEC);
  prot.bits.SC_DCHG = bitRead(regData, BIT_SA_SC_DCHG);
  prot.bits.OC2_DCHG = bitRead(regData, BIT_SA_OC2_DCHG);
  prot.bits.OC1_DCHG = bitRead(regData, BIT_SA_OC1_DCHG);
  prot.bits.OC_CHG = bitRead(regData, BIT_SA_OC_CHG);
  prot.bits.CELL_OV = bitRead(regData, BIT_SA_CELL_OV);
  prot.bits.CELL_UV = bitRead(regData, BIT_SA_CELL_UV);
  return prot;
}

// Check Temperature Protection status
bq76952_temperature_t bq76952::getTemperatureStatus(void) {
  bq76952_temperature_t prot;
  byte regData = (byte)directCommand(CMD_DIR_FTEMP);
  prot.bits.OVERTEMP_FET = bitRead(regData, BIT_SB_OTC);
  prot.bits.OVERTEMP_INTERNAL = bitRead(regData, BIT_SB_OTINT);
  prot.bits.OVERTEMP_DCHG = bitRead(regData, BIT_SB_OTD);
  prot.bits.OVERTEMP_CHG = bitRead(regData, BIT_SB_OTC);
  prot.bits.UNDERTEMP_INTERNAL = bitRead(regData, BIT_SB_UTINT);
  prot.bits.UNDERTEMP_DCHG = bitRead(regData, BIT_SB_UTD);
  prot.bits.UNDERTEMP_CHG = bitRead(regData, BIT_SB_UTC);
  return prot;
}

void bq76952::setFET(bq76952_fet fet, bq76952_fet_state state) {
  unsigned int subcmd;
  switch(state) {
    case OFF:
      switch(fet) {
        case DCHG:
          subcmd = 0x0093;
          break;
        case CHG:
          subcmd = 0x0094;
          break;
        default:
          subcmd = 0x0095;
          break;
      }
      break;
    case ON:
      subcmd = 0x0096;
      break;
  }
  subCommand(subcmd);
}
// is Charging FET ON?
bool bq76952::isCharging(void) {
  byte regData = (byte)directCommand(CMD_DIR_FET_STAT);
  if(regData & 0x01) {
    debugPrintln("[+] Charging FET -> ON");
    return true;
  }
  debugPrintln("[+] Charging FET -> OFF");
  return false;
}

// is Discharging FET ON?
bool bq76952::isDischarging(void) {
  byte regData = (byte)directCommand(CMD_DIR_FET_STAT);
  if(regData & 0x04) {
    debugPrintln("[+] Discharging FET -> ON");
    return true;
  }
  debugPrintln("[+] Discharging FET -> OFF");
  return false;
}



// Set user-defined overvoltage protection
void bq76952::setCellOvervoltageProtection(unsigned int mv, unsigned int ms) {
  byte thresh = (byte)mv/50.6;
  uint16_t dly = (uint16_t)(ms/3.3)-2;
  if(thresh < 20 || thresh > 110)
    thresh = 86;
  else {
    debugPrint("[+] COV Threshold => ");
    debugPrintlnCmd(thresh);
    writeDataMemory(0x9278, thresh, 1);
  }
  if(dly < 1 || dly > 2047)
    dly = 74;
  else {
    debugPrint("[+] COV Delay => ");
    debugPrintlnCmd(dly);
    writeDataMemory(0x9279, dly, 2);
  }
}

// Set user-defined undervoltage protection
void bq76952::setCellUndervoltageProtection(unsigned int mv, unsigned int ms) {
  byte thresh = (byte)mv/50.6;
  uint16_t dly = (uint16_t)(ms/3.3)-2;
  if(thresh < 20 || thresh > 90)
    thresh = 50;
  else {
    debugPrint("[+] CUV Threshold => ");
    debugPrintlnCmd(thresh);
    writeDataMemory(0x9275, thresh, 1);
  }
  if(dly < 1 || dly > 2047)
    dly = 74;
  else {
    debugPrint("[+] CUV Delay => ");
    debugPrintlnCmd(dly);
    writeDataMemory(0x9276, dly, 2);
  }
}

// Set user-defined charging current protection
void bq76952::setChargingOvercurrentProtection(byte mv, byte ms) {
  byte thresh = (byte)mv/2;
  byte dly = (byte)(ms/3.3)-2;
  if(thresh < 2 || thresh > 62)
    thresh = 2;
  else {
    debugPrint("[+] OCC Threshold => ");
    debugPrintlnCmd(thresh);
    writeDataMemory(0x9280, thresh, 1);
  }
  if(dly < 1 || dly > 127)
    dly = 4;
  else {
    debugPrint("[+] OCC Delay => ");
    debugPrintlnCmd(dly);
    writeDataMemory(0x9281, dly, 1);
  }
}

// Set user-defined discharging current protection
void bq76952::setDischargingOvercurrentProtection(byte mv, byte ms) {
  byte thresh = (byte)mv/2;
  byte dly = (byte)(ms/3.3)-2;
  if(thresh < 2 || thresh > 100)
    thresh = 2;
  else {
    debugPrint("[+] OCD Threshold => ");
    debugPrintlnCmd(thresh);
    writeDataMemory(0x9282, thresh, 1);
  }
  if(dly < 1 || dly > 127)
    dly = 1;
  else {
    debugPrint("[+] OCD Delay => ");
    debugPrintlnCmd(dly);
    writeDataMemory(0x9283, dly, 1);
  }
}

// Set user-defined discharging current protection
void bq76952::setDischargingShortcircuitProtection(bq76952_scd_thresh thresh, unsigned int us) {
  byte dly = (byte)(us/15)+1;
  debugPrint("[+] SCD Threshold => ");
  debugPrintlnCmd(thresh);
  writeDataMemory(0x9286, thresh, 1);
  if(dly < 1 || dly > 31)
    dly = 2;
  else {
    debugPrint("[+] SCD Delay (uS) => ");
    debugPrintlnCmd(dly);
    writeDataMemory(0x9287, dly, 1);
  }
}

// Set user-defined charging over temperature protection
void bq76952::setChargingTemperatureMaxLimit(signed int temp, byte sec) {
  if(temp < -40 || temp > 120)
    temp = 55;
  else {
    debugPrint("[+] OTC Threshold => ");
    debugPrintlnCmd(temp);
    writeDataMemory(0x929A, temp, 1);
  }
  if(sec< 0 || sec > 255)
    sec = 2;
  else {
    debugPrint("[+] OTC Delay => ");
    debugPrintlnCmd(sec);
    writeDataMemory(0x929B, sec, 1);
  }
}

// Set user-defined discharging over temperature protection
void bq76952::setDischargingTemperatureMaxLimit(signed int temp, byte sec) {
  if(temp < -40 || temp > 120)
    temp = 60;
  else {
    debugPrintf("[+] OTD Threshold => %#x\n", temp);
    writeDataMemory(0x929D, temp, 1);
  }
  if(sec< 0 || sec > 255)
    sec = 2;
  else {
    debugPrintf("[+] OTD Delay => %#x\n", sec);
    writeDataMemory(0x929E, sec, 1);
  }
}
#endif
