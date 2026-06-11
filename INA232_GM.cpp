/*****************************************************************
* This is a library for the INA232 Current and Power Sensor Module
******************************************************************/

#include "INA232_GM.h"

bool INA232_GM::init(){
    _wire->beginTransmission(i2cAddress);
    if(_wire->endTransmission()){
        return 0;
    }
    reset_INA232();
    calVal = 2048; // default
    writeRegister(INA232_CAL_REG, calVal);
    setAverage(INA232_AVERAGE_1);
    setConversionTime(INA232_CONV_TIME_1100);
    setMeasureMode(INA232_CONTINUOUS);
    currentDivider_mA = 4.0;
    pwrMultiplier_mW = 6.25;
    convAlert = false;
    limitAlert = false;
    corrFactor = 1.0;
    i2cErrorCode = 0;
    return 1;
}

void INA232_GM::reset_INA232(){
    writeRegister(INA232_CONF_REG, INA232_RST);
}

void INA232_GM::setCorrectionFactor(float corr){
    corrFactor = corr;
    uint16_t calValCorrected = static_cast<uint16_t>(calVal * corrFactor);
    writeRegister(INA232_CAL_REG, calValCorrected);
}

void INA232_GM::setAverage(INA232_AVERAGES averages){
    deviceAverages = averages;
    uint16_t currentConfReg = readRegister(INA232_CONF_REG);
    currentConfReg &= ~(0x0E00);
    currentConfReg |= deviceAverages;
    writeRegister(INA232_CONF_REG, currentConfReg);
}

void INA232_GM::setConversionTime(INA232_CONV_TIME shuntConvTime, INA232_CONV_TIME busConvTime){
    uint16_t currentConfReg = readRegister(INA232_CONF_REG);
    currentConfReg &= ~(0x01C0);
    currentConfReg &= ~(0x0038);
    uint16_t convMask = (static_cast<uint16_t>(shuntConvTime))<<3;
    currentConfReg |= convMask;
    convMask = busConvTime<<6;
    currentConfReg |= convMask;
    writeRegister(INA232_CONF_REG, currentConfReg);
}

void INA232_GM::setConversionTime(INA232_CONV_TIME convTime){
    setConversionTime(convTime, convTime);
}

void INA232_GM::setMeasureMode(INA232_MEASURE_MODE mode){
    deviceMeasureMode = mode;
    uint16_t currentConfReg = readRegister(INA232_CONF_REG);
    currentConfReg &= ~(0x0007);
    currentConfReg |= deviceMeasureMode;
    writeRegister(INA232_CONF_REG, currentConfReg);
}

//set resistor and current range independent. resistor value in ohm, current range in A
void INA232_GM::setResistorRange(float resistor, float current_range){
    if(current_range < 0) {
        current_range = 0.0819175/resistor;
    }

    float current_LSB=current_range/32768.0;

    calVal = 0.00512/(current_LSB*resistor);
    currentDivider_mA = 0.001/current_LSB;
    pwrMultiplier_mW = 1000.0*25.0*current_LSB;

    writeRegister(INA232_CAL_REG, calVal);
}

float INA232_GM::getShuntVoltage_V(){
    int16_t val;
    val = static_cast<int16_t>(readRegister(INA232_SHUNT_REG));
    return (val * 0.0000025 * corrFactor);
}

float INA232_GM::getShuntVoltage_mV(){
    int16_t val;
    val = static_cast<int16_t>(readRegister(INA232_SHUNT_REG));
    return (val * 0.0025 * corrFactor);
}

float INA232_GM::getBusVoltage_V(){
    uint16_t val;
    val = readRegister(INA232_BUS_REG);
    return (val * 0.0016);
}

float INA232_GM::getCurrent_mA(){
    int16_t val;
    val = static_cast<int16_t>(readRegister(INA232_CURRENT_REG));
    return (val / currentDivider_mA);
}

float INA232_GM::getCurrent_A() {
    return (getCurrent_mA()/1000);
}

float INA232_GM::getBusPower(){
    uint16_t val;
    val = readRegister(INA232_PWR_REG);
    return (val * pwrMultiplier_mW);
}

uint16_t INA232_GM::getManufacturerID(){
    uint16_t val;
    val = readRegister(INA232_MAN_ID_REG);
    return val;
}

void INA232_GM::startSingleMeasurement(){
    uint16_t val = readRegister(INA232_MASK_EN_REG); // clears CNVR (Conversion Ready) Flag
    val = readRegister(INA232_CONF_REG);
    writeRegister(INA232_CONF_REG, val);        // Starts conversion
    uint16_t convReady = 0x0000;
    unsigned long convStart = millis();
    while(!convReady && ((millis()-convStart) < 2000)){
        convReady = ((readRegister(INA232_MASK_EN_REG)) & 0x0008); // checks if sampling is completed
    }
}

// Don't wait for conversion to complete
void INA232_GM::startSingleMeasurementNoWait(){
    uint16_t val = readRegister(INA232_MASK_EN_REG); // clears CNVR (Conversion Ready) Flag
    val = readRegister(INA232_CONF_REG);
    writeRegister(INA232_CONF_REG, val);        // Starts conversion
}

void INA232_GM::powerDown(){
    confRegCopy = readRegister(INA232_CONF_REG);
    setMeasureMode(INA232_POWER_DOWN);
}

void INA232_GM::powerUp(){
    writeRegister(INA232_CONF_REG, confRegCopy);
    delayMicroseconds(40);
}

// Returns 1 if conversion is still ongoing
bool INA232_GM::isBusy(){
    return (!(readRegister(INA232_MASK_EN_REG) &0x0008));
}

void INA232_GM::waitUntilConversionCompleted(){
    readRegister(INA232_MASK_EN_REG); // clears CNVR (Conversion Ready) Flag
    uint16_t convReady = 0x0000;
    while(!convReady){
        convReady = ((readRegister(INA232_MASK_EN_REG)) & 0x0008); // checks if sampling is completed
    }
}

void INA232_GM::setAlertPinActiveHigh(){
    uint16_t val = readRegister(INA232_MASK_EN_REG);
    val |= 0x0002;
    writeRegister(INA232_MASK_EN_REG, val);
}

void INA232_GM::enableAlertLatch(){
    uint16_t val = readRegister(INA232_MASK_EN_REG);
    val |= 0x0001;
    writeRegister(INA232_MASK_EN_REG, val);
}

void INA232_GM::enableConvReadyAlert(){
    uint16_t val = readRegister(INA232_MASK_EN_REG);
    val |= 0x0400;
    writeRegister(INA232_MASK_EN_REG, val);
}
/*
void INA226_WE::setAlertType(INA226_ALERT_TYPE type, float limit){
    deviceAlertType = type;
    uint16_t alertLimit = 0;

    switch(deviceAlertType){
        case INA226_SHUNT_OVER:
            alertLimit = limit * 400;
            break;
        case INA226_SHUNT_UNDER:
            alertLimit = limit * 400;
            break;
        case INA226_CURRENT_OVER:
            deviceAlertType = INA226_SHUNT_OVER;
            alertLimit = limit * 2048 * currentDivider_mA / calVal;
            break;
        case INA226_CURRENT_UNDER:
            deviceAlertType = INA226_SHUNT_UNDER;
            alertLimit = limit * 2048 * currentDivider_mA / calVal;
            break;
        case INA226_BUS_OVER:
            alertLimit = limit * 800;
            break;
        case INA226_BUS_UNDER:
            alertLimit = limit * 800;
            break;
        case INA226_POWER_OVER:
            alertLimit = limit / pwrMultiplier_mW;
            break;
    }

    writeRegister(INA226_ALERT_LIMIT_REG, alertLimit);

    uint16_t value = readRegister(INA226_MASK_EN_REG);
    value &= ~(0xF800);
    value |= deviceAlertType;
    writeRegister(INA226_MASK_EN_REG, value);

}
*/
void INA232_GM::readAndClearFlags(){
    uint16_t value = readRegister(INA232_MASK_EN_REG);
    overflow = (value>>2) & 0x0001;
    convAlert = (value>>3) & 0x0001;
    limitAlert = (value>>4) & 0x0001;
}

uint8_t INA232_GM::getI2cErrorCode(){
    return i2cErrorCode;
}


/************************************************
    private functions
*************************************************/

void INA232_GM::writeRegister(uint8_t reg, uint16_t val){
  _wire->beginTransmission(i2cAddress);
  uint8_t lVal = val & 255;
  uint8_t hVal = val >> 8;
  _wire->write(reg);
  _wire->write(hVal);
  _wire->write(lVal);
  _wire->endTransmission();
}

uint16_t INA232_GM::readRegister(uint8_t reg){
  uint8_t MSByte = 0, LSByte = 0;
  uint16_t regValue = 0;
  _wire->beginTransmission(i2cAddress);
  _wire->write(reg);
  i2cErrorCode = _wire->endTransmission(false);
  _wire->requestFrom(static_cast<uint8_t>(i2cAddress),static_cast<uint8_t>(2));
  if(_wire->available()){
    MSByte = _wire->read();
    LSByte = _wire->read();
  }
  regValue = (MSByte<<8) + LSByte;
  return regValue;
}
