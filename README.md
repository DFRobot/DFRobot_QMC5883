# DFRobot_QMC5883
- [中文版](./README_CN.md)

This low-cost 10 DOF (degrees of the freedom) sensor from dfrobot is highly integrated with the ADXL345 accelerometer, QMC5883L magnetometer, ITG3205 gyroscope and BMP280 air pressure sensor and temperature sensor. It embeds a low noise LDO regulator for supplying a wide range of power input, working with a 3V-5V power supply. Certainly, the 10 DOF IMU is directly compatible with Arduino boards.

![正反面svg效果图](./resources/images/SEN0140.png)

## Product Link (https://www.dfrobot.com/product-818.html)

    SKU: SEN0140

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This library provides drivers for the QMC5883, HMC5883 and VMC5883.

## Installation

To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.

## Methods

```C++
    DFRobot_QMC5883(TwoWire*pWire = &Wire, uint8_t I2C_addr = 0x0C);
    /**
     * @fn begin
     * @brief Sensor init
     * @return bool init status
     * @retval true init succeeded
     * @retval false init failed
     */
    bool begin(void);

    /**
     * @fn readRaw
     * @brief Get the data collected by the sensor
     * @return sVector_t The data collected by the sensor
     */
    sVector_t readRaw(void);

    /**
     * @fn setRange
     * @brief Set sensor signal gain range
     * @param range 
     * @n    HMC5883L_RANGE_8_1GA    
     * @n    HMC5883L_RANGE_5_6GA    
     * @n    HMC5883L_RANGE_4_7GA    
     * @n    HMC5883L_RANGE_4GA      
     * @n    HMC5883L_RANGE_2_5GA    
     * @n    HMC5883L_RANGE_1_9GA    
     * @n    HMC5883L_RANGE_1_3GA //default    
     * @n    HMC5883L_RANGE_0_88GA   
     * @n    QMC5883_RANGE_2GA     
     * @n    QMC5883_RANGE_8GA     
     * @n    VCM5883L_RANGE_8GA    
     */
    void  setRange(eRange_t range);

    /**
     * @fn getRange
     * @brief Get sensor signal gain range
     * @return eRange_t
     */
    eRange_t getRange(void);

    /**
     * @fn setMeasurementMode
     * @brief Set measurement mode
     * @param mode
     * @n     HMC5883L_IDLE
     * @n     HMC5883_SINGLE
     * @n     HMC5883L_CONTINOUS
     * @n     QMC5883_SINGLE
     * @n     QMC5883_CONTINOUS
     * @n     VCM5883L_SINGLE
     * @n     VCM5883L_CONTINOUS
     */
    void setMeasurementMode(eMode_t mode);

    /**
     * @fn  getMeasurementMode
     * @brief Get measurement mode
     * @return eMode_t
     */
    eMode_t getMeasurementMode(void);

    /**
     * @fn setDataRate
     * @brief Set the data collection rate of the sensor
     * @param dataRate
     * @n     HMC5883L_DATARATE_75HZ
     * @n     HMC5883L_DATARATE_30HZ
     * @n     HMC5883L_DATARATE_15HZ
     * @n     HMC5883L_DATARATE_7_5HZ
     * @n     HMC5883L_DATARATE_3HZ
     * @n     HMC5883L_DATARATE_1_5HZ
     * @n     HMC5883L_DATARATE_0_75_HZ
     * @n     QMC5883_DATARATE_10HZ
     * @n     QMC5883_DATARATE_50HZ
     * @n     QMC5883_DATARATE_100HZ
     * @n     QMC5883_DATARATE_200HZ
     * @n     VCM5883L_DATARATE_200HZ
     * @n     VCM5883L_DATARATE_100HZ
     * @n     VCM5883L_DATARATE_50HZ
     * @n     VCM5883L_DATARATE_10HZ
     */
    void  setDataRate(eDataRate_t dataRate);

    /**
     * @fn getDataRate
     * @brief Get the data collection rate of the sensor
     * @return eDataRate_t
     */
    eDataRate_t getDataRate(void);

    /**
     * @fn setSamples
     * @brief Set sensor status
     * @param samples
     * @n     HMC5883L_SAMPLES_8
     * @n     HMC5883L_SAMPLES_4
     * @n     HMC5883L_SAMPLES_2
     * @n     HMC5883L_SAMPLES_1
     * @n     QMC5883_SAMPLES_8
     * @n     QMC5883_SAMPLES_4
     * @n     QMC5883_SAMPLES_2
     * @n     QMC5883_SAMPLES_1
     */
    void  setSamples(eSamples_t samples);

    /**
     * @fn getSamples
     * @brief Get sensor status
     * @return eSamples_t
     */
    eSamples_t getSamples(void);

    /**
     * @fn  setDeclinationAngle
     * @brief Set sensor declination angle
     * @param declinationAngle
     */
    void setDeclinationAngle(float declinationAngle);

    /**
     * @fn getHeadingDegrees
     * @brief Set the sensor range
     */
    void getHeadingDegrees(void);

    /**
     * @fn getICType
     * @brief Get sensor type
     * @return int
     */
    int getICType(void);

    /**
     * @fn  isHMC
     * @brief Determine if the sensor type is HMC5883
     * @return bool 
     * @retval ture it is
     * @retval false it isn't
     */
    bool isHMC(void){if(ICType == IC_HMC5883L ){return true;}return false;}

    /**
     * @fn  isQMC
     * @brief Determine if the sensor type is QMC5883
     * @return bool
     * @retval ture it is
     * @retval false it isn't
     */
    bool isQMC(void){if(ICType == IC_QMC5883 ){return true;}return false;}

    /**
     * @fn  isVCM
     * @brief Determine if the sensor type is VMC5883
     * @return bool
     * @retval ture it is
     * @retval false it isn't
     */
    bool isVCM(void){if(ICType == IC_VCM5883L ){return true;}return false;}
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
Arduino uno        |      √       |              |             | 
Mega2560        |      √       |              |             | 
Leonardo        |      √       |              |             | 
ESP32           |      √       |              |             | 
ESP8266           |      √       |              |             | 
micro:bit        |      √       |              |             | 

## History

- 2022/2/23 - Version 1.0.0 released.

## Credits

Written by PengKaixing(kaixing.peng@dfrobot.com), 2019. (Welcome to our [website](https://www.dfrobot.com/))
