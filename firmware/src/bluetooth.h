#pragma once

#include "BLEPeripheral.h"
#include "services.h"
#include "position_calculator.h"

typedef void (*cb_baseStationGeometry)(BaseStation *baseStations);

typedef struct {
  float32_t x;
  float32_t y;
  float32_t z;
} Vec3;

typedef struct {
    float32_t m11;
    float32_t m12;
    float32_t m13;
    float32_t m21;
    float32_t m22;
    float32_t m23;
    float32_t m31;
    float32_t m32;
    float32_t m33;
} Mat3;

class Bluetooth {
    public:
        void sendPosition(vec3d &position);
        void poll(void);
        void setBaseStationGeometryCB(cb_baseStationGeometry pCallback);
        void setup(void);
        void printBytes(unsigned char* ptr, unsigned char size);

        void stationNumberWritten(BLECentral &central, BLECharacteristic &characteristic);
        void positionWritten(BLECentral &central, BLECharacteristic &characteristic);
        void rotationWritten(BLECentral &central, BLECharacteristic &characteristic);
        
        static Bluetooth* getInstance(void);

    private:
        Bluetooth() {}

        static Bluetooth* instance;

        cb_baseStationGeometry stationCb;
        BaseStation baseStations[baseStationsCount];
        uint8_t baseStationIndex = 0;   // index of base station where position and rotation will be written

        void setPositionCharValueByIndex(void);
        void setRotationCharValueByIndex(void);
        
        /**
         * @brief Set the Row To Rotation Matrix object
         * 
         * @param rowNum starts from 0 .. [0 - 2]
         * @param row value of row to be set
         */
        void setRowToRotationMatrix(uint8_t rowNum, Vec3 row);

        BLEPeripheral peripheral = BLEPeripheral();

        BLEService coordinatesService = BLEService(coordinatesServiceUUID);
        BLETypedCharacteristic<Vec3> coordinatesCharacteristic = 
            BLETypedCharacteristic<Vec3>(coordinatesCharacteristicUUID, BLERead | BLENotify);

        BLEService baseStationGeometryService = BLEService(baseStationGeometryServiceUUID);
        BLECharCharacteristic stationNumberCharacteristic = 
            BLECharCharacteristic(stationNumberCharacteristicUUID, BLERead | BLEWrite);
        BLETypedCharacteristic<Vec3> positionCharacteristic = 
            BLETypedCharacteristic<Vec3>(positionCharacteristicUUID, BLERead | BLEWrite);
        BLETypedCharacteristic<Vec3> rotation1RowCharacteristic = 
            BLETypedCharacteristic<Vec3>(rotation1RowCharacteristicUUID, BLERead | BLEWrite);
        BLETypedCharacteristic<Vec3> rotation2RowCharacteristic = 
            BLETypedCharacteristic<Vec3>(rotation2RowCharacteristicUUID, BLERead | BLEWrite);
        BLETypedCharacteristic<Vec3> rotation3RowCharacteristic = 
            BLETypedCharacteristic<Vec3>(rotation3RowCharacteristicUUID, BLERead | BLEWrite);
};