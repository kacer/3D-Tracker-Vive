#include "bluetooth.h"


void charStationNumberWritten(BLECentral &central, BLECharacteristic &characteristic) {
    Bluetooth::getInstance()->stationNumberWritten(central, characteristic);
}

void charPositionWritten(BLECentral &central, BLECharacteristic &characteristic) {
    Bluetooth::getInstance()->positionWritten(central, characteristic);
}

void charRotationWritten(BLECentral &central, BLECharacteristic &characteristic) {
    Bluetooth::getInstance()->rotationWritten(central, characteristic);
}

void Bluetooth::printBytes(unsigned char* ptr, unsigned char size) {
    for(uint8_t i = 0; i < size; i++) {
        Serial.print(ptr[i]);
        Serial.print(" ");
    }
    Serial.println();
}


Bluetooth* Bluetooth::instance = 0;

Bluetooth* Bluetooth::getInstance(void) {
    if(instance == 0) {
        instance = new Bluetooth();
    }

    return instance;
}

void Bluetooth::setBaseStationGeometryCB(cb_baseStationGeometry pCallback) {
    stationCb = pCallback;
}

void Bluetooth::sendPosition(vec3d &position) {
    Vec3 coords = {position[0], position[1], position[2]};
    coordinatesCharacteristic.setValue(coords);
}

void Bluetooth::poll(void) {
    peripheral.poll();
}

void Bluetooth::setup(void) {
    // at the start zeros base station's values
    for(uint8_t b = 0; b < baseStationsCount; b++) {
        memset(baseStations[b].origin, 0, sizeof(vec3d));
        memset(baseStations[b].rotationMatrix, 0, sizeof(mat3Array));
    }

    peripheral.setLocalName("3D Tracker");
    peripheral.setAdvertisedServiceUuid(coordinatesService.uuid());

    peripheral.addAttribute(coordinatesService);
    peripheral.addAttribute(coordinatesCharacteristic);

    peripheral.addAttribute(baseStationGeometryService);
    peripheral.addAttribute(stationNumberCharacteristic);
    peripheral.addAttribute(positionCharacteristic);
    peripheral.addAttribute(rotation1RowCharacteristic);
    peripheral.addAttribute(rotation2RowCharacteristic);
    peripheral.addAttribute(rotation3RowCharacteristic);

    stationNumberCharacteristic.setEventHandler(BLEWritten, charStationNumberWritten); 
    positionCharacteristic.setEventHandler(BLEWritten, charPositionWritten);
    rotation1RowCharacteristic.setEventHandler(BLEWritten, charRotationWritten);
    rotation2RowCharacteristic.setEventHandler(BLEWritten, charRotationWritten);
    rotation3RowCharacteristic.setEventHandler(BLEWritten, charRotationWritten);

    peripheral.begin();
}

void Bluetooth::stationNumberWritten(BLECentral &central, BLECharacteristic &characteristic) {
    uint8_t value = characteristic.value()[0];
    
    if(value < baseStationsCount && characteristic.valueLength() == sizeof(uint8_t)) {
        // set the new written base station index
        baseStationIndex = value;

        Serial.print("Index: ");
        Serial.println(baseStationIndex);
        Serial.print("Position: ");
        printBytes((unsigned char*) baseStations[baseStationIndex].origin, sizeof(Vec3));
        Serial.print("Position values: ");
        Serial.print(baseStations[baseStationIndex].origin[0], 5);
        Serial.print(" ");
        Serial.print(baseStations[baseStationIndex].origin[1], 5);
        Serial.print(" ");
        Serial.print(baseStations[baseStationIndex].origin[2], 5);
        Serial.println();
        Serial.print("Rotation: ");
        printBytes((unsigned char*) baseStations[baseStationIndex].rotationMatrix, sizeof(Mat3));

        // update characteristic values
        setPositionCharValueByIndex();
        setRotationCharValueByIndex();
    } else {
        // restore the previous base station index
        stationNumberCharacteristic.setValue(baseStationIndex);
    }
}

void Bluetooth::positionWritten(BLECentral &central, BLECharacteristic &characteristic) {
    if(characteristic.valueLength() == sizeof(Vec3)) {
        // set the new written position to the base station
        Vec3 vec = positionCharacteristic.value();
        memcpy(baseStations[baseStationIndex].origin, &vec, sizeof(Vec3));
    } else {
        // restore the previous position from base station
        setPositionCharValueByIndex();
    }
}

void Bluetooth::rotationWritten(BLECentral &central, BLECharacteristic &characteristic) {
    if(characteristic.valueLength() != sizeof(Vec3)) {
        // restore the previous rotation from base station
        setRotationCharValueByIndex();

        return;
    }

    int rot1Row = memcmp(characteristic.uuid(), rotation1RowCharacteristic.uuid(), 16);
    int rot2Row = memcmp(characteristic.uuid(), rotation2RowCharacteristic.uuid(), 16);
    int rot3Row = memcmp(characteristic.uuid(), rotation3RowCharacteristic.uuid(), 16);

    if(rot1Row == 0) {
        Vec3 rot1 = rotation1RowCharacteristic.value();
        setRowToRotationMatrix(0, rot1);
    } else if(rot2Row == 0) {
        Vec3 rot2 = rotation2RowCharacteristic.value();
        setRowToRotationMatrix(1, rot2);
    } else if(rot3Row == 0) {
        Vec3 rot3 = rotation3RowCharacteristic.value();
        setRowToRotationMatrix(2, rot3);
    }

    // if base station's geometry is set then propagate new geometry
    for(uint8_t b = 0; b < baseStationsCount; b++) {
        if(!baseStations[b].isSet()) {
            return;
        }
    }
    stationCb(baseStations);
}

void Bluetooth::setPositionCharValueByIndex(void) {
    float32_t x = baseStations[baseStationIndex].origin[0];
    float32_t y = baseStations[baseStationIndex].origin[1];
    float32_t z = baseStations[baseStationIndex].origin[2];
    positionCharacteristic.setValue({x, y, z});
}

void Bluetooth::setRotationCharValueByIndex(void) {
    unsigned char* pRot = (unsigned char*) baseStations[baseStationIndex].rotationMatrix;
    Vec3 row1;
    Vec3 row2;
    Vec3 row3;
    memcpy(&row1, pRot, sizeof(Vec3));
    memcpy(&row2, pRot + sizeof(Vec3), sizeof(Vec3));
    memcpy(&row3, pRot + 2 * sizeof(Vec3), sizeof(Vec3));
    rotation1RowCharacteristic.setValue(row1);
    rotation2RowCharacteristic.setValue(row2);
    rotation3RowCharacteristic.setValue(row3);
}

void Bluetooth::setRowToRotationMatrix(uint8_t rowNum, Vec3 row) {
    unsigned char* pRot = (unsigned char*) baseStations[baseStationIndex].rotationMatrix;
    uint8_t j = 0;
    for(uint8_t i = rowNum * sizeof(Vec3); i < rowNum * sizeof(Vec3) + sizeof(Vec3); i++) {
        pRot[i] = ((unsigned char*) &row)[j++];
    }
}