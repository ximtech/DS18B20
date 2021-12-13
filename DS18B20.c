#include "DS18B20.h"

#define INITIAL_SENSOR_COUNT       1
#define DS18B20_SCRATCHPAD_SIZE    9
#define DS18B20_UNIQUE_CODE_LENGTH 64

#define DS18B20_MASTER_TX_RESET_PULSE_US 480
#define DS18B20_TX_PRESENCE_PULSE_US     80
#define DS18B20_MASTER_RX_PULSE_US       400
#define DS18B20_12BIT_CONVERSION_DELAY   750

#define MATCH_ROM_COMMAND           0x55
#define SEARCH_ROM_COMMAND          0xF0
#define READ_SCRATCH_PAD_COMMAND    0xBE
#define WRITE_SCRATCH_PAD_COMMAND   0x4E
#define CONVERT_TEMPERATURE_COMMAND 0x44
#define RESOLUTION_12BIT_COMMAND    0x7F

#define BITS_IN_BYTE 8
#define TEMPERATURE_LSB_INDEX 0
#define TEMPERATURE_MSB_INDEX 1
#define MAX_TEMPERATURE_THRESHOLD 125
#define MIN_TEMPERATURE_THRESHOLD 55
#define TEMPERATURE_CONVERSION_COEFFICIENT 0.0625f
#define SIGN_BIT_POSITION 11

#define BIT_READ(value, bit) (((value) >> (bit)) & 0x01)
#define BIT_SET(value, bit) ((value) |= (1ULL << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(1ULL << (bit)))
#define BIT_WRITE(value, bit, bitValue) ((bitValue) ? BIT_SET((value), (bit)) : BIT_CLEAR((value), (bit)))

typedef struct AddressNavigationData {
    uint8_t lastDiscrepancy;
    uint64_t address;
    uint8_t lastZero;
} AddressNavigationData;

typedef struct AddressItem {
    GPIO_TypeDef *GPIOx;
    uint32_t pin;
    uint64_t address;
} AddressItem;

static Vector addressItemVector = NULL;
static uint8_t scratchPadDataArray[DS18B20_SCRATCHPAD_SIZE] = {[0 ... DS18B20_SCRATCHPAD_SIZE - 1] = 0};

static bool isSensorsForLineAlreadyDiscovered(GPIO_TypeDef *GPIOx, uint32_t pin);
static void searchDS18B20Sensors(GPIO_TypeDef *GPIOx, uint32_t pin);
static DS18B20Status searchForDeviceAddress(GPIO_TypeDef *GPIOx, uint32_t pin, AddressNavigationData *navigationData);
static AddressItem *createAddressItem(GPIO_TypeDef *GPIOx, uint32_t pin, uint64_t address);
static void assignSensorAddress(DS18B20Sensor *sensor);
static void selectDS18B20Sensor(DS18B20Sensor *sensor);

static DS18B20Status startDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin);
static void writeByteDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t byte);
static void writeBitDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t bit);

static void readScratchpadDS18B20(DS18B20Sensor *sensor);
static uint8_t readByteDS18B20(DS18B20Sensor *sensor);
static uint8_t readBitDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin);
static DS18B20Status validateCheckSum();


DS18B20Sensor *initDS18B20Sensor(GPIO_TypeDef *GPIOx, uint32_t pin) {
    DS18B20Sensor *sensor = malloc(sizeof(DS18B20Sensor));
    if (sensor == NULL) return NULL;
    initSingletonVector(&addressItemVector, INITIAL_SENSOR_COUNT);
    sensor->GPIOx = GPIOx;
    sensor->pin = pin;
    sensor->address = 0;
    sensor->status = DS18B20_OK;
    sensor->temperature = 0;

    if (!isSensorsForLineAlreadyDiscovered(GPIOx, pin)) {
        searchDS18B20Sensors(GPIOx, pin);
    }

    bool isSensorFound = isVectorNotEmpty(addressItemVector);
    if (!isSensorFound) {
        sensor->status = DS18B20_NOT_FOUND_ERROR;
        return sensor;
    }
    assignSensorAddress(sensor);
    selectDS18B20Sensor(sensor);

    if (sensor->status == DS18B20_OK) {
        writeByteDS18B20(sensor->GPIOx, sensor->pin, WRITE_SCRATCH_PAD_COMMAND);
        writeByteDS18B20(sensor->GPIOx, sensor->pin, MAX_TEMPERATURE_THRESHOLD);
        writeByteDS18B20(sensor->GPIOx, sensor->pin, MIN_TEMPERATURE_THRESHOLD);
        writeByteDS18B20(sensor->GPIOx, sensor->pin, RESOLUTION_12BIT_COMMAND);
    }
    return sensor;
}

void readTemperatureDS18B20(DS18B20Sensor *sensor) {
    selectDS18B20Sensor(sensor);
    if (sensor->status == DS18B20_OK) {
        writeByteDS18B20(sensor->GPIOx, sensor->pin, CONVERT_TEMPERATURE_COMMAND);
        delay_ms(DS18B20_12BIT_CONVERSION_DELAY);  // wait for conversion

        selectDS18B20Sensor(sensor);
        if (sensor->status == DS18B20_OK) {
            writeByteDS18B20(sensor->GPIOx, sensor->pin, READ_SCRATCH_PAD_COMMAND);

            readScratchpadDS18B20(sensor);
            if (sensor->status == DS18B20_OK) {
                uint8_t temperatureLSB = scratchPadDataArray[TEMPERATURE_LSB_INDEX];
                uint8_t temperatureMSB = scratchPadDataArray[TEMPERATURE_MSB_INDEX];
                uint16_t temperature = (temperatureMSB << 8) | temperatureLSB;
                sensor->temperature = (float) temperature * TEMPERATURE_CONVERSION_COEFFICIENT;

                bool isNegativeTemperature = (temperatureMSB & (1 << SIGN_BIT_POSITION));
                if (isNegativeTemperature) {
                    sensor->temperature *= -1;
                }
            }
        }
    }
}

void checkConnectionStatusDS18B20(DS18B20Sensor *sensor) {
    sensor->status = startDS18B20(sensor->GPIOx, sensor->pin);
    if (sensor->status != DS18B20_OK) return;
    writeByteDS18B20(sensor->GPIOx, sensor->pin, SEARCH_ROM_COMMAND);

    for (uint8_t bitPosition = 0; bitPosition < DS18B20_UNIQUE_CODE_LENGTH; bitPosition++) {
        uint8_t actualBit = readBitDS18B20(sensor->GPIOx, sensor->pin);
        uint8_t complementBit = readBitDS18B20(sensor->GPIOx, sensor->pin);

        bool isNoDevicesOnLine = actualBit == 1 && complementBit == 1;
        if (isNoDevicesOnLine) {
            sensor->status = DS18B20_NOT_FOUND_ERROR;
            return;
        }
        writeBitDS18B20(sensor->GPIOx, sensor->pin, BIT_READ(sensor->address, bitPosition));
    }
    sensor->status = DS18B20_OK;
}

void deleteDS18B20(DS18B20Sensor *sensor) {
    free(sensor);
}

static bool isSensorsForLineAlreadyDiscovered(GPIO_TypeDef *GPIOx, uint32_t pin) {
    for (uint16_t i = 0; i < getVectorSize(addressItemVector); i++) {
        AddressItem *addressItem = vectorGet(addressItemVector, i);
        if (addressItem->GPIOx == GPIOx && addressItem->pin == pin) {
            return true;
        }
    }
    return false;
}

static void searchDS18B20Sensors(GPIO_TypeDef *GPIOx, uint32_t pin) {
    bool isLastDevice = false;
    uint8_t lastDiscrepancy = 0;

    while (!isLastDevice) {
        DS18B20Status status = startDS18B20(GPIOx, pin);
        if (status != DS18B20_OK) return;
        writeByteDS18B20(GPIOx, pin, SEARCH_ROM_COMMAND);

        AddressNavigationData navigationData = { .lastDiscrepancy = lastDiscrepancy, .lastZero = 0, .address = 0};
        status = searchForDeviceAddress(GPIOx, pin, &navigationData);
        if (status == DS18B20_NOT_FOUND_ERROR) return;

        lastDiscrepancy = navigationData.lastZero;
        if (!lastDiscrepancy) {
            isLastDevice = true;
        }

        AddressItem *addressItem = createAddressItem(GPIOx, pin, navigationData.address);
        vectorAdd(addressItemVector, addressItem);
    }
}

static DS18B20Status searchForDeviceAddress(GPIO_TypeDef *GPIOx, uint32_t pin, AddressNavigationData *navigationData) {
    for (uint8_t bitNumber = 0, idBitNumber = 1; bitNumber < DS18B20_UNIQUE_CODE_LENGTH; bitNumber++) {
        uint8_t actualBit = readBitDS18B20(GPIOx, pin);
        uint8_t complementBit = readBitDS18B20(GPIOx, pin);

        bool isNoDevicesOnLine = actualBit == 1 && complementBit == 1;
        if (isNoDevicesOnLine) {
            return DS18B20_NOT_FOUND_ERROR;
        }

        bool isMultipleDevicesOnLine = actualBit == 0 && complementBit == 0;
        if (isMultipleDevicesOnLine) {
            if (idBitNumber > navigationData->lastDiscrepancy) {    // First device search
                BIT_WRITE(navigationData->address, bitNumber, 0);
                navigationData->lastZero = idBitNumber;
                writeBitDS18B20(GPIOx, pin, 0);

            } else if (idBitNumber == navigationData->lastDiscrepancy) {
                BIT_WRITE(navigationData->address, bitNumber, 1);
                writeBitDS18B20(GPIOx, pin, 1);

            } else if (idBitNumber < navigationData->lastDiscrepancy) {// if this discrepancy if before the Last Discrepancy
                uint8_t directionBit = BIT_READ(navigationData->address, idBitNumber);  //on a previous next then pick the same as last time
                BIT_WRITE(navigationData->address, bitNumber, BIT_READ(navigationData->address, idBitNumber));
                navigationData->lastZero = (directionBit == 0) ? idBitNumber : navigationData->lastZero;    // if 0 was picked then record its position in lastZero
                writeBitDS18B20(GPIOx, pin, directionBit);
            }

        } else {    // only one device left
            BIT_WRITE(navigationData->address, bitNumber, actualBit);   // save to address
            writeBitDS18B20(GPIOx, pin, actualBit); // write received bit to the line
        }
        idBitNumber++;
    }
    return DS18B20_OK;
}

static AddressItem *createAddressItem(GPIO_TypeDef *GPIOx, uint32_t pin, uint64_t address) {
    AddressItem *addressItem = malloc(sizeof(AddressItem));
    addressItem->GPIOx = GPIOx;
    addressItem->pin = pin;
    addressItem->address = address;
    return addressItem;
}

static void assignSensorAddress(DS18B20Sensor *sensor) {
    for (uint16_t i = 0; i < getVectorSize(addressItemVector); i++) {
        AddressItem *addressItem = vectorGet(addressItemVector, i);
        if (addressItem->GPIOx == sensor->GPIOx && addressItem->pin == sensor->pin) {
            sensor->address = addressItem->address;
            free(vectorRemoveAt(addressItemVector, i)); // remove from vector and then delete item, because one address per one device only
            break;
        }
    }
}

static void selectDS18B20Sensor(DS18B20Sensor *sensor) {
    sensor->status = startDS18B20(sensor->GPIOx, sensor->pin);
    if (sensor->status != DS18B20_OK) return;
    writeByteDS18B20(sensor->GPIOx, sensor->pin, MATCH_ROM_COMMAND);

    for (uint8_t i = 0; i < DS18B20_UNIQUE_CODE_LENGTH; i++) {
        uint8_t bit = BIT_READ(sensor->address, i);
        writeBitDS18B20(sensor->GPIOx, sensor->pin, bit);
    }
}

static DS18B20Status startDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin) {
    DS18B20Status responseStatus;
    LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_OUTPUT);    // set the pin as output
    LL_GPIO_ResetOutputPin(GPIOx, pin);                     // pull the pin low
    delay_us(DS18B20_MASTER_TX_RESET_PULSE_US);            // delay according to datasheet
    LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_INPUT);     // set the pin as input
    delay_us(DS18B20_TX_PRESENCE_PULSE_US);

    responseStatus = !LL_GPIO_IsInputPinSet(GPIOx, pin) ? DS18B20_OK : DS18B20_RESPONSE_ERROR;// if the pin is LOW i.e the presence pulse is detected
    delay_us(DS18B20_MASTER_RX_PULSE_US);  // 480 us delay totally.
    return responseStatus;
}

static void writeByteDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t byte) {
    for (uint8_t i = 0; i < BITS_IN_BYTE; i++) {
        writeBitDS18B20(GPIOx, pin, BIT_READ(byte, i));
    }
}

static void writeBitDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin, uint8_t bit) {
    if (bit) {   // write 1
        LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_OUTPUT);    // set the pin as output
        LL_GPIO_ResetOutputPin(GPIOx, pin);  // pull the pin LOW
        delay_us(1);
        LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_INPUT);
        delay_us(60);

    } else {    // write 0
        LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_OUTPUT);    // set the pin as output
        LL_GPIO_ResetOutputPin(GPIOx, pin);  // pull the pin LOW
        delay_us(60);
        LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_INPUT);
        delay_us(15);       //wait for pull up
    }
}

static void readScratchpadDS18B20(DS18B20Sensor *sensor) {
    for (int i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++) {
        scratchPadDataArray[i] = readByteDS18B20(sensor);
    }
    sensor->status = validateCheckSum();
}

static uint8_t readByteDS18B20(DS18B20Sensor *sensor) {
    uint8_t result = 0;
    for (uint8_t i = 0; i < BITS_IN_BYTE; i++) {
        uint8_t dataBit = readBitDS18B20(sensor->GPIOx, sensor->pin);
        result += (dataBit << i);
    }
    return result;
}

static uint8_t readBitDS18B20(GPIO_TypeDef *GPIOx, uint32_t pin) {
    LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_OUTPUT);    // set the pin as output
    LL_GPIO_ResetOutputPin(GPIOx, pin);  // pull the pin LOW
    delay_us(2);

    LL_GPIO_SetPinMode(GPIOx, pin, LL_GPIO_MODE_INPUT);
    delay_us(10);       // wait for pull up if the sensor do not write 0

    uint8_t bit = LL_GPIO_IsInputPinSet(GPIOx, pin);    // if set 1, else 0
    delay_us(50);   // wait for the remaining 50 us (50 + 10 = 60)
    return bit;
}

static DS18B20Status validateCheckSum() {
    uint8_t generatedCRC = generateCRC8(scratchPadDataArray, DS18B20_SCRATCHPAD_SIZE - 1);
    uint8_t receivedCRC = scratchPadDataArray[DS18B20_SCRATCHPAD_SIZE - 1];
    return (generatedCRC == receivedCRC) ? DS18B20_OK : DS18B20_CHECKSUM_ERROR;
}