/**
 * @file pn532_ble.cpp
 * @author whywilson (https://github.com/whywilson)
 * @brief ESP PN532BLE
 * @version 0.0.1
 * @date 2024-11-06
 */

#include "pn532_ble.h"

uint8_t dcs(uint8_t *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }
    return (0x00 - checksum) & 0xFF;
}

void appendCrcA(uint8_t *data, size_t length)
{
    uint16_t crc = 0x6363; // Initial value for CRC-A

    for (size_t i = 0; i < length; i++)
    {
        uint8_t ch = data[i] ^ (crc & 0xFF);
        ch = (ch ^ (ch << 4)) & 0xFF;
        crc = (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4);
    }

    crc &= 0xFFFF;
    data[length] = crc & 0xFF;
    data[length + 1] = crc >> 8;
}

PN532_BLE::PN532_BLE(bool debug) { _debug = debug; }

PN532_BLE::~PN532_BLE()
{
    if (NimBLEDevice::getInitialized())
    {
        NimBLEDevice::deinit(true);
    }
}

std::vector<PN532_BLE::CmdResponse> pn532Responses;
PN532_BLE::CmdResponse rsp;

class scanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice *advertisedDevice)
    {
        if (advertisedDevice->getName().find("PN532") != std::string::npos &&
            advertisedDevice->getName().find("BLE") != std::string::npos)
        {
            NimBLEDevice::getScan()->stop();
        }
    }
};

bool isCompleteFrame(uint8_t *pData, size_t length)
{
    if (length < 10)
    {
        return false;
    }

    uint8_t len = pData[9];
    uint8_t lcs = pData[10];

    if ((len + lcs) & 0xFF != 0x00)
    {
        Serial.println("Length checksum failed");
        return false;
    }

    uint8_t dcsValue = pData[length - 2];

    uint8_t calculatedDcs = dcs(pData + 11, len);
    if (calculatedDcs != dcsValue)
    {
        Serial.println("Invalid data checksum");
        return false;
    }

    return true;
}

void pn532NotifyCallBack(NimBLERemoteCharacteristic *pRemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    Serial.print("pn532NotifyCallBack: ");
    Serial.println(length);
    Serial.println();

    memcpy(&rsp.raw[rsp.length], pData, length);
    rsp.length += length;

    Serial.println("-----------------------");
    Serial.println();
    for (int i = 0; i < rsp.length; i++)
    {
        Serial.print(" ");
        Serial.print(rsp.raw[i], HEX);
    }
    Serial.println();
    Serial.println("-----------------------");

    if (!isCompleteFrame(rsp.raw, rsp.length))
    {
        Serial.println("Invalid frame");
        return;
    }

    // Serial.print("RSP length: ");
    // Serial.println(rsp.length);
    uint8_t dataSize = rsp.raw[9];
    rsp.dataSize = dataSize - 2;
    rsp.command = rsp.raw[12] - 1;

    if (rsp.dataSize > 0)
    {
        memcpy(rsp.data, rsp.raw + 13, rsp.raw[9] - 2);
    }

    // Serial.println("Response Cmd: ");
    // Serial.println(rsp.command, HEX);
    // Serial.println();
    // Serial.println("Response data size: ");
    // Serial.println(rsp.dataSize);
    // Serial.println();
    // Serial.println("RSP Data: ");
    for (int i = 0; i < rsp.dataSize; i++)
    {
        Serial.print(rsp.data[i] < 0x10 ? " 0" : " ");
        Serial.print(rsp.data[i], HEX);
    }
    Serial.println();
    pn532Responses.push_back(rsp);
    rsp.length = 0;
}

bool PN532_BLE::searchForDevice()
{
    if (_debug)
        Serial.println("Searching for PN532 BLE device...");
    NimBLEDevice::init("");
    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new scanCallbacks());
    pScan->setActiveScan(true);
    if (_debug)
        Serial.println("Start scanning...");
    BLEScanResults foundDevices = pScan->start(5);
    if (_debug)
        Serial.printf("Scan done! Found %d devices.\n", foundDevices.getCount());
    for (int i = 0; i < foundDevices.getCount(); i++)
    {
        NimBLEAdvertisedDevice advertisedDevice = foundDevices.getDevice(i);
        if (advertisedDevice.getName().find("PN532") != std::string::npos &&
            advertisedDevice.getName().find("BLE") != std::string::npos)
        {
            _device = advertisedDevice;
            return true;
        }
    }
    return false;
}

bool PN532_BLE::isConnected()
{
    return chrWrite != nullptr && chrNotify != nullptr;
}

bool PN532_BLE::connectToDevice()
{
    NimBLEClient *pClient = NimBLEDevice::createClient();
    if (!pClient)
    {
        Serial.println("Failed to create client");
        return false;
    }

    if (!pClient->connect(&_device, false))
    {
        Serial.println("Failed to connect to device");
        return false;
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());

    delay(200);

    pSvc = pClient->getService(serviceUUID);
    if (!pSvc)
    {
        Serial.println("Service does not exist");
        return false;
    }

    chrWrite = pSvc->getCharacteristic(chrTxUUID);
    chrNotify = pSvc->getCharacteristic(chrRxUUID);

    if (!chrWrite || !chrNotify)
    {
        Serial.println("Characteristics do not exist");
        return false;
    }

    chrNotify->subscribe(true, pn532NotifyCallBack);

    return true;
}

bool PN532_BLE::writeCommand(Command cmd, uint8_t *data, size_t length)
{
    // reset rsp length for new task
    rsp.length = 0;

    if (data == nullptr)
    {
        data = new uint8_t[0];
    }

    std::vector<uint8_t> commands;
    commands.push_back(DATA_TIF_SEND);
    commands.push_back(cmd);
    commands.insert(commands.end(), data, data + length);

    std::vector<uint8_t> frame;
    frame.push_back(DATA_PREAMBLE);
    frame.insert(frame.end(), std::begin(DATA_START_CODE), std::end(DATA_START_CODE));

    uint8_t len = commands.size();
    uint8_t length_check_sum = (0x00 - len) & 0xFF;
    frame.push_back(len);
    frame.push_back(length_check_sum);
    frame.insert(frame.end(), commands.begin(), commands.end());

    uint8_t dcs_value = dcs(commands.data(), commands.size());
    frame.push_back(dcs_value);
    frame.push_back(DATA_POSTAMBLE);

    if (_debug)
    {
        Serial.print("PN532 Send: ");
        for (int i = 0; i < frame.size(); i++)
        {
            Serial.print(frame[i] < 0x10 ? " 0" : " ");
            Serial.print(frame[i], HEX);
        }
        Serial.println();
    }
    bool writeRes = chrWrite->writeValue(frame.data(), frame.size(), true);
    delay(10);

    bool res = checkResponse(uint8_t(cmd));
    return writeRes && res;
}

bool PN532_BLE::writeCommand(Command cmd, const std::vector<uint8_t>& data) {
    return writeCommand(cmd, const_cast<uint8_t*>(data.data()), data.size());
}

bool PN532_BLE::checkResponse(uint8_t cmd)
{
    Serial.print("  Checking response on: ");
    Serial.println(cmd, HEX);
    Serial.println();
    unsigned long startTime = millis();
    while (pn532Responses.empty())
    {
        if (millis() - startTime > 3000)
        {
            Serial.println();
            Serial.println("Timeout out");
            return false;
        }
        delay(10);
        Serial.print(".");
    }

    auto it = std::find_if(pn532Responses.begin(), pn532Responses.end(), [cmd](const CmdResponse &response)
                           { return response.command == cmd; });

    if (it != pn532Responses.end())
    {
        cmdResponse = *it;
    }
    else
    {
        if (_debug)
        {
            Serial.println("No matching response found.");
        }
        return false;
    }
    // cmdResponse = pn532Responses[0];
    if (_debug)
    {
        Serial.print("PN532 Response: ");
        for (int i = 0; i < cmdResponse.length; i++)
        {
            Serial.print(cmdResponse.raw[i] < 0x10 ? " 0" : " ");
            Serial.print(cmdResponse.raw[i], HEX);
        }
        Serial.println();
        // print response command, status, data size and data
        Serial.print("Response Command: ");
        Serial.println(cmdResponse.command, HEX);
        Serial.print("    Status: ");
        Serial.println(cmdResponse.status, HEX);
        Serial.print("    Size: ");
        Serial.println(cmdResponse.dataSize);
        Serial.print("    Data: ");
        for (int i = 0; i < cmdResponse.dataSize; i++)
        {
            Serial.print(cmdResponse.data[i] < 0x10 ? " 0" : " ");
            Serial.print(cmdResponse.data[i], HEX);
        }
        Serial.println();
    }

    bool success = true;

    if (success && cmdResponse.command == InListPassiveTarget)
    {
        hfTagData.size = cmdResponse.data[0];
        memcpy(hfTagData.uidByte, cmdResponse.data + 1, hfTagData.size);

        hfTagData.atqaByte[1] = cmdResponse.data[1 + hfTagData.size];
        hfTagData.atqaByte[0] = cmdResponse.data[2 + hfTagData.size];

        hfTagData.sak = cmdResponse.data[3 + hfTagData.size];
    }

    pn532Responses.clear();
    return success;
}

void PN532_BLE::writeData(const std::vector<uint8_t> &data)
{
    if (chrWrite)
    {
        chrWrite->writeValue(data.data(), data.size());
    }
}

void PN532_BLE::setDevice(NimBLEAdvertisedDevice device)
{
    _device = device;
}

bool PN532_BLE::setNormalMode()
{
    std::vector<uint8_t> wakeup = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    writeData(wakeup);
    return writeCommand(SAMConfiguration, {0x01});
}

bool PN532_BLE::getVersion()
{
    return writeCommand(GetFirmwareVersion);
}

bool PN532_BLE::hf14aScan()
{
    return writeCommand(InListPassiveTarget, {0x01, 0x00});
}

std::vector<uint8_t> PN532_BLE::sendData(std::vector<uint8_t> data, bool append_crc)
{
    if (append_crc)
    {
        appendCrcA(data.data(), data.size());
    }

    writeCommand(InCommunicateThru, data.data(), data.size());
    return std::vector<uint8_t>(cmdResponse.data, cmdResponse.data + cmdResponse.dataSize);
}

std::vector<uint8_t> PN532_BLE::send7bit(std::vector<uint8_t> data)
{
    writeCommand(WriteRegister, {0x63, 0x3D, 0x07});
    std::vector<uint8_t> responseData = sendData(data, false);
    writeCommand(WriteRegister, {0x63, 0x3D, 0x00});
    return responseData;
}

bool PN532_BLE::resetRegister()
{
    return writeCommand(WriteRegister, {0x63, 0x02, 0x00, 0x63, 0x03, 0x00});
}

bool PN532_BLE::halt()
{
    sendData({0x50, 0x00}, false);
    return true;
}

bool PN532_BLE::isGen1A(){
    halt();
    std::vector<uint8_t> unlock1 = send7bit({0x40});
    Serial.println("Unlock 1: ");
    for (int i = 0; i < unlock1.size(); i++)
    {
        Serial.print(unlock1[i] < 0x10 ? " 0" : " ");
        Serial.print(unlock1[i], HEX);
    }
    if (unlock1.size() == 2 && unlock1[1] == 0x0A) {
        std::vector<uint8_t> unlock2 = sendData({0x43}, false);
        Serial.println("Unlock 2: ");
        for (int i = 0; i < unlock2.size(); i++)
        {
            Serial.print(unlock2[i] < 0x10 ? " 0" : " ");
            Serial.print(unlock2[i], HEX);
        }
        if (unlock2.size() == 2 && unlock2[1] == 0x0A) {
            return true;
        }
    }
    return false;
}

bool PN532_BLE::isGen3(){
    return false;
}

bool PN532_BLE::isGen4(){
    return false;
}