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

void appendCrcA(std::vector<uint8_t> &data)
{
    uint16_t crc = 0x6363; // Initial value for CRC-A

    for (size_t i = 0; i < data.size(); i++)
    {
        uint8_t ch = data[i] ^ (crc & 0xFF);
        ch = (ch ^ (ch << 4)) & 0xFF;
        crc = (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4);
    }

    crc &= 0xFFFF;
    data.push_back(crc & 0xFF);
    data.push_back(crc >> 8);
}

String bytes2HexString(std::vector<uint8_t> *data, uint8_t dataSize)
{
    String hexString = "";
    for (size_t i = 0; i < dataSize; i++)
    {
        hexString += (*data)[i] < 0x10 ? " 0" : " ";
        hexString += String((*data)[i], HEX);
    }
    hexString.toUpperCase();
    return hexString;
}

std::vector<uint8_t> hexStringToUint8Array(const std::string &hexString)
{
    std::vector<uint8_t> result;
    if (hexString.length() % 2 != 0)
    {
        throw std::invalid_argument("Hex string length must be even.");
    }

    for (size_t i = 0; i < hexString.length(); i += 2)
    {
        std::string byteString = hexString.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteString, nullptr, 16));
        result.push_back(byte);
    }

    return result;
}

std::vector<uint8_t> pn532bleBuffer;

PN532_BLE::PN532_BLE(bool debug) { _debug = debug; }

PN532_BLE::~PN532_BLE()
{
    if (NimBLEDevice::getInitialized())
    {
        pn532bleBuffer.clear();
        NimBLEDevice::deinit(true);
    }
}

std::vector<PN532_BLE::CmdResponse> pn532Responses;

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

    Serial.print("Length: ");
    Serial.println(length);

    Serial.print("Data Length: ");
    Serial.println(len);

    if (len + 13 < length)
    {
        Serial.println("Invalid length");
        return false;
    }

    if ((len + lcs) & 0xFF != 0x00)
    {
        Serial.println("Length checksum failed");
        pn532bleBuffer.clear();
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
    // add data to buffer
    pn532bleBuffer.insert(pn532bleBuffer.end(), pData, pData + length);
    Serial.print("PN532 ->");
    for (int i = 0; i < pn532bleBuffer.size(); i++)
    {
        Serial.print(pn532bleBuffer[i] < 0x10 ? " 0" : " ");
        Serial.print(pn532bleBuffer[i], HEX);
    }
    Serial.println();

    if (!isCompleteFrame(pn532bleBuffer.data(), pn532bleBuffer.size()))
    {
        Serial.println("Invalid frame");
        return;
    }

    PN532_BLE::CmdResponse rsp;
    memcpy(rsp.raw, pn532bleBuffer.data(), pn532bleBuffer.size());
    rsp.length = pn532bleBuffer.size();
    rsp.command = rsp.raw[12] - 1;

    uint8_t dataSize = rsp.raw[9];
    rsp.dataSize = dataSize - 2;
    rsp.command = rsp.raw[12] - 1;

    if (rsp.dataSize > 0)
    {
        memcpy(rsp.data, rsp.raw + 13, rsp.dataSize);
    }

    pn532Responses.push_back(rsp);
    pn532bleBuffer.clear();
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

NimBLERemoteService *PN532_BLE::getService(NimBLEClient *pClient) {
    for (const auto &uuid : serviceUUIDs) {
        NimBLERemoteService *service = pClient->getService(uuid);
        if (service) {
            return service;
        }
    }
    return nullptr;
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

    pSvc = getService(pClient);
    if (!pSvc)
    {
        Serial.println("Service does not exist");
        return false;
    }

    auto characteristics = pSvc->getCharacteristics(true);
    Serial.println("Characteristics Size:");
    Serial.println(characteristics->size());

    for (auto &characteristic : *characteristics)
    {
        if (characteristic->canWrite())
        {
            chrWrite = characteristic;
            break;
        }
    }
    for (auto &characteristic : *characteristics)
    {
        if (characteristic->canNotify())
        {
            chrNotify = characteristic;
            break;
        }
    }

    if (!chrWrite)
    {
        Serial.println("Write characteristic does not exist");
        return false;
    }

    if (!chrNotify)
    {
        Serial.println("Notify characteristic does not exist");
        return false;
    }

    chrNotify->subscribe(true, pn532NotifyCallBack);
    return true;
}

bool PN532_BLE::writeCommand(Command cmd, uint8_t *data, size_t length)
{
    pn532bleBuffer.clear();

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

    Serial.print("PN532 <-");
    for (int i = 0; i < frame.size(); i++)
    {
        Serial.print(frame[i] < 0x10 ? " 0" : " ");
        Serial.print(frame[i], HEX);
    }
    Serial.println();
    bool writeRes = chrWrite->writeValue(frame.data(), frame.size(), true);
    delay(10);

    bool res = checkResponse(uint8_t(cmd));
    return writeRes && res;
}

bool PN532_BLE::writeCommand(Command cmd, const std::vector<uint8_t> &data)
{
    return writeCommand(cmd, const_cast<uint8_t *>(data.data()), data.size());
}

bool PN532_BLE::checkResponse(uint8_t cmd)
{
    unsigned long startTime = millis();
    while (pn532Responses.empty())
    {
        if (millis() - startTime > 4000)
        {
            Serial.println("Timeout out");
            return false;
        }
        delay(10);
        if (_debug)
        {
            Serial.print(".");
        }
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

    // if (success && cmdResponse.command == InListPassiveTarget)
    // {
    //     hfTagData.size = cmdResponse.data[0];
    //     memcpy(hfTagData.uidByte, cmdResponse.data + 1, hfTagData.size);

    //     hfTagData.atqaByte[1] = cmdResponse.data[1 + hfTagData.size];
    //     hfTagData.atqaByte[0] = cmdResponse.data[2 + hfTagData.size];

    //     hfTagData.sak = cmdResponse.data[3 + hfTagData.size];
    // }

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

String PN532_BLE::getTagType() {
    switch (hf14aTagInfo.sak) {
        case 0x09:
            return "MIFARE Mini";
        case 0x08:
        case 0x88:
            return "MIFARE 1K";
        case 0x18:
            return "MIFARE 4K";
        case 0x00:
            return "MIFARE Ultralight";
        default:
            return "Unknown";
    }
}

void PN532_BLE::wakeup()
{
    writeData({0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
}

bool PN532_BLE::setNormalMode()
{
    wakeup();
    return writeCommand(SAMConfiguration, {0x01});
}

bool PN532_BLE::getVersion()
{
    return writeCommand(GetFirmwareVersion);
}

PN532_BLE::Iso14aTagInfo PN532_BLE::hf14aScan()
{
    bool res = writeCommand(InListPassiveTarget, {0x01, 0x00});
    if (!res)
    {
        return PN532_BLE::Iso14aTagInfo();
    }
    u_int8_t *data = cmdResponse.data;
    u_int8_t dataSize = cmdResponse.dataSize;
    return parseHf14aScan(data, dataSize);
}

PN532_BLE::Iso14aTagInfo PN532_BLE::parseHf14aScan(uint8_t *data, uint8_t dataSize)
{
    hf14aTagInfo.atqa = {data[2], data[3]};
    hf14aTagInfo.sak = data[4];
    hf14aTagInfo.uidSize = data[5];
    hf14aTagInfo.uid.assign(data + 6, data + 6 + hf14aTagInfo.uidSize);
    hf14aTagInfo.uid_hex = "";
    for (size_t i = 0; i < hf14aTagInfo.uid.size(); i++)
    {
        hf14aTagInfo.uid_hex += hf14aTagInfo.uid[i] < 0x10 ? "0" : "";
        hf14aTagInfo.uid_hex += String(hf14aTagInfo.uid[i], HEX);
    }
    hf14aTagInfo.uid_hex.toUpperCase();
    hf14aTagInfo.atqa_hex = bytes2HexString(&hf14aTagInfo.atqa, 2);
    std::vector<uint8_t> sakVector = {hf14aTagInfo.sak};
    hf14aTagInfo.sak_hex = bytes2HexString(&sakVector, 1);
    hf14aTagInfo.type = getTagType();
    return hf14aTagInfo;
}

bool PN532_BLE::mfAuth(std::vector<uint8_t> uid, uint8_t block, uint8_t *key, bool useKeyA)
{
    std::vector<uint8_t> authData = {0x40, 0x01,};
    authData.push_back(useKeyA ? 0x60 : 0x61);
    authData.push_back(block);
    authData.insert(authData.end(), key, key + 6);
    uint8_t uidLength = uid.size();
    authData.insert(authData.end(), uid.end() - 4, uid.end());
    bool res = writeCommand(InDataExchange, authData);
    if (!res)
    {
        return false;
    }
    return cmdResponse.dataSize >= 1 && cmdResponse.data[0] == 0x00;
}

std::vector<uint8_t> PN532_BLE::sendData(std::vector<uint8_t> data, bool append_crc)
{
    if (append_crc)
    {
        appendCrcA(data);
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
    resetRegister();
    sendData({0x50, 0x00}, false);
    return true;
}

bool PN532_BLE::isGen1A()
{
    halt();
    std::vector<uint8_t> unlock1 = send7bit({0x40});
    if (unlock1.size() == 2 && unlock1[1] == 0x0A)
    {
        delay(10);
        Serial.println("Unlock1 success");
        std::vector<uint8_t> unlock2 = sendData({0x43}, false);
        if (unlock2.size() == 2 && unlock2[1] == 0x0A)
        {
            delay(10);
            Serial.println("Unlock2 success");
            return true;
        }
    }
    return false;
}

bool PN532_BLE::selectTag()
{
    PN532_BLE::Iso14aTagInfo tag_info = hf14aScan();
    halt();
    if (tag_info.uid.empty())
    {
        Serial.println("No tag found");
        return false;
    }
    size_t uid_length = tag_info.uid.size();
    if (_debug)
    {
        Serial.print("Found UID: ");
        Serial.println(tag_info.uid_hex);
    }

    std::vector<uint8_t> wupa_result = send7bit({0x52});
    if (_debug)
    {
        Serial.print("WUPA: ");
        Serial.println(bytes2HexString(&wupa_result, wupa_result.size()));
    }

    auto anti_coll_result = sendData({0x93, 0x20}, false);
    if (_debug)
    {
        Serial.print("Anticollision CL1: ");
        Serial.println(bytes2HexString(&anti_coll_result, anti_coll_result.size()));
    }

    if (anti_coll_result[0] != 0x00)
    {
        if (_debug)
        {
            Serial.println("Anticollision failed");
        }
        return false;
    }

    std::vector<uint8_t> anti_coll_data(anti_coll_result.begin() + 1, anti_coll_result.end());
    std::vector<uint8_t> select_data = {0x93, 0x70};
    select_data.insert(select_data.end(), anti_coll_data.begin(), anti_coll_data.end());
    auto select_result = sendData(select_data, true);
    if (_debug)
    {
        Serial.print("Select CL1: ");
        Serial.println(bytes2HexString(&select_result, select_result.size()));
    }

    if (uid_length == 4)
    {
        return select_result.size() > 1 && select_result[0] == 0x00;
    }
    else if (uid_length == 7)
    {
        auto anti_coll2_result = sendData({0x95, 0x20}, false);
        if (_debug)
        {
            Serial.print("Anticollision CL2: ");
            Serial.println(bytes2HexString(&anti_coll2_result, anti_coll2_result.size()));
        }
        if (anti_coll2_result[0] != 0x00)
        {
            if (_debug)
            {
                Serial.println("Anticollision CL2 failed");
            }
            return false;
        }
        std::vector<uint8_t> anti_coll2_data(anti_coll2_result.begin() + 1, anti_coll2_result.end());
        std::vector<uint8_t> select2_data = {0x95, 0x70};
        select2_data.insert(select2_data.end(), anti_coll2_data.begin(), anti_coll2_data.end());
        auto select2_result = sendData(select2_data, true);
        if (_debug)
        {
            Serial.print("Select CL2: ");
            Serial.println(bytes2HexString(&select2_result, select2_result.size()));
        }
        return select2_result.size() > 1 && select2_result[0] == 0x00;
    }
    return false;
}

bool PN532_BLE::isGen3()
{
    bool selected = selectTag();
    if (!selected)
    {
        return false;
    }
    std::vector<uint8_t> result = sendData({0x30, 0x00}, true);
    return result.size() >= 16;
}

bool PN532_BLE::isGen4(std::string pwd)
{
    bool selected = selectTag();
    if (!selected)
    {
        return false;
    }
    std::vector<uint8_t> auth_data = {0xCF};
    std::vector<uint8_t> pwd_bytes = hexStringToUint8Array(pwd);
    auth_data.insert(auth_data.end(), pwd_bytes.begin(), pwd_bytes.end());
    auth_data.push_back(0xC6);
    std::vector<uint8_t> result = sendData(auth_data, true);
    return result.size() >= 15;
}