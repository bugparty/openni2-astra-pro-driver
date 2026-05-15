#include "AstraDevice.h"
#include "AstraDepthStream.h"
#include "AstraIRStream.h"
#include "UsbDevice.h"
#include "FirmwareCmd.h"
#include "PacketParser.h"
#include "DepthProcessor.h"
#include "IrProcessor.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

AstraDevice::AstraDevice(const char* uri, oni::driver::DriverServices& driverServices)
    : m_driverServices(driverServices)
    , m_depthModes{}
    , m_irModes{}
    , m_sensorInfos{}
{
    strncpy(m_uri, uri, ONI_MAX_STR - 1);
    m_uri[ONI_MAX_STR - 1] = '\0';

    // Create subsystems
    m_usbDev = new UsbDevice();
    m_fwCmd = new FirmwareCmd(m_usbDev);

    // Create frame processors
    m_depthProc = new DepthProcessor();
    m_irProc = new IrProcessor();

    // Create packet parsers (wire up to processors later during stream start)
    m_depthParser = new PacketParser();
    m_irParser = new PacketParser();

    // Build sensor info arrays
    buildSensorInfos();
}

AstraDevice::~AstraDevice()
{
    stopAllStreams();

    delete m_irParser;
    m_irParser = nullptr;

    delete m_depthParser;
    m_depthParser = nullptr;

    delete m_irProc;
    m_irProc = nullptr;

    delete m_depthProc;
    m_depthProc = nullptr;

    delete m_fwCmd;
    m_fwCmd = nullptr;

    delete m_usbDev;
    m_usbDev = nullptr;
}

// ---------------------------------------------------------------------------
// DeviceBase: sensor info
// ---------------------------------------------------------------------------

OniStatus AstraDevice::getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors)
{
    if (!m_initialized && !initUsb()) {
        *numSensors = 0;
        return ONI_STATUS_ERROR;
    }

    *pSensorInfos = m_sensorInfos;
    *numSensors = 2;
    return ONI_STATUS_OK;
}

// ---------------------------------------------------------------------------
// DeviceBase: stream creation / destruction
// ---------------------------------------------------------------------------

oni::driver::StreamBase* AstraDevice::createStream(OniSensorType sensorType)
{
    if (!m_initialized && !initUsb()) {
        return nullptr;
    }

    if (sensorType == ONI_SENSOR_DEPTH) {
        auto* stream = new AstraDepthStream(this);
        m_depthStream = stream;
        return stream;
    } else if (sensorType == ONI_SENSOR_IR) {
        auto* stream = new AstraIRStream(this);
        m_irStream = stream;
        return stream;
    }

    return nullptr;
}

void AstraDevice::destroyStream(oni::driver::StreamBase* pStream)
{
    if (pStream == m_depthStream) {
        m_depthStream = nullptr;
    } else if (pStream == m_irStream) {
        m_irStream = nullptr;
    }
    delete pStream;
}

// ---------------------------------------------------------------------------
// DeviceBase: properties
// ---------------------------------------------------------------------------

OniStatus AstraDevice::setProperty(int propertyId, const void* data, int dataSize)
{
    if (!m_fwCmd) {
        return ONI_STATUS_ERROR;
    }

    switch (propertyId) {
    case OBEXTENSION_ID_LASER_EN:
        if (dataSize < 1) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setLaser(*static_cast<const uint8_t*>(data) != 0)) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    case OBEXTENSION_ID_LDP_EN:
        if (dataSize < 1) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setLdp(*static_cast<const uint8_t*>(data) != 0)) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    case OBEXTENSION_ID_IR_GAIN:
        if (dataSize < static_cast<int>(sizeof(int))) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setIRGain(*static_cast<const int*>(data))) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    case OBEXTENSION_ID_IR_EXP:
        if (dataSize < static_cast<int>(sizeof(int))) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setIRExposure(*static_cast<const int*>(data))) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniStatus AstraDevice::getProperty(int propertyId, void* data, int* pDataSize)
{
    if (!m_fwCmd) {
        return ONI_STATUS_ERROR;
    }

    switch (propertyId) {
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    case OBEXTENSION_ID_SERIALNUMBER: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        int serialLen = static_cast<int>(m_serialNumber.size()) + 1;
        if (*pDataSize < serialLen) {
            *pDataSize = serialLen;
            return ONI_STATUS_BAD_PARAMETER;
        }
        strncpy(static_cast<char*>(data), m_serialNumber.c_str(), serialLen);
        *pDataSize = serialLen;
        return ONI_STATUS_OK;
    }

    case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<int*>(data) = static_cast<int>(m_fwVersion);
        *pDataSize = sizeof(int);
        return ONI_STATUS_OK;
    }

    case OBEXTENSION_ID_LASER_EN: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < 1) {
            *pDataSize = 1;
            return ONI_STATUS_BAD_PARAMETER;
        }
        bool on = false;
        if (m_fwCmd->getLaserState(on)) {
            *static_cast<uint8_t*>(data) = on ? 1 : 0;
            *pDataSize = 1;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    case OBEXTENSION_ID_IR_GAIN: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        int gain = 0;
        if (m_fwCmd->getIRGain(gain)) {
            *static_cast<int*>(data) = gain;
            *pDataSize = sizeof(int);
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    case OBEXTENSION_ID_IR_EXP: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        int exposure = 0;
        if (m_fwCmd->getIRExposure(exposure)) {
            *static_cast<int*>(data) = exposure;
            *pDataSize = sizeof(int);
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    case OBEXTENSION_ID_CAM_PARAMS: {
        // Read camera intrinsics from flash address 0x10000
        // The OBCameraParams struct is 92 bytes
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        const int CAM_PARAMS_SIZE = 92;
        if (*pDataSize < CAM_PARAMS_SIZE) {
            *pDataSize = CAM_PARAMS_SIZE;
            return ONI_STATUS_BAD_PARAMETER;
        }
        // Read from flash 0x10000 in chunks (max 256 per read)
        uint8_t params[CAM_PARAMS_SIZE];
        int totalRead = 0;
        bool ok = true;
        while (totalRead < CAM_PARAMS_SIZE && ok) {
            int chunkSize = std::min(256, CAM_PARAMS_SIZE - totalRead);
            int bytesRead = 0;
            if (!m_fwCmd->readFlash(0x10000 + totalRead,
                                     static_cast<uint16_t>(chunkSize),
                                     params + totalRead, &bytesRead)) {
                ok = false;
            } else {
                totalRead += bytesRead;
            }
        }
        if (ok && totalRead >= CAM_PARAMS_SIZE) {
            memcpy(data, params, CAM_PARAMS_SIZE);
            *pDataSize = CAM_PARAMS_SIZE;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniBool AstraDevice::isPropertySupported(int propertyId)
{
    switch (propertyId) {
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION:
    case OBEXTENSION_ID_LASER_EN:
    case OBEXTENSION_ID_LDP_EN:
    case OBEXTENSION_ID_IR_GAIN:
    case OBEXTENSION_ID_IR_EXP:
    case OBEXTENSION_ID_CAM_PARAMS:
    case OBEXTENSION_ID_SERIALNUMBER:
        return TRUE;
    default:
        return FALSE;
    }
}

// ---------------------------------------------------------------------------
// USB initialization
// ---------------------------------------------------------------------------

bool AstraDevice::initUsb()
{
    if (m_initialized) {
        return true;
    }

    // 1. Open USB device
    if (!m_usbDev->open(m_uri)) {
        fprintf(stderr, "AstraDevice: failed to open USB device '%s'\n", m_uri);
        return false;
    }
    m_usbOpen = true;

    // 2. Initialize firmware (read version, serial)
    if (!initFirmware()) {
        fprintf(stderr, "AstraDevice: firmware init failed\n");
    }

    // 3. PrimeSense init sequence: GetMode -> KeepAlive -> SetMode(PS)
    uint16_t fwMode = 0;
    if (m_fwCmd->getMode(fwMode)) {
        // mode read OK
    }

    // Send keep-alive to verify device is responsive
    for (int i = 0; i < 3; i++) {
        if (m_fwCmd->keepAlive()) {
            break;
        }
    }

    // Switch firmware to PS streaming mode (mode 1)
    if (!m_fwCmd->setMode(1)) {
        fprintf(stderr, "AstraDevice: setMode(PS) failed\n");
    }

    // 4. Read camera parameters and compute S2D table
    readCameraParams();
    computeShiftToDepthTable();

    // 5. Pass S2D table and firmware version to depth processor
    m_depthProc->setShiftToDepthTable(m_s2dTable.data(),
                                       static_cast<int>(m_s2dTable.size()));
    // fwVersion is encoded as (major << 8) | minor (e.g. 0x0508 for 5.8)
    m_depthProc->setFirmwareVersion(static_cast<uint16_t>(m_fwVersion & 0xFFFF));

    // 6. Enable laser by default
    m_fwCmd->setLaser(true);

    // 7. Query device frequency (AlgorithmParams type 0x80)
    uint8_t freqBuf[16];
    int freqSize = sizeof(freqBuf);
    if (m_fwCmd->algorithmParams(FirmwareCmd::ALG_FREQUENCY, 0, 0, freqBuf, &freqSize)) {
        float deviceFreq = 0.0f;
        if (freqSize >= 4) {
            memcpy(&deviceFreq, freqBuf, 4);
        }
    }

    m_initialized = true;
    return true;
}

bool AstraDevice::initFirmware()
{
    if (!m_fwCmd->init()) {
        return false;
    }

    m_fwVersion = m_fwCmd->firmwareVersion();
    m_serialNumber = m_fwCmd->serialNumber();

    return true;
}

// ---------------------------------------------------------------------------
// Sensor info
// ---------------------------------------------------------------------------

void AstraDevice::buildSensorInfos()
{
    // Depth modes: 160x120@30, 320x240@30, 640x480@30, 1280x1024@7
    m_depthModes[0] = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 160, 120, 30};
    m_depthModes[1] = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 320, 240, 30};
    m_depthModes[2] = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 640, 480, 30};
    m_depthModes[3] = {ONI_PIXEL_FORMAT_DEPTH_100_UM, 1280, 1024, 7};

    m_sensorInfos[0].sensorType = ONI_SENSOR_DEPTH;
    m_sensorInfos[0].numSupportedVideoModes = NUM_DEPTH_MODES;
    m_sensorInfos[0].pSupportedVideoModes = m_depthModes;

    // IR modes: 320x240@30, 320x240@60, 640x480@30, 1280x1024@30
    m_irModes[0] = {ONI_PIXEL_FORMAT_GRAY16, 320, 240, 30};
    m_irModes[1] = {ONI_PIXEL_FORMAT_GRAY16, 320, 240, 60};
    m_irModes[2] = {ONI_PIXEL_FORMAT_GRAY16, 640, 480, 30};
    m_irModes[3] = {ONI_PIXEL_FORMAT_GRAY8, 1280, 1024, 30};

    m_sensorInfos[1].sensorType = ONI_SENSOR_IR;
    m_sensorInfos[1].numSupportedVideoModes = NUM_IR_MODES;
    m_sensorInfos[1].pSupportedVideoModes = m_irModes;
}

// ---------------------------------------------------------------------------
// Shift-to-depth LUT
// ---------------------------------------------------------------------------

void AstraDevice::computeShiftToDepthTable()
{
    // Compute shift-to-depth lookup table using PrimeSense triangulation formula.
    // Each shift value from the depth sensor maps to a distance in mm via:
    //   nConstShift_effective = (nParamCoeff * nConstShift_raw) / pixelSizeFactor
    //   refX   = (shift - nConstShift_effective) / nParamCoeff - 0.375
    //   metric = refX * (ZPPS * pixelSizeFactor)
    //   depth  = shiftScale * (metric * ZPD / (EmitterD - metric) + ZPD)
    //
    // pixelSizeFactor depends on resolution:
    //   SXGA (1280x1024) = 1, VGA (640x480) = 2
    //
    // Astra Pro VGA at nParamCoeff=4 gives 1022 valid entries (shift 3-1024).
    // With nParamCoeff=1 we only got 511 valid entries (shift 1-511), causing
    // ~35% zero pixels ("flower screen" / garbled display).

    const int maxShift = 2048;
    m_s2dTable.resize(maxShift, 0);

    const double ZPD          = 100.0;     // zero plane distance (mm)
    const double ZPPS          = 0.03;      // zero plane pixel size (mm, raw from flash)
    const double EmitterD     = 51.0;      // emitter-to-CMOS baseline (mm)
    const double nParamCoeff  = 4.0;       // parameter coefficient (from flash)
    const double nConstShift  = -1171.4247; // constant shift (raw, before *nParamCoeff)
    const double nShiftScale  = 1.0;
    const double minCutoff    = 300.0;      // minimum depth (mm)
    const double maxCutoff    = 10000.0;    // maximum depth (mm)

    // VGA (640x480) uses pixelSizeFactor=2
    const int pixelSizeFactor = 2;
    double dPlanePixelSize = ZPPS * pixelSizeFactor;
    double nConstShiftEffective = (nParamCoeff * nConstShift) / pixelSizeFactor;

    for (int s = 1; s < maxShift; s++) {
        double refX = (s - nConstShiftEffective) / nParamCoeff - 0.375;
        double metric = refX * dPlanePixelSize;
        if (metric < 0.0 || metric >= EmitterD)
            continue;
        double depth = nShiftScale * (metric * ZPD / (EmitterD - metric) + ZPD);
        if (depth >= minCutoff && depth < maxCutoff)
            m_s2dTable[s] = static_cast<uint16_t>(depth);
    }
}

bool AstraDevice::readCameraParams()
{
    // S2D conversion now uses the PrimeSense triangulation formula with
    // fitted parameters (see computeShiftToDepthTable). Camera params
    // from flash are not needed for the basic depth pipeline.
    return true;
}

// ---------------------------------------------------------------------------
// Resolution mapping
// ---------------------------------------------------------------------------

uint16_t AstraDevice::pixelToFwResolution(int width)
{
    // PrimeSense resolution enum: 0=QQVGA, 1=QVGA, 2=VGA, 3=SXGA
    // Astra Pro uses: 0=QVGA(160x120), 1=VGA(640x480), 2=SXGA(1280x1024)
    switch (width) {
    case 160:
    case 320:  return 0;  // QVGA family
    case 640:  return 1;  // VGA
    case 1280: return 2;  // SXGA
    default:   return 1;  // default VGA
    }
}

// ---------------------------------------------------------------------------
// Stream configuration
// ---------------------------------------------------------------------------

bool AstraDevice::configureDepthStream(int width, int height, int fps)
{
    if (!m_fwCmd) return false;

    uint16_t fwRes = pixelToFwResolution(width);
    m_depthProc->setResolution(width, height);

    // PrimeSense firmware depth format enum (XnIODepthFormats):
    //   0 = Uncompressed16, 1 = PSCompressed, 2 = Uncompressed10, 3 = Packed11
    // Our DepthProcessor uses a different internal enum:
    //   0 = PSCompressed, 1 = Packed11, 2 = Uncompressed16
    // Firmware format and decoder format must match.
    // Try Packed11 (firmware fmt 3) — PSCompressed (fmt 1) on this fw 0xe752
    // produces nibble streams dominated by 0x0 (mostly zero bytes), which the
    // PrimeSense PSCompressed decoder collapses to ~LUT[1]. Packed11 worked in
    // earlier sessions on the same hardware.
    m_fwCmd->setDepthFormat(3);     // 3 = XN_IO_DEPTH_FORMAT_PACKED_11_BIT
    m_depthProc->setDepthFormat(1);  // 1 = Packed11 (our internal enum)

    if (!m_fwCmd->setDepthResolution(fwRes)) {
        fprintf(stderr, "AstraDevice: setDepthResolution(%d) failed\n", fwRes);
        return false;
    }

    if (!m_fwCmd->setDepthFPS(static_cast<uint16_t>(fps))) {
        fprintf(stderr, "AstraDevice: setDepthFPS(%d) failed\n", fps);
        return false;
    }

    // Query blanking info (PrimeSense caches coefficients for later use)
    uint8_t blankingBuf[64];
    int blankingSize = sizeof(blankingBuf);
    if (!m_fwCmd->algorithmParams(FirmwareCmd::ALG_BLANKING, fwRes,
                                   static_cast<uint16_t>(fps), blankingBuf, &blankingSize)) {
        fprintf(stderr, "AstraDevice: AlgorithmParams(BLANKING, depth) failed\n");
    }

    return true;
}

bool AstraDevice::configureIRStream(int width, int height, int fps)
{
    if (!m_fwCmd) return false;

    uint16_t fwRes = pixelToFwResolution(width);
    m_irProc->setResolution(width, height);

    // PrimeSense IR stream start order (from XnSensorIRStream):
    // 1. SetParam: Stream0Mode = IR (done by caller before bulk read)
    // 2. Start USB read thread (done by caller before this function)
    // 3. ConfigureFirmware: IRResolution, IRFPS
    // 4. SetCmosConfig (AlgorithmParams BLANKING)

    if (!m_fwCmd->setIRResolution(fwRes)) {
        fprintf(stderr, "AstraDevice: setIRResolution(%d) failed\n", fwRes);
        return false;
    }

    if (!m_fwCmd->setIRFPS(static_cast<uint16_t>(fps))) {
        fprintf(stderr, "AstraDevice: setIRFPS(%d) failed\n", fps);
        return false;
    }

    // Query blanking info (PrimeSense caches coefficients for later use)
    uint8_t blankingBuf[64];
    int blankingSize = sizeof(blankingBuf);
    if (!m_fwCmd->algorithmParams(FirmwareCmd::ALG_BLANKING, fwRes,
                                   static_cast<uint16_t>(fps), blankingBuf, &blankingSize)) {
        fprintf(stderr, "AstraDevice: AlgorithmParams(BLANKING, IR) failed\n");
    }

    return true;
}

// ---------------------------------------------------------------------------
// USB bulk read management
// ---------------------------------------------------------------------------

bool AstraDevice::startDepthBulkRead()
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        return false;
    }

    // Wire up: bulk data -> depth parser -> depth processor
    m_depthParser->setCallback([this](const PacketHeader& header,
                                      const uint8_t* data,
                                      uint32_t dataOffset,
                                      uint32_t dataSize) {
        static int cbDiag = 0;
        if (cbDiag < 5) {
            cbDiag++;
            FILE* f = fopen("/tmp/depth_cb.log", "a");
            if (f) {
                fprintf(f, "depth_cb[%d]: type=0x%04x pktId=%u size=%u\n",
                        cbDiag, header.type, header.packetID, dataSize);
                fclose(f);
            }
        }
        m_depthProc->processPacket(header, data, dataOffset, dataSize);
    });

    // Notify the depth stream when a new frame is ready
    m_depthProc->setFrameReadyCallback([this]() {
        if (m_depthStream) {
            m_depthStream->onNewFrameReady();
        }
    });

    return m_usbDev->startBulkRead(DEPTH_BULK_EP, [this](uint8_t* data, int size) {
        m_depthParser->feed(data, static_cast<size_t>(size));
    });
}

void AstraDevice::stopDepthBulkRead()
{
    if (m_usbDev && m_usbDev->isOpen()) {
        m_usbDev->stopBulkRead(DEPTH_BULK_EP);
    }
    m_depthParser->reset();

    // Turn off depth stream in firmware
    if (m_fwCmd) {
        m_fwCmd->setStream1Mode(0);  // OFF
    }
}

bool AstraDevice::startIRBulkRead()
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        fprintf(stderr, "AstraDevice: startIRBulkRead: USB device not open\n");
        return false;
    }

    // Wire up: bulk data -> IR parser -> IR processor
    m_irParser->setCallback([this](const PacketHeader& header,
                                   const uint8_t* data,
                                   uint32_t dataOffset,
                                   uint32_t dataSize) {
        m_irProc->processPacket(header, data, dataOffset, dataSize);
    });

    // Notify the IR stream when a new frame is ready
    m_irProc->setFrameReadyCallback([this]() {
        if (m_irStream) {
            m_irStream->onNewFrameReady();
        }
    });

    bool ok = m_usbDev->startBulkRead(IR_BULK_EP, [this](uint8_t* data, int size) {
        m_irParser->feed(data, static_cast<size_t>(size));
    });
    return ok;
}

void AstraDevice::stopIRBulkRead()
{
    if (m_usbDev && m_usbDev->isOpen()) {
        m_usbDev->stopBulkRead(IR_BULK_EP);
    }
    m_irParser->reset();

    // Turn off IR stream in firmware
    if (m_fwCmd) {
        m_fwCmd->setStream0Mode(0);  // OFF
    }
}

void AstraDevice::stopAllStreams()
{
    if (m_depthStreaming.load()) {
        stopDepthBulkRead();
        m_depthStreaming.store(false);
    }
    if (m_irStreaming.load()) {
        stopIRBulkRead();
        m_irStreaming.store(false);
    }
}
