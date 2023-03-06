/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "base/samples/Frame.hpp"
#include <base-logging/Logging.hpp>

using namespace std;
using namespace camera_lucid;
using namespace base::samples::frame;
using namespace GenICam;

struct RequeueImageFrame {
    Arena::IImage* image = nullptr;
    Arena::IDevice* device = nullptr;
    RequeueImageFrame()
    {
    }

    RequeueImageFrame(RequeueImageFrame const&) = delete;
    RequeueImageFrame& operator=(RequeueImageFrame const&) = delete;

    RequeueImageFrame(Arena::IImage* v_image, Arena::IDevice* v_device)
        : image(v_image)
        , device(v_device)
    {
    }
    ~RequeueImageFrame()
    {
        releaseBuffer();
    }
    void releaseBuffer()
    {
        if (device != nullptr) {
            device->RequeueBuffer(image);
            device = nullptr;
            image = nullptr;
        }
    }
    void reset(Arena::IImage* v_image, Arena::IDevice* v_device)
    {
        image = v_image;
        device = v_device;
    }
};

struct ArenaDevice {
    Arena::IDevice* device = nullptr;
    Arena::ISystem* system = nullptr;
    ArenaDevice(Arena::IDevice* v_device, Arena::ISystem& v_system)
        : device(v_device)
        , system(&v_system)
    {
    }

    ArenaDevice(ArenaDevice const&) = delete;
    ArenaDevice& operator=(ArenaDevice const&) = delete;

    ~ArenaDevice()
    {
        destroyDevice();
    }
    Arena::IDevice& operator*()
    {
        return *device;
    }

    Arena::IDevice* release()
    {
        Arena::IDevice* v_device = device;
        device = nullptr;
        return v_device;
    }

    void reset(Arena::IDevice* v_device, Arena::ISystem& v_system)
    {
        destroyDevice();
        device = v_device;
        system = &v_system;
    }

    void destroyDevice()
    {
        if (device != nullptr) {
            system->DestroyDevice(device);
            device = nullptr;
        }
    }
};

struct ArenaSystem {
    Arena::ISystem* system = nullptr;
    ArenaSystem()
    {
    }
    ArenaSystem(Arena::ISystem* v_system)
        : system(v_system)
    {
    }
    ~ArenaSystem()
    {
        closeSystem();
    }

    ArenaSystem(ArenaSystem const&) = delete;
    ArenaSystem& operator=(ArenaSystem const&) = delete;

    Arena::ISystem& operator*()
    {
        return *system;
    }

    Arena::ISystem* release()
    {
        Arena::ISystem* v_system = system;
        system = nullptr;
        return v_system;
    }
    void closeSystem()
    {
        if (system != nullptr) {
            Arena::CloseSystem(system);
            system = nullptr;
        }
    }
};

template <typename T> const char* getEnumName(T enum_type, vector<string> name)
{
    return name.at(enum_type).c_str();
}

Task::Task(string const& name)
    : TaskBase(name)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (!TaskBase::configureHook()) {
        return false;
    }
    try {
        ArenaSystem system(Arena::OpenSystem());
        ArenaDevice device(connectToCamera(*system), *system);
        switchOverAccess(*device); // get() returns a reference

        if (_camera_config.get().factory_reset) {
            LOG_INFO_S << "Performing Factory Reset" << endl;
            factoryReset(device.release(), *system);

            device.reset(connectToCamera(*system), *system);
            switchOverAccess(*device);
        }

        configureCamera(*device, *system);
        m_system = system.release(); // returns pointer
        m_device = device.release(); // returns pointer
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
        throw std::runtime_error(ge.what());
    }
    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) {
        return false;
    }

    LOG_INFO_S << "Starting frame acquisition." << endl;
    try {
        m_device->StartStream();
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
        throw std::runtime_error(ge.what());
    }
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    acquireFrame();
    collectInfo();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    try {
        m_device->StopStream();
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
        throw std::runtime_error(ge.what());
    }
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    try {
        m_system->DestroyDevice(m_device);
        Arena::CloseSystem(m_system);
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
        throw std::runtime_error(ge.what());
    }
    m_system = nullptr;
    m_device = nullptr;
}

Arena::IDevice* Task::connectToCamera(Arena::ISystem& system)
{
    LOG_INFO_S << "Looking for camera" << endl;
    m_ip = _camera_config.get().ip;
    system.UpdateDevices(100);
    vector<Arena::DeviceInfo> device_infos = system.GetDevices();
    for (auto device : device_infos) {
        if (m_ip.compare(device.IpAddressStr()) == 0) {
            LOG_INFO_S << "Connected to camera" << endl;
            return system.CreateDevice(device);
        }
    }
    string cameras;
    for (auto device : device_infos) {
        cameras.append(device.IpAddressStr());
        cameras.append(", ");
    }
    LOG_ERROR_S << "Camera " << _camera_config.get().ip
                << " not found. Found cameras: " << cameras << endl;
    throw runtime_error("Camera not found.");
}

void Task::switchOverAccess(Arena::IDevice& device)
{
    LOG_INFO_S << "Checking Camera Read/Write Access." << endl;
    // Retrieve DeviceAccessStatus
    GenICam::gcstring device_access_status =
        Arena::GetNodeValue<GenICam::gcstring>(device.GetTLDeviceNodeMap(),
            "DeviceAccessStatus");
    // Check DeviceAccessStatus to determine if we have ReadWrite status
    // If we have ReadOnly status, another application has control of the camera:
    if (device_access_status == "ReadWrite") {
        // Set a switchover key in case another application needs to take control
        Arena::SetNodeValue<int64_t>(device.GetTLDeviceNodeMap(),
            "CcpSwitchoverKey",
            m_switchover_key);
        LOG_INFO_S << "Task already has the Camera Read/Write Access." << endl;
        return;
    }

    // Apply the correct switchover key and change the DeviceAccessStatus to ReadWrite
    // to gain control If there is no switchover key, or an incorrect switchover key,
    // ReadWrite access will not be granted
    Arena::SetNodeValue<int64_t>(device.GetTLDeviceNodeMap(),
        "CcpSwitchoverKey",
        m_switchover_key);
    Arena::SetNodeValue<GenICam::gcstring>(device.GetTLDeviceNodeMap(),
        "DeviceAccessStatus",
        "ReadWrite");

    // confirm if operation was successfull
    device_access_status =
        Arena::GetNodeValue<GenICam::gcstring>(device.GetTLDeviceNodeMap(),
            "DeviceAccessStatus");
    if (device_access_status == "ReadWrite") {
        LOG_INFO_S << "Task retrieved the Camera Read/Write Access." << endl;
        return;
    }

    LOG_WARN_S << "Task failed to retrieved the Camera Read/Write Access." << endl;
    throw runtime_error("Task failed to retrieved the Camera Read/Write Access.");
}

void Task::configureCamera(Arena::IDevice& device, Arena::ISystem& system)
{
    LOG_INFO_S << "Configuring camera." << endl;
    acquisitionConfiguration(device);

    LOG_INFO_S << "Setting StreamBufferHandlingMode." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(device.GetTLStreamNodeMap(),
        "StreamBufferHandlingMode",
        "NewestOnly");

    LOG_INFO_S << "Setting StreamAutoNegotiatePacketSize." << endl;
    Arena::SetNodeValue<bool>(device.GetTLStreamNodeMap(),
        "StreamAutoNegotiatePacketSize",
        true);

    LOG_INFO_S << "Setting StreamPacketResendEnable." << endl;
    Arena::SetNodeValue<bool>(device.GetTLStreamNodeMap(),
        "StreamPacketResendEnable",
        true);

    LOG_INFO_S << "Setting PixelFormat." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(device.GetNodeMap(),
        "PixelFormat",
        convertFrameModeToPixelFormat(_image_config.get().format,
            _image_config.get().depth)
            .c_str());

    // When horizontal binning is used, horizontal decimation (if supported) is not
    // available. When vertical binning is used, vertical decimation (if supported) is
    // not available.

    // If the camera is acquiring images, AcquisitionStop must be called before
    // adjusting binning settings.
    binningConfiguration(device);
    decimationConfiguration(device);
    dimensionsConfiguration(device);
    exposureConfiguration(device);
    analogConfiguration(device);
    // infoConfiguration(device);

    Frame* frame = new Frame(_image_config.get().width,
        _image_config.get().height,
        _image_config.get().depth,
        _image_config.get().format,
        0,
        0);
    m_frame.reset(frame);
}

void Task::acquireFrame()
{
    RequeueImageFrame frame;
    try {
        frame.reset(
            m_device->GetImage(_image_config.get().frame_timeout.toMilliseconds()),
            m_device);
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
        throw runtime_error(ge.what());
    }

    if (frame.image->IsIncomplete()) {
        LOG_ERROR_S << "Image is not complete!! " << endl;
        return;
    }

    size_t size = frame.image->GetSizeFilled();
    size_t width = frame.image->GetWidth();
    size_t height = frame.image->GetHeight();
    PfncFormat pixel_format = static_cast<PfncFormat>(frame.image->GetPixelFormat());
    uint8_t data_depth = 0U;
    auto format = convertPixelFormatToFrameMode(pixel_format, data_depth);

    Frame* out_frame = m_frame.write_access();
    if (width != out_frame->getWidth() || height != out_frame->getHeight() ||
        format != out_frame->getFrameMode()) {
        out_frame->init(width, height, data_depth, format, 0, size);
    }
    out_frame->time = base::Time::now();
    out_frame->received_time = out_frame->time;

    out_frame->setImage(frame.image->GetData(), size);
    out_frame->setStatus(STATUS_VALID);
    m_frame.reset(out_frame);
    _frame.write(m_frame);
}

frame_mode_t Task::convertPixelFormatToFrameMode(PfncFormat format, uint8_t& data_depth)
{
    switch (format) {
        case PfncFormat_::Mono8:
            data_depth = 8U;
            return frame_mode_t::MODE_GRAYSCALE;

        case PfncFormat_::RGB8:
            data_depth = 8U;
            return frame_mode_t::MODE_RGB;

        case PfncFormat_::YUV8_UYV:
            data_depth = 8U;
            return frame_mode_t::MODE_UYVY;

        case PfncFormat_::BGR8:
            data_depth = 8U;
            return frame_mode_t::MODE_BGR;

        case PfncFormat_::RGB10p32:
            data_depth = 10U;
            return frame_mode_t::MODE_RGB32;

        case PfncFormat_::BayerRG8:
            data_depth = 8U;
            return frame_mode_t::MODE_BAYER_RGGB;

        case PfncFormat_::BayerGR8:
            data_depth = 8U;
            return frame_mode_t::MODE_BAYER_GRBG;

        case PfncFormat_::BayerBG8:
            data_depth = 8U;
            return frame_mode_t::MODE_BAYER_BGGR;

        case PfncFormat_::BayerGB8:
            data_depth = 8U;
            return frame_mode_t::MODE_BAYER_GBRG;

        default:
            return frame_mode_t::MODE_UNDEFINED;
    }
}

string Task::convertFrameModeToPixelFormat(frame_mode_t format, uint8_t data_depth)
{
    switch (format) {
        case frame_mode_t::MODE_GRAYSCALE:
            return "Mono" + to_string(data_depth);

        case frame_mode_t::MODE_RGB:
            return "RGB" + to_string(data_depth);

        case frame_mode_t::MODE_UYVY:
            return "YUV" + to_string(data_depth) + "_UYV";

        case frame_mode_t::MODE_BGR:
            return "BGR" + to_string(data_depth);

        case frame_mode_t::MODE_RGB32:
            return "RGB" + to_string(data_depth) + "p32";

        case frame_mode_t::MODE_BAYER_RGGB:
            return "BayerRG" + to_string(data_depth);

        case frame_mode_t::MODE_BAYER_GRBG:
            return "BayerGR" + to_string(data_depth);

        case frame_mode_t::MODE_BAYER_BGGR:
            return "BayerBG" + to_string(data_depth);

        case frame_mode_t::MODE_BAYER_GBRG:
            return "BayerGB" + to_string(data_depth);

        default:
            return "";
    }
}

void Task::binningConfiguration(Arena::IDevice& device)
{
    // Initial check if sensor binning is supported.
    //    Entry may not be in XML file. Entry may be in the file but set to
    //    unreadable or unavailable. Note: there is a case where sensor
    //    binning is not supported but this test passes However, we must set
    //    BinningSelector to Sensor before we can test for that.
    GenApi::CEnumerationPtr binning_selector_node =
        device.GetNodeMap()->GetNode("BinningSelector");
    GenApi::CEnumEntryPtr binning_sensor_entry =
        binning_selector_node->GetEntryByName("Digital");
    if (binning_sensor_entry == 0 || !GenApi::IsAvailable(binning_sensor_entry)) {
        LOG_WARN_S << "Sensor binning not supported by device: not available from "
                      "BinningSelector"
                   << endl;
        throw runtime_error("Sensor binning not supported by device: not available "
                            "from BinningSelector");
    }

    LOG_INFO_S << "Set binning mode to "
               << getEnumName(_binning_config.get().selector, binning_selector_name)
               << endl;
    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "BinningSelector",
        getEnumName(_binning_config.get().selector, binning_selector_name));

    // Check if the nodes for the height and width of the bin are available
    //    For rare case where sensor binning is unsupported but still appears as
    //    an option. Must be done after setting BinningSelector to Sensor. It was
    //    probably just a bug in the firmware.
    if (!GenApi::IsAvailable(device.GetNodeMap()->GetNode("BinningVertical")) ||
        !GenApi::IsAvailable(device.GetNodeMap()->GetNode("BinningVertical"))) {
        LOG_WARN_S << "Sensor binning not supported by device: BinningVertical or "
                      "BinningHorizontal not available."
                   << endl;
        throw runtime_error("Sensor binning not supported by device: BinningVertical or "
                            "BinningHorizontal not available.");
    }

    // Find max for bin height & width.
    //    For maximum compression.
    GenApi::CIntegerPtr binning_vertical_node =
        device.GetNodeMap()->GetNode("BinningVertical");
    GenApi::CIntegerPtr binning_horizontal_node =
        device.GetNodeMap()->GetNode("BinningHorizontal");

    int64_t max_binning_height = binning_vertical_node->GetMax();
    int64_t max_binning_width = binning_horizontal_node->GetMax();

    if (_binning_config.get().binning_x > max_binning_height) {
        LOG_WARN_S << "Binning_x is bigger than the maximum allowed value! Setpoint: "
                   << _binning_config.get().binning_x
                   << ". Maximum: " << max_binning_height << endl;
        throw runtime_error("Binning_x is bigger than the maximum allowed value.");
    }
    else if (_binning_config.get().binning_y > max_binning_width) {
        LOG_WARN_S << "Binning_y is bigger than the maximum allowed value! Setpoint: "
                   << _binning_config.get().binning_y
                   << ". Maximum: " << max_binning_width << endl;
        throw runtime_error("Binning_y is bigger than the maximum allowed value.");
    }

    Arena::SetNodeValue<int64_t>(device.GetNodeMap(),
        "BinningVertical",
        _binning_config.get().binning_x);

    Arena::SetNodeValue<int64_t>(device.GetNodeMap(),
        "BinningHorizontal",
        _binning_config.get().binning_y);

    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "BinningVerticalMode",
        getEnumName(_binning_config.get().vertical_mode, binning_mode_name));

    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "BinningHorizontalMode",
        getEnumName(_binning_config.get().horizontal_mode, binning_mode_name));

    if (_binning_config.get().binning_y !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "BinningVertical") ||
        _binning_config.get().binning_x !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "BinningHorizontal")) {
        LOG_WARN_S << "Failed setting binning parameters.";
        throw runtime_error("Failed setting binning parameters.");
    }
}

void Task::decimationConfiguration(Arena::IDevice& device)
{

    // Initial check if sensor decimation is supported.
    //    Entry may not be in XML file. Entry may be in the file but set to
    //    unreadable or unavailable. Note: there is a case where sensor
    //    decimation is not supported but this test passes However, we must set
    //    DecimationSelector to Sensor before we can test for that.
    GenApi::CEnumerationPtr decimation_selector_node =
        device.GetNodeMap()->GetNode("DecimationSelector");
    GenApi::CEnumEntryPtr decimation_sensor_entry =
        decimation_selector_node->GetEntryByName(
            getEnumName(_decimation_config.get().selector, decimation_selector_name));
    if (decimation_sensor_entry == 0 || !GenApi::IsAvailable(decimation_sensor_entry)) {
        LOG_WARN_S << "Sensor decimation not supported by device: not available from "
                      "DecimationSelector."
                   << endl;
        throw runtime_error("Sensor decimation not supported by device: not available "
                            "from DecimationSelector.");
    }

    LOG_INFO_S << "Set decimation mode to: "
               << getEnumName(_decimation_config.get().selector, decimation_selector_name)
               << endl;
    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "DecimationSelector",
        getEnumName(_decimation_config.get().selector, decimation_selector_name));

    // Check if the nodes for the height and width of the bin are available
    //    For rare case where sensor decimation is unsupported but still appears as
    //    an option. Must be done after setting DecimationSelector to Sensor. It was
    //    probably just a bug in the firmware.
    if (!GenApi::IsAvailable(device.GetNodeMap()->GetNode("DecimationVertical")) ||
        !GenApi::IsAvailable(device.GetNodeMap()->GetNode("DecimationVertical"))) {
        LOG_WARN_S << "Sensor decimation not supported by device: DecimationVertical or "
                      "DecimationHorizontal not available."
                   << endl;
        throw runtime_error("Sensor decimation not supported by device: "
                            "DecimationVertical or DecimationHorizontal not available.");
    }

    GenApi::CIntegerPtr decimation_vertical_node =
        device.GetNodeMap()->GetNode("DecimationVertical");
    GenApi::CIntegerPtr decimation_horizontal_node =
        device.GetNodeMap()->GetNode("DecimationHorizontal");

    int64_t max_decimation_height = decimation_vertical_node->GetMax();
    int64_t max_decimation_width = decimation_horizontal_node->GetMax();

    if (_decimation_config.get().decimation_x > max_decimation_height) {
        LOG_WARN_S << "Decimation_x is bigger than the maximum allowed value! Setpoint: "
                   << _decimation_config.get().decimation_x
                   << ". Maximum: " << max_decimation_height << endl;
        throw runtime_error("Decimation_x is bigger than the maximum allowed value");
    }
    else if (_decimation_config.get().decimation_y > max_decimation_width) {
        LOG_WARN_S << "Decimation_y is bigger than the maximum allowed value! Setpoint: "
                   << _decimation_config.get().decimation_y
                   << ". Maximum: " << max_decimation_width << endl;
        throw runtime_error("Decimation_y is bigger than the maximum allowed value.");
    }

    Arena::SetNodeValue<int64_t>(device.GetNodeMap(),
        "DecimationVertical",
        _decimation_config.get().decimation_x);

    Arena::SetNodeValue<int64_t>(device.GetNodeMap(),
        "DecimationHorizontal",
        _decimation_config.get().decimation_y);

    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "DecimationVerticalMode",
        getEnumName(_decimation_config.get().vertical_mode, decimation_mode_name));

    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "DecimationHorizontalMode",
        getEnumName(_decimation_config.get().horizontal_mode, decimation_mode_name));

    if (_decimation_config.get().decimation_y !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "DecimationVertical") ||
        _decimation_config.get().decimation_x !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "DecimationHorizontal")) {
        LOG_WARN_S << "Failed setting decimation parameters.";
        throw runtime_error("Failed setting decimation parameters.");
    }
}

void Task::dimensionsConfiguration(Arena::IDevice& device)
{
    LOG_INFO_S << "Setting Dimensions." << endl;
    GenApi::CIntegerPtr width = device.GetNodeMap()->GetNode("Width");
    GenApi::CIntegerPtr height = device.GetNodeMap()->GetNode("Height");
    GenApi::CIntegerPtr offset_x = device.GetNodeMap()->GetNode("OffsetX");
    GenApi::CIntegerPtr offset_y = device.GetNodeMap()->GetNode("OffsetY");

    if (!width || !GenApi::IsReadable(width) || !GenApi::IsWritable(width)) {
        LOG_ERROR_S << "Width node not found/readable/writable." << endl;
        throw runtime_error("Width node not found/readable/writable.");
    }

    if (!height || !GenApi::IsReadable(height) || !GenApi::IsWritable(height)) {
        LOG_ERROR_S << "Height node not found/readable/writable." << endl;
        throw runtime_error("Height node not found/readable/writable.");
    }

    if (!offset_x || !GenApi::IsReadable(offset_x) || !GenApi::IsWritable(offset_x)) {
        LOG_ERROR_S << "Offset X node not found/readable/writable." << endl;
        throw runtime_error("Offset X node not found/readable/writable.");
    }

    if (!offset_y || !GenApi::IsReadable(offset_y) || !GenApi::IsWritable(offset_y)) {
        LOG_ERROR_S << "Offset Y node not found/readable/writable." << endl;
        throw runtime_error("Offset Y node not found/readable/writable.");
    }

    LOG_INFO_S << "Setting Width." << endl;
    width->SetValue(_image_config.get().width);
    LOG_INFO_S << "Setting Height." << endl;
    height->SetValue(_image_config.get().height);
    LOG_INFO_S << "Setting Offset X." << endl;
    offset_x->SetValue(_image_config.get().offset_x);
    LOG_INFO_S << "Setting Offset Y." << endl;
    offset_y->SetValue(_image_config.get().offset_y);

    if (_image_config.get().width !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "Width") ||
        _image_config.get().height !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "Height") ||
        _image_config.get().offset_x !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "OffsetX") ||
        _image_config.get().offset_y !=
            Arena::GetNodeValue<int64_t>(device.GetNodeMap(), "OffsetY")) {
        throw runtime_error("Width/Heigth/Offset not properly configured.");
    }
}

void Task::exposureConfiguration(Arena::IDevice& device)
{
    LOG_INFO_S << "Setting auto exposure to "
               << getEnumName(_image_config.get().exposure_auto, exposure_auto_name)
               << endl;
    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "ExposureAuto",
        getEnumName(_image_config.get().exposure_auto, exposure_auto_name));

    GenApi::CEnumerationPtr exposure_auto = device.GetNodeMap()->GetNode("ExposureAuto");

    auto current = exposure_auto->GetCurrentEntry()->GetSymbolic();
    if (current != getEnumName(_image_config.get().exposure_auto, exposure_auto_name)) {
        LOG_WARN_S << "ExposureAuto value differs setpoint: "
                   << getEnumName(_image_config.get().exposure_auto, exposure_auto_name)
                   << " current: " << current << endl;
        throw runtime_error("ExposureAuto value differs.");
    }

    if (_image_config.get().exposure_auto == ExposureAuto::EXPOSURE_AUTO_OFF) {
        LOG_INFO_S << "Setting exposure time to: "
                   << _image_config.get().exposure_time.toMicroseconds() << "us" << endl;
        GenApi::CFloatPtr exposure_time = device.GetNodeMap()->GetNode("ExposureTime");

        auto max_exposure_time = exposure_time->GetMax();
        auto min_exposure_time = exposure_time->GetMin();
        auto setpoint = _image_config.get().exposure_time.toMicroseconds();

        if (setpoint > max_exposure_time) {
            LOG_WARN_S << "Exposure time exceeds maximum value. Setting Exposure to "
                          "maximum value: "
                       << max_exposure_time << "us" << endl;
            setpoint = max_exposure_time;
        }
        else if (setpoint < min_exposure_time) {
            LOG_WARN_S << "Exposure time exceeds minimum value. Setting Exposure to "
                          "minimum value: "
                       << min_exposure_time << "us" << endl;
            setpoint = min_exposure_time;
        }
        exposure_time->SetValue(static_cast<double>(setpoint));

        auto current = exposure_time->GetValue();
        if (abs(current - setpoint) >= 10) {
            LOG_WARN_S << "Exposure Time value differs from setpoint: " << setpoint
                       << " current: " << current << endl;
            throw runtime_error("Exposure Time value differs.");
        }
    }
}

void Task::acquisitionConfiguration(Arena::IDevice& device)
{
    LOG_INFO_S << "Configuring Acquisition." << endl;

    LOG_INFO_S << "Setting Acquisition Mode." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(device.GetNodeMap(),
        "AcquisitionMode",
        "Continuous");

    LOG_INFO_S << "Enable Acquisition Frame Rate." << endl;
    // Enable Acquisition Frame Rate
    Arena::SetNodeValue<bool>(device.GetNodeMap(), "AcquisitionFrameRateEnable", true);

    // get Acquisition Frame Rate node
    GenApi::CFloatPtr acquisition_frame_rate =
        device.GetNodeMap()->GetNode("AcquisitionFrameRate");
    acquisition_frame_rate->SetValue(_image_config.get().frame_rate);
    LOG_INFO_S << "Setting Acquisition Frame Rate to " << _image_config.get().frame_rate
               << " FPS" << endl;
    auto current =
        Arena::GetNodeValue<double>(device.GetNodeMap(), "AcquisitionFrameRate");
    if (abs(current - _image_config.get().frame_rate) >= 0.1) {
        LOG_ERROR_S << "Frame rate was not set correctly. Setpoint: "
                    << _image_config.get().frame_rate << ". Current: " << current << endl;
        throw runtime_error("Frame rate value differs.");
    }
}

void Task::analogConfiguration(Arena::IDevice& device)
{
    LOG_INFO_S << "Configuring Analog Control." << endl;

    LOG_INFO_S << "Setting Gain Selector to ."
               << getEnumName(_analog_controller_config.get().gain_selector,
                      gain_selector_name)
               << endl;
    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "GainSelector",
        getEnumName(_analog_controller_config.get().gain_selector, gain_selector_name));

    GenApi::CEnumerationPtr gain_selector = device.GetNodeMap()->GetNode("GainSelector");

    auto current = gain_selector->GetCurrentEntry()->GetSymbolic();
    if (current !=
        getEnumName(_analog_controller_config.get().gain_selector, gain_selector_name)) {
        LOG_WARN_S << "GainSelector value differs setpoint: "
                   << getEnumName(_analog_controller_config.get().gain_selector,
                          gain_selector_name)
                   << " current: " << current << endl;
        throw runtime_error("GainSelector value differs.");
    }

    LOG_INFO_S << "Setting Gain Auto mode to ."
               << getEnumName(_analog_controller_config.get().gain_auto, gain_auto_name)
               << endl;
    Arena::SetNodeValue<gcstring>(device.GetNodeMap(),
        "GainAuto",
        getEnumName(_analog_controller_config.get().gain_auto, gain_auto_name));

    GenApi::CEnumerationPtr gain_auto = device.GetNodeMap()->GetNode("GainAuto");

    current = gain_auto->GetCurrentEntry()->GetSymbolic();
    if (current !=
        getEnumName(_analog_controller_config.get().gain_auto, gain_auto_name)) {
        LOG_WARN_S << "GainAuto value differs setpoint: "
                   << getEnumName(_analog_controller_config.get().gain_auto,
                          gain_auto_name)
                   << " current: " << current << endl;
        throw runtime_error("GainAuto value differs.");
    }

    if (_analog_controller_config.get().gain_auto == GainAuto::GAIN_AUTO_OFF) {
        LOG_INFO_S << "Setting gain to: " << _analog_controller_config.get().gain << endl;
        GenApi::CFloatPtr gain = device.GetNodeMap()->GetNode("Gain");

        auto max_gain = gain->GetMax();
        auto min_gain = gain->GetMin();
        auto setpoint = _analog_controller_config.get().gain;

        if (setpoint > max_gain) {
            LOG_WARN_S << "Gain exceeds maximum value. Setting Exposure to "
                          "maximum value: "
                       << max_gain << endl;
            setpoint = max_gain;
        }
        else if (setpoint < min_gain) {
            LOG_WARN_S << "Gain exceeds minimum value. Setting Exposure to "
                          "minimum value: "
                       << min_gain << endl;
            setpoint = min_gain;
        }
        gain->SetValue(static_cast<double>(setpoint));

        auto current = gain->GetValue();
        if (abs(current - setpoint) >= 10) {
            LOG_WARN_S << "Gain value differs from setpoint: " << setpoint
                       << " current: " << current << endl;
            throw runtime_error("Gain value differs.");
        }
    }
}

void Task::factoryReset(Arena::IDevice* device, Arena::ISystem& system)
{
    ArenaDevice guard(device, system);

    GenApi::CCommandPtr trigger_software =
        device->GetNodeMap()->GetNode("DeviceFactoryReset");
    trigger_software->Execute();
    bool online = false;
    auto start_time = base::Time::now();
    while (!online) {
        try {
            GenApi::CIntegerPtr width = device->GetNodeMap()->GetNode("Width");
            width->GetMax();
            online = true;
        }
        catch (GenICam::TimeoutException& ex) {
            if (base::Time::now() - start_time >=
                _camera_config.get().camera_reset_timeout) {
                LOG_ERROR_S << "Timeout waiting for camera restart." << endl;
                throw runtime_error("Timeout waiting for camera restart.");
            }
            usleep(500000);
        }
    }
}

void Task::infoConfiguration(Arena::IDevice& device)
{
    LOG_INFO_S << "Setting device temperature selector to "
               << getEnumName(_camera_config.get().temperature_selector,
                      device_temperature_selector_name)
               << endl;
    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "DeviceTemperatureSelector",
        getEnumName(_camera_config.get().temperature_selector,
            device_temperature_selector_name));

    GenApi::CEnumerationPtr temperature_selector =
        m_device->GetNodeMap()->GetNode("DeviceTemperatureSelector");

    auto current = temperature_selector->GetCurrentEntry()->GetSymbolic();
    if (current != getEnumName(_camera_config.get().temperature_selector,
                       device_temperature_selector_name)) {
        LOG_WARN_S << "DeviceTemperatureSelector value differs setpoint: "
                   << getEnumName(_camera_config.get().temperature_selector,
                          device_temperature_selector_name)
                   << " current: " << current << endl;
        throw runtime_error("DeviceTemperatureSelector value differs.");
    }
    m_last_message = base::Time::now();
}

void Task::collectInfo()
{
    auto current_time = base::Time::now();
    if (current_time - m_last_message < _camera_config.get().update_info) {
        return;
    }
    else {
        camera_lucid::CameraInfo info_message;
        try {
            // get DeviceTemperature
            LOG_INFO_S << "Acquiring temperature from "
                       << getEnumName(_camera_config.get().temperature_selector,
                              device_temperature_selector_name)
                       << endl;
            info_message.temperature = info_message.temperature.fromCelsius(
                Arena::GetNodeValue<double>(m_device->GetNodeMap(), "DeviceTemperature"));
        }
        catch (GenICam::GenericException& ge) {
            LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
            throw runtime_error(ge.what());
        }
        _info.write(info_message);
        m_last_message = current_time;
    }
}