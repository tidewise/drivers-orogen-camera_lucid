/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include "base/samples/Frame.hpp"
#include <base-logging/Logging.hpp>

using namespace std;
using namespace camera_lucid;
using namespace base::samples::frame;
using namespace GenICam;

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
    try {
        if (!TaskBase::configureHook()) {
            return false;
        }

        if (!connectToCamera()) {
            return false;
        }

        if (!switchOverAccess()) {
            return false;
        }

        if (!configureCamera()) {
            return false;
        }
        return true;
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
        return false;
    }
    catch (std::exception& ex) {
        LOG_ERROR_S << "Standard exception thrown: " << ex.what() << endl;
        return false;
    }
}

bool Task::startHook()
{
    if (!TaskBase::startHook()) {
        return false;
    }
    startCamera();
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    acquireFrame();
}
void Task::errorHook()
{
    TaskBase::errorHook();
    m_device->StopStream();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    m_device->StopStream();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    m_system->DestroyDevice(m_device);
    Arena::CloseSystem(m_system);
}

bool Task::connectToCamera()
{
    LOG_INFO_S << "Looking for camera" << endl;
    m_ip = _ip.get();
    try {
        m_system = Arena::OpenSystem();
    }
    catch (GenICam::GenericException& ge) {
        LOG_WARN_S << "Arena System was already on, closing it and reopening." << endl;
        Arena::CloseSystem(m_system);
        m_system = Arena::OpenSystem();
    }
    m_system->UpdateDevices(100);
    vector<Arena::DeviceInfo> device_infos = m_system->GetDevices();
    for (auto device : device_infos) {
        if (m_ip.compare(device.IpAddressStr()) == 0) {
            m_device = m_system->CreateDevice(device);
            LOG_INFO_S << "Connected to camera" << endl;
            return true;
        }
    }
    string cameras;
    for (auto device : device_infos) {
        cameras.append(device.IpAddressStr());
        cameras.append(", ");
    }
    LOG_ERROR_S << "Camera " << _ip.get() << " not found. Found cameras: " << cameras
                << endl;
    return false;
}

bool Task::switchOverAccess()
{
    LOG_INFO_S << "Checking Camera Read/Write Access." << endl;
    // Retrieve DeviceAccessStatus
    GenICam::gcstring device_access_status =
        Arena::GetNodeValue<GenICam::gcstring>(m_device->GetTLDeviceNodeMap(),
            "DeviceAccessStatus");
    // Check DeviceAccessStatus to determine if we have ReadWrite status
    // If we have ReadOnly status, another application has control of the camera:
    if (device_access_status == "ReadWrite") {
        // Set a switchover key in case another application needs to take control
        Arena::SetNodeValue<int64_t>(m_device->GetTLDeviceNodeMap(),
            "CcpSwitchoverKey",
            m_switchover_key);
        LOG_INFO_S << "Task already has the Camera Read/Write Access." << endl;
        return true;
    }

    // Apply the correct switchover key and change the DeviceAccessStatus to ReadWrite
    // to gain control If there is no switchover key, or an incorrect switchover key,
    // ReadWrite access will not be granted
    Arena::SetNodeValue<int64_t>(m_device->GetTLDeviceNodeMap(),
        "CcpSwitchoverKey",
        m_switchover_key);
    Arena::SetNodeValue<GenICam::gcstring>(m_device->GetTLDeviceNodeMap(),
        "DeviceAccessStatus",
        "ReadWrite");

    // confirm if operation was successfull
    device_access_status =
        Arena::GetNodeValue<GenICam::gcstring>(m_device->GetTLDeviceNodeMap(),
            "DeviceAccessStatus");
    if (device_access_status == "ReadWrite") {
        LOG_INFO_S << "Task retrieved the Camera Read/Write Access." << endl;
        return true;
    }

    LOG_WARN_S << "Task failed to retrieved the Camera Read/Write Access." << endl;
    return false;
}

bool Task::configureCamera()
{
    LOG_INFO_S << "Performing Factory Reset" << endl;
    if (!factoryReset()) {
        return false;
    }

    LOG_INFO_S << "Configuring camera." << endl;
    if (!acquisitionConfiguration()) {
        return false;
    }

    LOG_INFO_S << "Setting StreamBufferHandlingMode." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(m_device->GetTLStreamNodeMap(),
        "StreamBufferHandlingMode",
        "NewestOnly");

    LOG_INFO_S << "Setting StreamAutoNegotiatePacketSize." << endl;
    Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(),
        "StreamAutoNegotiatePacketSize",
        true);

    LOG_INFO_S << "Setting StreamPacketResendEnable." << endl;
    Arena::SetNodeValue<bool>(m_device->GetTLStreamNodeMap(),
        "StreamPacketResendEnable",
        true);

    LOG_INFO_S << "Setting PixelFormat." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
        "PixelFormat",
        convertFrameModeToPixelFormat(_image_config.get().format,
            _image_config.get().depth)
            .c_str());

    // When horizontal binning is used, horizontal decimation (if supported) is not
    // available. When vertical binning is used, vertical decimation (if supported) is not
    // available.

    // If the camera is acquiring images, AcquisitionStop must be called before adjusting
    // binning settings.
    if (!binningConfiguration()) {
        return false;
    }

    if (!decimationConfiguration()) {
        return false;
    }

    if (!dimensionsConfiguration()) {
        return false;
    }

    if (!exposureConfiguration()) {
        return false;
    }

    Frame* frame = new Frame(_image_config.get().width,
        _image_config.get().height,
        _image_config.get().depth,
        _image_config.get().format,
        0,
        0);
    m_frame.reset(frame);

    return true;
}

void Task::startCamera()
{
    LOG_INFO_S << "Starting frame acquisition." << endl;
    m_device->StartStream();
}

void Task::acquireFrame()
{
    try {
        Arena::IImage* image =
            m_device->GetImage(_image_config.get().frame_timeout.toMilliseconds());
        if (image->IsIncomplete()) {
            LOG_ERROR_S << "Image is not complete!! " << endl;
            m_device->RequeueBuffer(image);
        }

        size_t size = image->GetSizeFilled();
        size_t width = image->GetWidth();
        size_t height = image->GetHeight();
        PfncFormat pixel_format = static_cast<PfncFormat>(image->GetPixelFormat());
        uint8_t data_depth = 0U;
        auto format = convertPixelFormatToFrameMode(pixel_format, data_depth);

        Frame* out_frame = m_frame.write_access();
        if (width != out_frame->getWidth() || height != out_frame->getHeight() ||
            format != out_frame->getFrameMode()) {
            out_frame->init(width, height, data_depth, format, 0, size);
        }
        out_frame->time = base::Time::now();
        out_frame->received_time = out_frame->time;

        out_frame->setImage(image->GetData(), size);
        out_frame->setStatus(STATUS_VALID);
        m_frame.reset(out_frame);
        m_device->RequeueBuffer(image);
        _frame.write(m_frame);
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
    }
    catch (std::exception& ex) {
        LOG_ERROR_S << "Standard exception thrown: " << ex.what() << endl;
    }
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

bool Task::binningConfiguration()
{
    // Initial check if sensor binning is supported.
    //    Entry may not be in XML file. Entry may be in the file but set to
    //    unreadable or unavailable. Note: there is a case where sensor
    //    binning is not supported but this test passes However, we must set
    //    BinningSelector to Sensor before we can test for that.
    GenApi::CEnumerationPtr binning_selector_node =
        m_device->GetNodeMap()->GetNode("BinningSelector");
    GenApi::CEnumEntryPtr binning_sensor_entry =
        binning_selector_node->GetEntryByName("Digital");
    if (binning_sensor_entry == 0 || !GenApi::IsAvailable(binning_sensor_entry)) {
        LOG_WARN_S << "Sensor binning not supported by device: not available from "
                      "BinningSelector"
                   << endl;
        return false;
    }

    LOG_INFO_S << "Set binning mode to "
               << binning_selector_name.at(_binning_config.get().selector) << endl;
    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "BinningSelector",
        binning_selector_name.at(_binning_config.get().selector).c_str());

    // Check if the nodes for the height and width of the bin are available
    //    For rare case where sensor binning is unsupported but still appears as
    //    an option. Must be done after setting BinningSelector to Sensor. It was
    //    probably just a bug in the firmware.
    if (!GenApi::IsAvailable(m_device->GetNodeMap()->GetNode("BinningVertical")) ||
        !GenApi::IsAvailable(m_device->GetNodeMap()->GetNode("BinningVertical"))) {
        LOG_WARN_S << "Sensor binning not supported by device: BinningVertical or "
                      "BinningHorizontal not available."
                   << endl;
        return false;
    }

    // Find max for bin height & width.
    //    For maximum compression.
    GenApi::CIntegerPtr binning_vertical_node =
        m_device->GetNodeMap()->GetNode("BinningVertical");
    GenApi::CIntegerPtr binning_horizontal_node =
        m_device->GetNodeMap()->GetNode("BinningHorizontal");

    int64_t max_binning_height = binning_vertical_node->GetMax();
    int64_t max_binning_width = binning_horizontal_node->GetMax();

    if (_binning_config.get().binning_x > max_binning_height) {
        LOG_WARN_S << "Binning_x is bigger than the maximum allowed value! Setpoint: "
                   << _binning_config.get().binning_x
                   << ". Maximum: " << max_binning_height << endl;
        return false;
    }
    else if (_binning_config.get().binning_y > max_binning_width) {
        LOG_WARN_S << "Binning_y is bigger than the maximum allowed value! Setpoint: "
                   << _binning_config.get().binning_y
                   << ". Maximum: " << max_binning_width << endl;
        return false;
    }

    Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
        "BinningVertical",
        _binning_config.get().binning_x);

    Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
        "BinningHorizontal",
        _binning_config.get().binning_y);

    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "BinningVerticalMode",
        binning_mode_name.at(_binning_config.get().vertical_mode).c_str());

    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "BinningHorizontalMode",
        binning_mode_name.at(_binning_config.get().horizontal_mode).c_str());

    if (_binning_config.get().binning_y !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "BinningVertical") ||
        _binning_config.get().binning_x !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "BinningHorizontal")) {
        LOG_WARN_S << "Failed setting binning parameters";
        return false;
    }

    return true;
}

bool Task::decimationConfiguration()
{

    // Initial check if sensor decimation is supported.
    //    Entry may not be in XML file. Entry may be in the file but set to
    //    unreadable or unavailable. Note: there is a case where sensor
    //    decimation is not supported but this test passes However, we must set
    //    DecimationSelector to Sensor before we can test for that.
    GenApi::CEnumerationPtr decimation_selector_node =
        m_device->GetNodeMap()->GetNode("DecimationSelector");
    GenApi::CEnumEntryPtr decimation_sensor_entry =
        decimation_selector_node->GetEntryByName(
            decimation_selector_name.at(_decimation_config.get().selector).c_str());
    if (decimation_sensor_entry == 0 || !GenApi::IsAvailable(decimation_sensor_entry)) {
        LOG_WARN_S << "Sensor decimation not supported by device: not available from "
                      "DecimationSelector"
                   << endl;
        return false;
    }

    LOG_INFO_S << "Set decimation mode to: "
               << decimation_selector_name.at(_decimation_config.get().selector) << endl;
    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "DecimationSelector",
        decimation_selector_name.at(_decimation_config.get().selector).c_str());

    // Check if the nodes for the height and width of the bin are available
    //    For rare case where sensor decimation is unsupported but still appears as
    //    an option. Must be done after setting DecimationSelector to Sensor. It was
    //    probably just a bug in the firmware.
    if (!GenApi::IsAvailable(m_device->GetNodeMap()->GetNode("DecimationVertical")) ||
        !GenApi::IsAvailable(m_device->GetNodeMap()->GetNode("DecimationVertical"))) {
        LOG_WARN_S << "Sensor decimation not supported by device: DecimationVertical or "
                      "DecimationHorizontal not available."
                   << endl;
        return false;
    }

    GenApi::CIntegerPtr decimation_vertical_node =
        m_device->GetNodeMap()->GetNode("DecimationVertical");
    GenApi::CIntegerPtr decimation_horizontal_node =
        m_device->GetNodeMap()->GetNode("DecimationHorizontal");

    int64_t max_decimation_height = decimation_vertical_node->GetMax();
    int64_t max_decimation_width = decimation_horizontal_node->GetMax();

    if (_decimation_config.get().decimation_x > max_decimation_height) {
        LOG_WARN_S << "Decimation_x is bigger than the maximum allowed value! Setpoint: "
                   << _decimation_config.get().decimation_x
                   << ". Maximum: " << max_decimation_height << endl;
        return false;
    }
    else if (_decimation_config.get().decimation_y > max_decimation_width) {
        LOG_WARN_S << "Decimation_y is bigger than the maximum allowed value! Setpoint: "
                   << _decimation_config.get().decimation_y
                   << ". Maximum: " << max_decimation_width << endl;
        return false;
    }

    Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
        "DecimationVertical",
        _decimation_config.get().decimation_x);

    Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
        "DecimationHorizontal",
        _decimation_config.get().decimation_y);

    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "DecimationVerticalMode",
        decimation_mode_name.at(_decimation_config.get().vertical_mode).c_str());

    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "DecimationHorizontalMode",
        decimation_mode_name.at(_decimation_config.get().horizontal_mode).c_str());

    if (_decimation_config.get().decimation_y !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "DecimationVertical") ||
        _decimation_config.get().decimation_x !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(),
                "DecimationHorizontal")) {
        return true;
    }

    return true;
}

bool Task::dimensionsConfiguration()
{
    LOG_INFO_S << "Setting Dimensions." << endl;
    GenApi::CIntegerPtr width = m_device->GetNodeMap()->GetNode("Width");
    GenApi::CIntegerPtr height = m_device->GetNodeMap()->GetNode("Height");
    GenApi::CIntegerPtr offset_x = m_device->GetNodeMap()->GetNode("OffsetX");
    GenApi::CIntegerPtr offset_y = m_device->GetNodeMap()->GetNode("OffsetY");

    if (!width || !GenApi::IsReadable(width) || !GenApi::IsWritable(width)) {
        LOG_ERROR_S << "Width node not found/readable/writable" << endl;
        return false;
    }

    if (!height || !GenApi::IsReadable(height) || !GenApi::IsWritable(height)) {
        LOG_ERROR_S << "Height node not found/readable/writable" << endl;
        return false;
    }

    if (!offset_x || !GenApi::IsReadable(offset_x) || !GenApi::IsWritable(offset_x)) {
        LOG_ERROR_S << "Offset X node not found/readable/writable" << endl;
        return false;
    }

    if (!offset_y || !GenApi::IsReadable(offset_y) || !GenApi::IsWritable(offset_y)) {
        LOG_ERROR_S << "Offset Y node not found/readable/writable" << endl;
        return false;
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
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "Width") ||
        _image_config.get().height !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "Height") ||
        _image_config.get().offset_x !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "OffsetX") ||
        _image_config.get().offset_y !=
            Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "OffsetY")) {
        return false;
    }
    return true;
}

bool Task::exposureConfiguration()
{
    LOG_INFO_S << "Setting auto exposure to "
               << exposure_auto_name.at(_image_config.get().exposure_auto) << endl;
    Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
        "ExposureAuto",
        exposure_auto_name.at(_image_config.get().exposure_auto).c_str());

    GenApi::CEnumerationPtr exposure_auto =
        m_device->GetNodeMap()->GetNode("ExposureAuto");

    auto current = exposure_auto->GetCurrentEntry()->GetSymbolic();
    if (current != exposure_auto_name.at(_image_config.get().exposure_auto).c_str()) {
        LOG_WARN_S << "ExposureAuto value differs setpoint: "
                   << exposure_auto_name.at(_image_config.get().exposure_auto)
                   << " current: " << current << endl;
        return false;
    }

    if (_image_config.get().exposure_auto == ExposureAuto::EXPOSURE_AUTO_OFF) {
        LOG_INFO_S << "Setting exposure time to: "
                   << _image_config.get().exposure_time.toMicroseconds() << "us" << endl;
        GenApi::CFloatPtr exposure_time = m_device->GetNodeMap()->GetNode("ExposureTime");

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
            return false;
        }
    }

    return true;
}

bool Task::acquisitionConfiguration()
{
    LOG_INFO_S << "Configuring Acquisition." << endl;

    LOG_INFO_S << "Setting Acquisition Mode." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
        "AcquisitionMode",
        "Continuous");

    LOG_INFO_S << "Setting Acquisition Frame Rate." << endl;
    // Enable Acquisition Frame Rate
    Arena::SetNodeValue<bool>(m_device->GetNodeMap(), "AcquisitionFrameRateEnable", true);

    // get Acquisition Frame Rate node
    GenApi::CFloatPtr acquisition_frame_rate =
        m_device->GetNodeMap()->GetNode("AcquisitionFrameRate");

    GenApi::CFloatPtr pExposureTime = m_device->GetNodeMap()->GetNode("ExposureTime");
    LOG_INFO_S << "Exposure Time " << pExposureTime->GetValue() << " milliseconds"
               << std::endl;

    acquisition_frame_rate->SetValue(_image_config.get().frame_rate);

    auto current =
        Arena::GetNodeValue<double>(m_device->GetNodeMap(), "AcquisitionFrameRate");
    if (abs(current - _image_config.get().frame_rate) >= 0.1) {
        LOG_ERROR_S << "Frame rate was not set correctly. Setpoint: "
                    << _image_config.get().frame_rate << ". Current: " << current << endl;
        return false;
    }

    return true;
}

bool Task::factoryReset()
{
    GenApi::CCommandPtr trigger_software =
        m_device->GetNodeMap()->GetNode("DeviceFactoryReset");
    trigger_software->Execute();
    bool online = false;
    while (!online) {
        try {
            GenApi::CIntegerPtr width = m_device->GetNodeMap()->GetNode("Width");
            width->GetMax();
            online = true;
        }
        catch (GenICam::TimeoutException& ex) {
            usleep(500000);
        }
    }
    m_system->DestroyDevice(m_device);

    if (!connectToCamera()) {
        return false;
    }

    if (!switchOverAccess()) {
        return false;
    }
    return true;
}