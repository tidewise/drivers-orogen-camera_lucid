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
    if (!TaskBase::configureHook())
        return false;

    if (!connectToCamera())
        return false;

    if (!switchOverAccess())
        return false;

    if (!configureCamera())
        return false;
    return true;
}
bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;

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
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    m_device->StopStream();
    m_system->DestroyDevice(m_device);
    Arena::CloseSystem(m_system);
}

template <typename F> bool Task::applyCommand(F f)
{
    try {
        if (!f)
            throw;
        else
            return true;
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
    }
    catch (std::exception& ex) {
        LOG_ERROR_S << "Standard exception thrown: " << ex.what() << endl;
    }
    catch (...) {
        LOG_ERROR_S << "Unexpected exception thrown" << endl;
    }
    return false;
}

void Task::treatError()
{
    try {
        throw;
    }
    catch (GenICam::GenericException& ge) {
        LOG_ERROR_S << "GenICam exception thrown: " << ge.what() << endl;
    }
    catch (std::exception& ex) {
        LOG_ERROR_S << "Standard exception thrown: " << ex.what() << endl;
    }
    catch (...) {
        LOG_ERROR_S << "Unexpected exception thrown" << endl;
    }
}

bool Task::connectToCamera()
{
    LOG_INFO_S << "Looking for camera" << endl;
    m_ip = _ip.get();

    m_system = Arena::OpenSystem();
    m_system->UpdateDevices(100);
    vector<Arena::DeviceInfo> device_infos = m_system->GetDevices();
    for (auto device : device_infos) {
        if (device.IpAddressStr() == m_ip.c_str()) {
            m_device = m_system->CreateDevice(device);
            LOG_INFO_S << "Connected to camera" << endl;
            return true;
        }
    }
    LOG_ERROR_S << "Camera not found" << endl;
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
    else {
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
    }
    LOG_WARN_S << "Task failed to retrieved the Camera Read/Write Access." << endl;
    return false;
}

bool Task::configureCamera()
{
    if (!resetConfiguration())
        return false;

    LOG_INFO_S << "Configuring camera." << endl;

    LOG_INFO_S << "Setting Acquisition Mode." << endl;
    Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
        "AcquisitionMode",
        "Continuous");

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
        convertFrameModeToPixelFormat(_camera_format.get(), _depth.get()).c_str());

    if (!dimensionsConfiguration())
        return false;

    // When horizontal binning is used, horizontal decimation (if supported) is not
    // available. When vertical binning is used, vertical decimation (if supported) is not
    // available.

    // If the camera is acquiring images, AcquisitionStop must be called before adjusting
    // binning settings.
    if (!binningConfiguration())
        return false;

    if (!decimationConfiguration())
        return false;

    if (!exposureConfiguration())
        return false;

    Frame* frame =
        new Frame(_width.get(), _height.get(), _depth.get(), _camera_format.get(), 0, 0);
    m_frame.reset(frame);

    return true;
}

void Task::startCamera()
{
    m_device->StartStream();
}

void Task::acquireFrame()
{
    Arena::IImage* image = m_device->GetImage(_frame_timeout.get().toMilliseconds());

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
    _frame.write(m_frame);
    m_device->RequeueBuffer(image);
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

bool Task::resetConfiguration()
{
    try {
        LOG_INFO_S << "Reseting configuration" << endl;
        Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(),
            "UserSetSelector",
            "Default");

        Arena::ExecuteNode(m_device->GetNodeMap(), "UserSetLoad");

        if (Arena::GetNodeValue<gcstring>(m_device->GetNodeMap(), "UserSetSelector") ==
            "Default")
            return true;
    }
    catch (...) {
        treatError();
    }
    return false;
}

bool Task::binningConfiguration()
{
    try {
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

        LOG_INFO_S << "Set binning mode to " << _binning_selector.get() << endl;
        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "BinningSelector",
            _binning_selector.get().c_str());

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

        if (_binning_x.get() > max_binning_height) {
            LOG_WARN_S << "Binning_x is bigger than the maximum allowed value! Setpoint: "
                       << _binning_x.get() << ". Maximum: " << max_binning_height << endl;
            return false;
        }
        else if (_binning_y.get() > max_binning_width) {
            LOG_WARN_S << "Binning_y is bigger than the maximum allowed value! Setpoint: "
                       << _binning_y.get() << ". Maximum: " << max_binning_width << endl;
            return false;
        }

        Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
            "BinningVertical",
            _binning_x.get());

        Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
            "BinningHorizontal",
            _binning_y.get());

        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "BinningVerticalMode",
            _binning_type.get().c_str());

        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "BinningHorizontalMode",
            _binning_type.get().c_str());

        if (_binning_y.get() ==
                Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "BinningVertical") &&
            _binning_x.get() == Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(),
                                    "BinningHorizontal")) {
            return true;
        }
    }
    catch (...) {
        treatError();
    }
    return false;
}

bool Task::decimationConfiguration()
{
    try {
        // Initial check if sensor decimation is supported.
        //    Entry may not be in XML file. Entry may be in the file but set to
        //    unreadable or unavailable. Note: there is a case where sensor
        //    decimation is not supported but this test passes However, we must set
        //    DecimationSelector to Sensor before we can test for that.
        GenApi::CEnumerationPtr decimation_selector_node =
            m_device->GetNodeMap()->GetNode("DecimationSelector");
        GenApi::CEnumEntryPtr decimation_sensor_entry =
            decimation_selector_node->GetEntryByName(_decimation_selector.get().c_str());
        if (decimation_sensor_entry == 0 ||
            !GenApi::IsAvailable(decimation_sensor_entry)) {
            LOG_WARN_S << "Sensor decimation not supported by device: not available from "
                          "DecimationSelector"
                       << endl;
            return false;
        }

        LOG_INFO_S << "Set decimation mode to: " << _decimation_selector.get() << endl;
        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "DecimationSelector",
            _decimation_selector.get().c_str());

        // Check if the nodes for the height and width of the bin are available
        //    For rare case where sensor decimation is unsupported but still appears as
        //    an option. Must be done after setting DecimationSelector to Sensor. It was
        //    probably just a bug in the firmware.
        if (!GenApi::IsAvailable(m_device->GetNodeMap()->GetNode("DecimationVertical")) ||
            !GenApi::IsAvailable(m_device->GetNodeMap()->GetNode("DecimationVertical"))) {
            LOG_WARN_S
                << "Sensor decimation not supported by device: DecimationVertical or "
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

        if (_decimation_x.get() > max_decimation_height) {
            LOG_WARN_S
                << "Decimation_x is bigger than the maximum allowed value! Setpoint: "
                << _decimation_x.get() << ". Maximum: " << max_decimation_height << endl;
            return false;
        }
        else if (_decimation_y.get() > max_decimation_width) {
            LOG_WARN_S
                << "Decimation_y is bigger than the maximum allowed value! Setpoint: "
                << _decimation_y.get() << ". Maximum: " << max_decimation_width << endl;
            return false;
        }

        Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
            "DecimationVertical",
            _decimation_x.get());

        Arena::SetNodeValue<int64_t>(m_device->GetNodeMap(),
            "DecimationHorizontal",
            _decimation_y.get());

        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "DecimationVerticalMode",
            _decimation_type.get().c_str());

        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "DecimationHorizontalMode",
            _decimation_type.get().c_str());

        if (_decimation_y.get() == Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(),
                                       "DecimationVertical") &&
            _decimation_x.get() == Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(),
                                       "DecimationHorizontal")) {
            return true;
        }
    }
    catch (...) {
        treatError();
    }
    return false;
}

bool Task::dimensionsConfiguration()
{
    try {
        LOG_INFO_S << "Setting Dimensions." << endl;
        GenApi::CIntegerPtr width = m_device->GetNodeMap()->GetNode("Width");
        GenApi::CIntegerPtr height = m_device->GetNodeMap()->GetNode("Height");
        GenApi::CIntegerPtr offset_x = m_device->GetNodeMap()->GetNode("OffsetX");
        GenApi::CIntegerPtr offset_y = m_device->GetNodeMap()->GetNode("OffsetY");

        if (!width || !GenApi::IsReadable(width) || !GenApi::IsWritable(width)) {
            LOG_INFO_S << "Width node not found/readable/writable" << endl;
            return false;
        }

        if (!height || !GenApi::IsReadable(height) || !GenApi::IsWritable(height)) {
            LOG_INFO_S << "Height node not found/readable/writable" << endl;
            return false;
        }

        if (!offset_x || !GenApi::IsReadable(offset_x) || !GenApi::IsWritable(offset_x)) {
            LOG_INFO_S << "Offset X node not found/readable/writable" << endl;
            return false;
        }

        if (!offset_y || !GenApi::IsReadable(offset_y) || !GenApi::IsWritable(offset_y)) {
            LOG_INFO_S << "Offset Y node not found/readable/writable" << endl;
            return false;
        }

        LOG_INFO_S << "Setting Width." << endl;
        width->SetValue(_width.get());
        LOG_INFO_S << "Setting Height." << endl;
        height->SetValue(_height.get());
        LOG_INFO_S << "Setting Offset X." << endl;
        offset_x->SetValue(_offset_x.get());
        LOG_INFO_S << "Setting Offset Y." << endl;
        offset_y->SetValue(_offset_y.get());

        if (_width.get() ==
                Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "Width") &&
            _height.get() ==
                Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "Height") &&
            _offset_x.get() ==
                Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "OffsetX") &&
            _offset_y.get() ==
                Arena::GetNodeValue<int64_t>(m_device->GetNodeMap(), "OffsetY")) {
            return true;
        }
    }
    catch (...) {
        treatError();
    }
    return false;
}

bool Task::exposureConfiguration()
{
    try {
        LOG_INFO_S << "Setting auto exposure to " << _auto_exposure.get() << endl;
        Arena::SetNodeValue<gcstring>(m_device->GetNodeMap(),
            "ExposureAuto",
            _auto_exposure.get().c_str());

        GenApi::CEnumerationPtr exposure_auto =
            m_device->GetNodeMap()->GetNode("ExposureAuto");

        if (auto current = exposure_auto->GetCurrentEntry()->GetSymbolic() !=
                           _auto_exposure.get().c_str()) {
            LOG_WARN_S << "ExposureAuto value differs setpoint: " << _auto_exposure.get()
                       << " current: " << current << endl;
            return false;
        }

        if (_auto_exposure.get() == "Off") {
            LOG_INFO_S << "Setting exposure time to: "
                       << _exposure_time.get().toMicroseconds() << "us" << endl;
            GenApi::CFloatPtr exposure_time =
                m_device->GetNodeMap()->GetNode("ExposureTime");

            auto max_exposure_time = exposure_time->GetMax();
            auto min_exposure_time = exposure_time->GetMin();
            auto setpoint = _exposure_time.get().toMicroseconds();

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
            exposure_time->SetValue(setpoint);

            if (auto current = exposure_time->GetValue() == setpoint) {
                LOG_WARN_S << "Exposure Time value differs from setpoint: " << setpoint
                           << " current: " << current << endl;
                return true;
            }
        }
    }
    catch (...) {
        treatError();
    }

    return true;
}

bool Task::offsetConfiguration()
{
    return false;
}