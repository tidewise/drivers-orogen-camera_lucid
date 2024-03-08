#ifndef camera_lucid_TYPES_HPP
#define camera_lucid_TYPES_HPP

#include "Arena/ArenaApi.h"
#include <base/Temperature.hpp>
#include <base/Time.hpp>
#include <base/samples/Frame.hpp>
#include <string>
#include <vector>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace camera_lucid {
    enum BinningSelector {
        BINNING_SELECTOR_DIGITAL = 0,
        BINNING_SELECTOR_SENSOR = 1
    };
    static std::vector<std::string> binning_selector_name = {"Digital", "Sensor"};

    enum BinningMode {
        BINNING_MODE_SUM = 0,
        BINNING_MODE_AVERAGE = 1
    };
    static std::vector<std::string> binning_mode_name = {"Sum", "Average"};

    enum DecimationSelector {
        DECIMATION_SELECTOR_SENSOR = 0
    };
    static std::vector<std::string> decimation_selector_name = {"Sensor"};

    enum DecimationMode {
        DECIMATION_MODE_DISCARD = 0,
    };
    static std::vector<std::string> decimation_mode_name = {"Discard"};

    enum ExposureAuto {
        EXPOSURE_AUTO_OFF = 0,
        EXPOSURE_AUTO_ONCE = 1,
        EXPOSURE_AUTO_CONTINUOUS = 2
    };
    static std::vector<std::string> exposure_auto_name = {"Off", "Once", "Continuous"};

    enum ExposureAutoAlgorithm {
        EXPOSURE_AUTO_ALGORITHM_MEDIAN = 0,
        EXPOSURE_AUTO_ALGORITHM_MEAN = 1
    };
    static std::vector<std::string> exposure_auto_algorithm = {"Median", "Mean"};

    enum ExposureAutoLimitAuto {
        EXPOSURE_AUTO_LIMIT_AUTO_OFF = 0,
        EXPOSURE_AUTO_LIMIT_AUTO_CONTINUOUS = 1
    };
    static std::vector<std::string> exposure_auto_limit_auto_name = {"Off", "Continuous"};

    enum DeviceTemperatureSelector {
        DEVICE_TEMPERATURE_SELECTOR_SENSOR = 0,
        DEVICE_TEMPERATURE_SELECTOR_TEC = 1
    };
    static std::vector<std::string> device_temperature_selector_name = {"Sensor", "TEC"};

    enum GainSelector {
        GAIN_SELECTOR_ALL = 0,
        GAIN_SELECTOR_SHUTTER_1 = 1,
        GAIN_SELECTOR_SHUTTER_2 = 2
    };
    static std::vector<std::string> gain_selector_name = {"All", "Shutter1", "Shutter2"};

    enum GainAuto {
        GAIN_AUTO_OFF = 0,
        GAIN_AUTO_ONCE = 1,
        GAIN_AUTO_CONTINUOUS = 2
    };
    static std::vector<std::string> gain_auto_name = {"Off", "Once", "Continuous"};

    enum AcquisitionStartMode {
        ACQUISITION_START_MODE_NORMAL = 0,
        ACQUISITION_START_MODE_LOWLATENCY = 1,
        ACQUISITION_START_MODE_PTPSYNC = 2
    };
    static std::vector<std::string> acquisition_start_mode_name{"Normal",
        "LowLatency",
        "PTPSync"};

    struct BinningConfig {
        /** Selects which binning engine is controlled by the BinningHorizontal and
         * BinningVertical features. */
        BinningSelector selector = BINNING_SELECTOR_DIGITAL;
        /** Selects how to combine the horizontal pixels together.*/
        BinningMode horizontal_mode = BINNING_MODE_SUM;
        /** Selects how to combine the vertical pixels together.*/
        BinningMode vertical_mode = BINNING_MODE_SUM;
        /**Number of horizontal pixels to combine together. This reduces the horizontal
         * resolution (width) of the image. A value of 1 indicates that no horizontal
         * binning is performed by the camera.*/
        int binning_x = 1;
        /** Number of vertical pixels to combine together. This reduces the vertical
         * resolution (height) of the image. A value of 1 indicates that no vertical
         * binning is performed by the camera. */
        int binning_y = 1;
    };

    struct DecimationConfig {
        /** Selects which decimation engine is controlled by the DecimationHorizontal and
         * DecimationVertical features. */
        DecimationSelector selector = DECIMATION_SELECTOR_SENSOR;
        /** Selects how to decimate the horizontal pixels.*/
        DecimationMode horizontal_mode = DECIMATION_MODE_DISCARD;
        /** Selects how to decimate the vertical pixels.*/
        DecimationMode vertical_mode = DECIMATION_MODE_DISCARD;
        /** Number of horizontal pixels to decimate. This reduces the horizontal
         * resolution (width) of the image. A value of 1 indicates that no horizontal
         * decimation is performed by the camera.*/
        int decimation_x = 1;
        /** Number of vertical pixels to decimate. This reduces the vertical resolution
         * (height) of the image. A value of 1 indicates that no vertical decimation is
         * performed by the camera.*/
        int decimation_y = 1;
    };

    struct AnalogControllerConfig {
        /** Value used for gain*/
        float gain = 0.0;
        /** Minimal Gain Value*/
        float gain_min = 0.0;
        /** Maximum Gain Value*/
        float gain_max = 48.0;
        /** Sets the automatic Gain mode*/
        GainAuto gain_auto = GainAuto::GAIN_AUTO_OFF;
        /** Selects all shutters or shutter1 or shutter2*/
        GainSelector gain_selector = GainSelector::GAIN_SELECTOR_ALL;
        /** Gamma correction mode - For uncontrolled outdoor environments,
         *  The recommended configuration is enabled (true). */
        bool gamma_enabled = true;
        /** Gamma correction configuration - For uncontrolled outdoor environments,
         * The recommended gamma value is 0.5. */
        float gamma = 0.5;
    };

    struct PTPConfig {
        /** Whether PTP support is enabled or not */
        bool enabled = false;
        /** Whether this camera should only configure itself as a slave, or
         * could be master */
        bool slave_only = false;
        /** PTP Synchronization timeout
         *
         * If set, the component will wait for the camera's PTP clock to either become
         * master, or get synchronized for this long. It will raise after this timeout
         */
        base::Time synchronization_timeout;
    };

    struct TransmissionConfig {
        /** Whether the Arena software requires a packet resend*/
        bool stream_packet_resend = true;
        /** Whether the camera controls data transfer (false), or the component (true) */
        bool explicit_data_transfer = false;
        /** Delay between two packets sent by the camera in nanoseconds
         *
         * There is no sound default, but there has to always be a delay. Set to zero to
         * keep the camera's current value
         */
        uint64_t packet_delay = 0;

        /** Delay between acquisition and transmission of frames
         *
         * The default is zero. Use non-zero values to interleave transmission of
         * different cameras, when the cameras have synchronized acquisition (e.g.
         * using PTPSync)
         */
        uint64_t frame_transmission_delay = 0;

        /** Configures the MTU. Lucid Arena SDK negotiates automatically the packet size.
         * If true, the mtu negotiation will be done automatically and it will be checked
         * if the negotiated mtu is greater or equal than the desired mtu.
         * If false, the mtu will be set as desired mtu.*/
        bool auto_negotiate_mtu = false;

        /** Desired mtu
         * If the camera's mtu value is bellow this value, after the timeout an exception
         * is thrown.*/
        int mtu = 1500;

        /** Sleep while checking mtu value. The configurable task will be in a  loop until
         * the desired mtu is configured or if timeouts.*/
        base::Time mtu_check_sleep = base::Time::fromMilliseconds(500);
        /** Timeout for negotiating mtu.*/
        base::Time mtu_check_timeout = base::Time::fromSeconds(10);
    };

    struct ImageConfig {
        /** Control when/how acquisition starts on the camera*/
        AcquisitionStartMode acquisition_start_mode = ACQUISITION_START_MODE_NORMAL;

        /** Specifies the frequency in which frames are acquired */
        double frame_rate = 24.0;

        /** Timeout for frame acquisition.*/
        base::Time frame_timeout = base::Time::fromMilliseconds(500);
        /** Maximum number of timeouts every period based on check_image_status, after
         * this value it will return an exeption*/
        int max_acquisition_timeout = 10;
        /** Maximum number of incomplete images every period based on check_image_status,
         * after this value it will return an exeption*/
        int max_incomplete_images = 10;
        /** Period to check the amount of acquisition timeouts and incomplete images.*/
        base::Time check_image_status = base::Time::fromSeconds(1);
        /** Image format*/
        base::samples::frame::frame_mode_t format =
            base::samples::frame::frame_mode_t::MODE_BAYER_RGGB;
        /** Depth*/
        uint8_t depth = 8;
        /** Sets the automatic exposure mode.*/
        ExposureAuto exposure_auto = EXPOSURE_AUTO_CONTINUOUS;
        /** Sets the automatic exposure algorithm*/
        ExposureAutoAlgorithm exposure_auto_algorithm = EXPOSURE_AUTO_ALGORITHM_MEAN;
        /** Sets the automatic exposure damping represented as %.
         * Maximum, Minimum: 99.6094, 0.390625 */
        double exposure_auto_damping = 89.8438;
        /** Sets the exposure auto limit auto exposure mode*/
        ExposureAutoLimitAuto exposure_auto_limit_auto = EXPOSURE_AUTO_LIMIT_AUTO_OFF;
        /** Controls the device exposure time.*/
        base::Time exposure_time = base::Time::fromMilliseconds(1);
        /** Minimum exposure time.*/
        base::Time min_exposure_time = base::Time::fromMicroseconds(46.912);
        /** Maximum exposure time.*/
        base::Time max_exposure_time = base::Time::fromSeconds(10);
        /** Width of the image provided by the device in pixels.*/
        int width = 2448;
        /** Height of the image provided by the device in pixels.*/
        int height = 2048;
        /** Horizontal offset from the origin to the region of interest in pixels.*/
        int offset_x = 0;
        /** Vertical offset from the origin to the region of interest in pixels.*/
        int offset_y = 0;
        /** Default pixel value the camera will try to reach automatically.
         * For uncontrolled outdoor environments, the recommended value is 70. */
        uint8_t target_brightness = 70;
    };

    struct CameraConfig {
        /** Camera's ip*/
        std::string ip;
        /** Perform factory reset? */
        bool factory_reset = false;
        /** Timeout after factory reset*/
        base::Time camera_reset_timeout = base::Time::fromSeconds(50);
        /** Period of info update (such as temperature, pressure...)*/
        base::Time update_info = base::Time::fromSeconds(1);
        /** Choose the temperature sensor*/
        DeviceTemperatureSelector temperature_selector =
            DeviceTemperatureSelector::DEVICE_TEMPERATURE_SELECTOR_SENSOR;
    };

    struct CameraInfo {
        /** Camera's temperature from Sensor mode*/
        base::Temperature temperature;
        /** Number of frame acquisition timeouts since start*/
        uint64_t acquisition_timeouts = 0;
        /** Number of incomplete images since start*/
        uint64_t incomplete_images = 0;
        /** Mean exposure time*/
        uint16_t mean_exposure; // the pattern may be different (unit64_t)
    };
}

#endif