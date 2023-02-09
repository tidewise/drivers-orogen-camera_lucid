#ifndef camera_lucid_TYPES_HPP
#define camera_lucid_TYPES_HPP

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

    struct ImageConfig {
        /** Timeout for frame acquisition.*/
        base::Time frame_timeout;
        /** Specifies the frequency in which frames are acquired. Note that TriggerMode
         * must be off for this parameter to take effect.*/
        double frame_rate;
        /** Image format*/
        base::samples::frame::frame_mode_t format;
        /** Depth*/
        uint8_t depth;
        /** Sets the automatic exposure mode.*/
        ExposureAuto exposure_auto = EXPOSURE_AUTO_CONTINUOUS;
        /** Controls the device exposure time.*/
        base::Time exposure_time;
        /** Width of the image provided by the device in pixels.*/
        int width;
        /** Height of the image provided by the device in pixels.*/
        int height;
        /** Horizontal offset from the origin to the region of interest in pixels.*/
        int offset_x;
        /** Vertical offset from the origin to the region of interest in pixels.*/
        int offset_y;
    };
}

#endif
