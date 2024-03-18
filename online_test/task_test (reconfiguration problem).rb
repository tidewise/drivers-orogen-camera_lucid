# frozen_string_literal: true

# This test will reconfigure the camera multiple times until it breaks to test it
using_task_library "camera_lucid"
import_types_from "camera_lucid"
import_types_from "base"

describe OroGen.camera_lucid.Task do
    run_live

    it "works when reconfigured" do
        i = 1
        task = create_configure_and_start_task

        while i < 100
            expect_execution { task.stop! }.to { emit task.stop_event }
            task.needs_reconfiguration!

            task = create_configure_and_start_task
            output = expect_execution.to do
                [have_one_new_sample(task.frame_port),
                 have_one_new_sample(task.info_port)]
            end

            i += 1

            puts "Size: "
            pp output[0].size
            puts "Frame mode: "
            pp output[0].frame_mode
            puts "Data depth:"
            pp output[0].data_depth
            puts "Temperature:"
            pp output[1].temperature
        end
    end

    def create_task # rubocop:disable Metrics/AbcSize
        task = syskit_deploy(
            OroGen.camera_lucid
                  .Task
                  .deployed_as("camera_lucid_task")
        )

        task.properties.camera_config = {
            ip: "10.1.1.67",
            factory_reset: false,
            camera_reset_timeout: Time.at(50),
            temperature_selector: "DEVICE_TEMPERATURE_SELECTOR_SENSOR",
            update_info: Time.at(1)
        }

        task.properties.transmission_config = {
            auto_negotiate_mtu: true,
            explicit_data_transfer: false,
            mtu: 1_400,
            stream_packet_resend: true
        }

        task.properties.image_config = {
            frame_timeout: Time.at(0.2),
            frame_rate: 21.0,
            format: "MODE_BAYER_RGGB",
            depth: 8,
            exposure_auto: "EXPOSURE_AUTO_OFF",
            exposure_time: Time.at(0.001),
            exposure_auto_algorithm: :EXPOSURE_AUTO_ALGORITHM_MEAN,
            exposure_auto_damping: 89.8438,
            min_exposure_time: Time.at(0.000_046_912),
            max_exposure_time: Time.at(0.082_446_100),
            short_exposure_enable: false,
            width: 2448,
            height: 2048,
            offset_x: 0,
            offset_y: 0,
            target_brightness: 70
        }

        task.properties.analog_controller_config = {
            gain_auto: "GAIN_AUTO_OFF",
            gain_selector: "GAIN_SELECTOR_ALL",
            gain: 0,
            gain_min: 0,
            gain_max: 48,
            gamma_enabled: true,
            gamma: 0.5,
            balance_white_enable: true,
            balance_ratio_selector: "BALANCE_RATIO_SELECTOR_GREEN",
            balance_white_auto: "BALANCE_WHITE_AUTO_CONTINUOUS",
            balance_white_auto_anchor_selector:
                "BALANCE_WHITE_AUTO_ANCHOR_SELECTOR_MEANRGB"
        }

        task.properties.binning_config = {
            selector: "BINNING_SELECTOR_DIGITAL",
            horizontal_mode: "BINNING_MODE_SUM",
            vertical_mode: "BINNING_MODE_SUM",
            binning_x: 1,
            binning_y: 1

        }

        task.properties.decimation_config = {
            selector: "DECIMATION_SELECTOR_SENSOR",
            horizontal_mode: "DECIMATION_MODE_DISCARD",
            vertical_mode: "DECIMATION_MODE_DISCARD",
            decimation_x: 1,
            decimation_y: 1
        }

        task
    end

    def create_configure_and_start_task
        task = create_task
        syskit_configure(task)
        expect_execution { task.start! }.timeout(2).to do
            emit task.start_event
        end
        task
    end
end
