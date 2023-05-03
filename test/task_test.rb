# frozen_string_literal: true

using_task_library "camera_lucid"
import_types_from "camera_lucid"
import_types_from "base"

describe OroGen.camera_lucid.Task do
    run_live

    it "starts and gets a frame" do
        task = create_configure_and_start_task

        output = expect_execution.timeout(1.5).to do
            [have_one_new_sample(task.frame_port),
             have_one_new_sample(task.info_port)]
        end

        puts "Size: "
        pp output[0].size
        puts "Frame mode: "
        pp output[0].frame_mode
        puts "Data depth:"
        pp output[0].data_depth
        puts "Temperature:"
        pp output[1].temperature
        puts "Does the output looks OK ?"
        ask_ok
    end

    it "works when restarted" do
        task = create_configure_and_start_task
        expect_execution { task.stop! }.to { emit task.stop_event }

        task = create_configure_and_start_task
        output = expect_execution.to do
            [have_one_new_sample(task.frame_port),
             have_one_new_sample(task.info_port)]
        end

        puts "Size: "
        pp output[0].size
        puts "Frame mode: "
        pp output[0].frame_mode
        puts "Data depth:"
        pp output[0].data_depth
        puts "Temperature:"
        pp output[1].temperature
        puts "Does the output looks OK ?"
        ask_ok
    end

    it "works when reconfigured" do
        task = create_configure_and_start_task
        expect_execution { task.stop! }.to { emit task.stop_event }
        task.needs_reconfiguration!

        task = create_configure_and_start_task
        output = expect_execution.to do
            [have_one_new_sample(task.frame_port),
             have_one_new_sample(task.info_port)]
        end

        puts "Size: "
        pp output[0].size
        puts "Frame mode: "
        pp output[0].frame_mode
        puts "Data depth:"
        pp output[0].data_depth
        puts "Temperature:"
        pp output[1].temperature
        puts "Does the output looks OK ?"
        ask_ok
    end

    def ask_ok
        puts "OK ? (y/n)"
        value = STDIN.readline.chomp
        raise "test failed" unless value == "y"
    end

    def create_task
        task = syskit_deploy(
            OroGen.camera_lucid
                  .Task
                  .deployed_as("camera_lucid_task")
        )

        task.properties.camera_config = Types.camera_lucid.CameraConfig.new
        task.properties.camera_config.ip = "10.160.16.10"
        task.properties.camera_config.factory_reset = false
        task.properties.camera_config.camera_reset_timeout = Time.at(50)
        task.properties.camera_config
            .temperature_selector = "DEVICE_TEMPERATURE_SELECTOR_SENSOR"
        task.properties.camera_config.update_info = Time.at(1)

        task.properties.ptp_config.enable_ptp = true
        task.properties.ptp_config.slave_only = false

        task.properties.transmission_config.explicit_data_transfer = false
        task.properties.transmission_config.packet_delay = 0
        task.properties.transmission_config.frame_transmission_delay = 0

        task.properties.image_config = Types.camera_lucid.ImageConfig.new
        task.properties.image_config.frame_timeout = Time.at(0.2)
        task.properties.image_config.frame_rate = 21.0
        task.properties.image_config.format = "MODE_BAYER_RGGB"
        task.properties.image_config.depth = 8
        task.properties.image_config.exposure_auto = "EXPOSURE_AUTO_OFF"
        task.properties.image_config.exposure_time = Time.at(0.001)
        task.properties.image_config.min_exposure_time = Time.at(46.912)
        task.properties.image_config.max_exposure_time = Time.at(82_446.1)
        task.properties.image_config.width = 2448
        task.properties.image_config.height = 2048
        task.properties.image_config.offset_x = 0
        task.properties.image_config.offset_y = 0
        task.properties.image_config.target_brightness = 128

        task.properties.analog_controller_config = Types.camera_lucid
                                                        .AnalogControllerConfig.new
        task.properties.analog_controller_config.gain_auto = "GAIN_AUTO_OFF"
        task.properties.analog_controller_config.gain_selector = "GAIN_SELECTOR_ALL"
        task.properties.analog_controller_config.gain = 0
        task.properties.analog_controller_config.gain_min = 0
        task.properties.analog_controller_config.gain_max = 48.0
        task.properties.analog_controller_config.gamma_enabled = true
        task.properties.analog_controller_config.gamma = 0.5

        task.properties.binning_config = Types.camera_lucid.BinningConfig.new
        task.properties.binning_config.selector = "BINNING_SELECTOR_DIGITAL"
        task.properties.binning_config.horizontal_mode = "BINNING_MODE_SUM"
        task.properties.binning_config.vertical_mode = "BINNING_MODE_SUM"
        task.properties.binning_config.binning_x = 1
        task.properties.binning_config.binning_y = 1

        task.properties.decimation_config = Types.camera_lucid.DecimationConfig.new
        task.properties.decimation_config.selector = "DECIMATION_SELECTOR_SENSOR"
        task.properties.decimation_config.horizontal_mode = "DECIMATION_MODE_DISCARD"
        task.properties.decimation_config.vertical_mode = "DECIMATION_MODE_DISCARD"
        task.properties.decimation_config.decimation_x = 1
        task.properties.decimation_config.decimation_y = 1

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
