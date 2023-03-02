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
        task.properties.camera_config.ip = "10.1.1.24"
        task.properties.camera_config.camera_reset_timeout = Time.at(50)
        task.properties.camera_config.update_info = Time.at(1)
        task.properties.image_config = Types.camera_lucid.ImageConfig.new
        task.properties.image_config.frame_timeout = Time.at(0.2)
        task.properties.image_config.frame_rate = 21.0
        task.properties.image_config.format = "MODE_BAYER_RGGB"
        task.properties.image_config.depth = 8
        task.properties.image_config.exposure_auto = "EXPOSURE_AUTO_OFF"
        task.properties.image_config.exposure_time = Time.at(0.008)
        task.properties.image_config.width = 2448
        task.properties.image_config.height = 2048
        task.properties.image_config.offset_x = 0
        task.properties.image_config.offset_y = 0

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
