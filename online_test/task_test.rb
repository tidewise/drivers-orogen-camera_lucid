# frozen_string_literal: true

using_task_library "camera_lucid"
import_types_from "camera_lucid"
import_types_from "base"

describe OroGen.camera_lucid.Task do
    run_live

    it "starts and gets a frame" do
        task = create_configure_and_start_task

        output = expect_execution.timeout(0.5).to do
            # have_one_new_sample(task.frame_port)
        end

        # puts "Size: "
        # pp output.size
        # puts "Frame mode: "
        # pp output.frame_mode
        # puts "Data depth:"
        # pp output.data_depth
        STDIN.readline
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

        task.properties.ip = "10.1.1.56"
        task.properties.frame_timeout = Time.at(0.2)
        task.properties.frame_rate = 15.0
        task.properties.binning_selector = "Digital"
        task.properties.binning_type = "Sum"
        task.properties.binning_x = 1
        task.properties.binning_y = 1
        task.properties.decimation_selector = "Sensor"
        task.properties.decimation_type = "Discard"
        task.properties.decimation_x = 1
        task.properties.decimation_y = 1
        task.properties.camera_format = "MODE_BAYER_RGGB"
        task.properties.depth = 8
        task.properties.auto_exposure = "Continuous"
        task.properties.exposure_time = Time.at(0.008)
        task.properties.width = 1224
        task.properties.height = 1024
        task.properties.offset_x = 0
        task.properties.offset_y = 0

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
