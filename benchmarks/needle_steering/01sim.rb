require 'parallel'
require_relative 'model'

100.times do

  # channel planning

  %w[needle_steering_separate_penalty needle_steering_same_penalty].each do |pg_name|
    [1, 2].each do |method|
      [[1, 0]].each do |separate_planning_first, simultaneous_planning|
        exp_options = {
          goal_orientation_constraint: 1,
          r_min: 1,
          curvature_constraint: 2,
          channel_planning: 1,
          method: method,
          separate_planning_first: separate_planning_first,
          simultaneous_planning: simultaneous_planning,
          T: 15,
          max_sequential_solves: 10,
          first_run_only: 1,
          total_curvature_limit: 1.57,
          data_dir: "../../data",
        }

        command = "TRAJOPT_LOG_THRESH=FATAL time ../../build/bin/#{pg_name}"
        exp_options.each do |k, v|
          command += " --#{k}=#{v}"
        end
        command += " 2>&1"

        result = `#{command}`

        Record.create! exp_options.merge result: result, pg_name: pg_name, problem: :channel_planning, version: 30, description: "hybrid testing for needle steering and channel planning"

        puts result.split("\n").reverse.take(5)
      end
    end
  end

  # needle steering

  %w[needle_steering_separate_penalty needle_steering_same_penalty].each do |pg_name|
    [1, 2].each do |method|
      [[1, 0], [0, 1], [1, 1]].each do |separate_planning_first, simultaneous_planning|
        exp_options = {
          goal_orientation_constraint: 0,
          r_min: 5,
          curvature_constraint: 1,
          channel_planning: 0,
          method: method,
          separate_planning_first: separate_planning_first,
          simultaneous_planning: simultaneous_planning,
          T: 10,
          max_sequential_solves: 3,
          data_dir: "../../data",
        }

        command = "TRAJOPT_LOG_THRESH=FATAL time ../../build/bin/#{pg_name}"
        exp_options.each do |k, v|
          command += " --#{k}=#{v}"
        end
        command += " 2>&1"

        result = `#{command}`

        Record.create! exp_options.merge result: result, pg_name: pg_name, problem: :needle_steering, version: 30, description: "hybrid testing for needle steering and channel planning"

        puts result.split("\n").reverse.take(5)
      end
    end
  end
end
