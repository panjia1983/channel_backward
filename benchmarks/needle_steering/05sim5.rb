require 'parallel'
require_relative 'model'

cnt = 0

#seeds = Record.where(version: 34).pluck(:seeds).uniq
#
#puts seeds.size
#exit(1)

File.open('new_points_10000.txt').read.split("\n").map(&:split).each do |goal_trans_x, goal_trans_y, goal_trans_z|
  cnt += 1
  if cnt > 200
    break
  end

  seed = rand(100000000)
  [1, 10].each do |collision_clearance_coeff|
  %w[needle_steering_same_penalty].each do |pg_name|
    [1, 2].each do |method|
      [[1, 0]].each do |separate_planning_first, simultaneous_planning|
        exp_options = {
          goal_orientation_constraint: 0,
          r_min: 4,
          curvature_constraint: 1,
          channel_planning: 0,
          method: method,
          separate_planning_first: separate_planning_first,
          simultaneous_planning: simultaneous_planning,
          T: 10,
          max_sequential_solves: 5,
          first_run_only: 1,
          data_dir: "../../data",
          env_file_path: "../../data/prostate2.env.xml",
          goal_vec: "#{goal_trans_x},#{goal_trans_y},#{goal_trans_z},0,1.57,0",
          start_vec: "-7.5,5.75,0,0,1.57,0",
          start_position_error_relax_x: 0.05,#start_position_error_relax_x,
          start_position_error_relax_y: 2.5,
          start_position_error_relax_z: 1.25,
          start_orientation_error_relax: 0.08,
          collision_clearance_coeff: collision_clearance_coeff,
          collision_dist_pen: 0.125,
          goal_distance_error_relax: 0.125,
          seed: seed,
        }

        command = "../../build/bin/#{pg_name}"
        exp_options.each do |k, v|
          command += " --#{k}=#{v}"
        end

        ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

        result = `#{command}`

        Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 10505
      end
    end
  end
  end
end
