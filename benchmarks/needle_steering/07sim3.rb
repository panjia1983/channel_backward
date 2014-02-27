require 'parallel'
require_relative 'model'

#cnt = 0

#File.open('new_points_10000.txt').read.split("\n").map(&:split).each do |goal_trans_x, goal_trans_y, goal_trans_z|
#  cnt += 1
#  if cnt > 200
#    break
#  end


starts = [
  [-1.25,0.0,0,0,0,0],
  [1.25,0.0,0,0,0,0],
  [0.0,-1.25,0,0,0,0],
  [0.0,1.25,0,0,0,0],
  [-1.5556349186104046,-1.5556349186104048,0,0,0,0],
  [-2.12503681783595,-0.5694018992255463,0,0,0,0],
  [-2.1250368178359507,0.5694018992255448,0,0,0,0],
  [-1.555634918610405,1.5556349186104046,0,0,0,0],
  [1.4142135623730951,1.414213562373095,0,0,0,0],
  [2.0,0.0,0,0,0,0],
  [1.4142135623730951,-1.414213562373095,0,0,0,0],
]
goals = [
  [-1.25,0.0,7,-1.1780972450961724,-0.0,-0.0],
  [1.25,0.0,7,1.1780972450961724,0.0,0.0],
  [0.0,-1.25,7,-0.0,-1.1780972450961724,-0.0],
  [0.0,1.25,7,0.0,1.1780972450961724,0.0],
  [-1.5556349186104046,-1.5556349186104048,3.5,0.0,0.0,0.0],
  [-2.12503681783595,-0.5694018992255463,3.5,0.0,0.0,0.0],
  [-2.1250368178359507,0.5694018992255448,3.5,0.0,0.0,0.0],
  [-1.555634918610405,1.5556349186104046,3.5,0.0,0.0,0.0],
  [1.4142135623730951,1.414213562373095,3.5,0.0,0.39269908169872414,0.0],
  [2.0,0.0,3.5,0.0,0.39269908169872414,0.0],
  [1.4142135623730951,-1.414213562373095,3.5,0.0,0.39269908169872414,0.0],
]

      [1, 2].each do |method|
10.times do
  seed = rand(100000000)
  %w[needle_steering_channel].each do |pg_name|
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
          #collision_dist_pen: 0.1,
          #goal_vec: "#{goal_trans_x},#{goal_trans_y},#{goal_trans_z},0,1.57,0",
          #start_vec: "-11.17067,5.04934,0,0,1.57,0",
          #start_position_error_relax_x: 0.05,#start_position_error_relax_x,
          #start_position_error_relax_y: 1.25,
          #start_position_error_relax_z: 1.25,
          #start_orientation_error_relax: 0.08,
          #goal_distance_error_relax: 0.25,
          seed: seed,
        }

        command = "../../build/bin/#{pg_name}"
        exp_options.each do |k, v|
          command += " --#{k}=#{v}"
        end

        ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

        result = `#{command}`

        Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 10702, description: "11 channels test"
      end
    end
  end
end
