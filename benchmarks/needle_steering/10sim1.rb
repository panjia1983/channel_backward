require 'parallel'
require_relative 'model'

pointstr = "-2.17475,6.99194,-1.61106,0,1.57,0"

point = pointstr.split(",").map(&:to_f)[0..2]
goal_trans_x, goal_trans_y, goal_trans_z = point


(0..5).step(0.5).each do |noise_scale|#(0..5).each do |noise_level| 
  seeds = Record.where(version: 11001, noise_scale: noise_scale).pluck(:seed).uniq
  seeds.each do |seed|
    #seed = rand(100000000)
    open_loop = 0
    method = 2
    use_colocation_correction = 1
    #[0, 1].each do |open_loop|
      [10].each do |collision_clearance_coeff|
        %w[needle_steering_11001_correction].each do |pg_name|
          #[1, 2].each do |method|
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
                collision_clearance_coeff: collision_clearance_coeff,
                #first_run_only: 1,
                data_dir: "../../data",
                goal_vec: "#{goal_trans_x},#{goal_trans_y},#{goal_trans_z},0,1.57,0",
                start_vec: "-7.5,5.75,0,0,1.57,0",
                start_position_error_relax_x: 0.05,#start_position_error_relax_x,
                start_position_error_relax_y: 2.5,
                start_position_error_relax_z: 1.25,
                start_orientation_error_relax: 0.08,
                goal_distance_error_relax: 0,
                collision_dist_pen: 0.125,
                use_colocation_correction: use_colocation_correction,
                open_loop: open_loop,
                seed: seed,
                noise_scale: noise_scale,
                #noise_level: noise_level,
              }

              command = "../../build/bin/#{pg_name}"
              exp_options.each do |k, v|
                command += " --#{k}=#{v}"
              end

              ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

              result = `#{command}`

              Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 11001
            end
          end
        end
      end
    end
#  end
#end

#100.times do
#  (0..5).step(0.5).each do |noise_scale|#(0..5).each do |noise_level| 
#    seed = rand(100000000)
#    [0, 1].each do |open_loop|
#      [10].each do |collision_clearance_coeff|
#        %w[needle_steering_11001].each do |pg_name|
#          [1, 2].each do |method|
#            [[1, 0]].each do |separate_planning_first, simultaneous_planning|
#              exp_options = {
#                goal_orientation_constraint: 0,
#                r_min: 4,
#                curvature_constraint: 1,
#                channel_planning: 0,
#                method: method,
#                separate_planning_first: separate_planning_first,
#                simultaneous_planning: simultaneous_planning,
#                T: 10,
#                max_sequential_solves: 5,
#                collision_clearance_coeff: collision_clearance_coeff,
#                #first_run_only: 1,
#                data_dir: "../../data",
#                goal_vec: "#{goal_trans_x},#{goal_trans_y},#{goal_trans_z},0,1.57,0",
#                start_vec: "-7.5,5.75,0,0,1.57,0",
#                start_position_error_relax_x: 0.05,#start_position_error_relax_x,
#                start_position_error_relax_y: 2.5,
#                start_position_error_relax_z: 1.25,
#                start_orientation_error_relax: 0.08,
#                goal_distance_error_relax: 0,
#                collision_dist_pen: 0.125,
#                open_loop: open_loop,
#                seed: seed,
#                noise_scale: noise_scale,
#                #noise_level: noise_level,
#              }
#
#              command = "../../build/bin/#{pg_name}"
#              exp_options.each do |k, v|
#                command += " --#{k}=#{v}"
#              end
#
#              ENV["TRAJOPT_LOG_THRESH"] = "FATAL"
#
#              result = `#{command}`
#
#              Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 11001
#            end
#          end
#        end
#      end
#    end
#  end
#end
