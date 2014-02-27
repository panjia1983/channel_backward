require 'parallel'
require_relative 'model'

cnt = 0

each_converged = []

[1, 2].each do |method|
  [[1, 0]].each do |separate_planning_first, simultaneous_planning|
    [1, 10].each do |collision_clearance_coeff|
      converged_points = Record.where(version: 10503,
                                      #collision_status: [0],#converged: true,
                                      converged: true,
                                      simultaneous_planning: simultaneous_planning,
                                      method: method,
                                      collision_clearance_coeff: collision_clearance_coeff
                                      )
                                      .pluck(:goal_vec).uniq#.count
      each_converged << converged_points
    end
  end
end

all_converged_pts = each_converged.inject(:&).map{|x|x.split(",")[0..2].map(&:to_f)}

#puts all_converged_pts[0]
#puts each_converged.inject(:&).count


#puts all_converged_pts.count
#exit(1)

#exit(1)

all_converged_pts.each do |goal_trans_x, goal_trans_y, goal_trans_z|

#File.open('new_points_10000.txt').read.split("\n").map(&:split).each do |goal_trans_x, goal_trans_y, goal_trans_z|
#  cnt += 1
#  if cnt > 200
#    break
#  end

  10.times do
    seed = rand(100000000)
    [1, 10].each do |collision_clearance_coeff|
      %w[needle_steering_small_noise].each do |pg_name|
        [1, 2].each do |method|
          [[1, 0], [0, 1]].each do |separate_planning_first, simultaneous_planning|
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
              goal_distance_error_relax: 0.03,
              collision_dist_pen: 0.125,
              seed: seed,
            }

            command = "../../build/bin/#{pg_name}"
            exp_options.each do |k, v|
              command += " --#{k}=#{v}"
            end

            ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

            result = `#{command}`

            Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 10805
          end
        end
      end
    end
  end
end
