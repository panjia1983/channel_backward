require 'parallel'
require_relative 'model'

#cnt = 0

#File.open('new_points_10000.txt').read.split("\n").map(&:split).each do |goal_trans_x, goal_trans_y, goal_trans_z|
#  cnt += 1
#  if cnt > 200
#    break
#  end

channels = [
  [1.5, -0.4, 6.2, -1.05231, -0.494678, -0.306652],
  [0, 1.2, 7.5, 0.60843, -0.958753, -1.01405],
  [-1, -1.7, 4.1, -0, 0, -0.785398],
  [0, -1.7, 4.1, -0, 0, -0.785398],
  [1, -1.3, 4.1, -0, 0, -0.785398],
  [1.8, -0.6, 4.1, 0.156714, 0.0642851, -0.776742],
  [0, 1.5, 3.2, 0.720124, -0.720034, -0.905084],
  [-1.6, 0.8, 4.3, 0.811483, 0.0612882, -0.605674],
]#.map{|x,y,z,a,b,c| [x,y,z,-a,-b,-c]}

starts = channels.map{|x,y,z,a,b,c| [x,y,0,0,0,0]}


100.times do
[[1, 0]].each do |separate_planning_first, simultaneous_planning|
  %w[needle_steering].each do |pg_name|
    [2].each do |method|
      seed = rand(100000000)
      exp_options = {
        goal_orientation_constraint: 1,
        r_min: 1,
        curvature_constraint: 2,
        channel_planning: 1,
        method: method,
        separate_planning_first: separate_planning_first,
        simultaneous_planning: simultaneous_planning,
        T: 15,
        max_sequential_solves: 20,
        first_run_only: 1,
        total_curvature_limit: 1.57,
        data_dir: "../../data",
        start_vec: starts.map{|x| x.join(",")},
        goal_vec: channels.map{|x| x.join(",")},
        start_position_error_relax_x: [2.3] * starts.size,
        start_position_error_relax_y: [2.3] * starts.size,
        start_position_error_relax_z: [0.1] * starts.size,
        start_orientation_error_relax: [0.1744] * starts.size,
        goal_distance_error_relax: [0] * starts.size,
        #seq_result_plotting: 1,

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
        if v.respond_to? :each
          v.each do |v_val|
            command += " --#{k}=#{v_val}"
          end
        else
          command += " --#{k}=#{v}"
        end
      end

      ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

      result = `#{command}`

      Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 10702, description: "8 channels test"
    end
  end
end
end
#end
