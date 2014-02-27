require 'parallel'
require_relative 'model'

#cnt = 0

#File.open('new_points_10000.txt').read.split("\n").map(&:split).each do |goal_trans_x, goal_trans_y, goal_trans_z|
#  cnt += 1
#  if cnt > 200
#    break
#  end

channels = [
  [1.5,-0.8,7.7,0.057986926180183501,2.2213468852597504,-1.8707395110547083],
  [0,1.2,8,1.1843099322978712,1.5281438610198439,0.9098796369544363],
  [-1,-1.7,4.1,0,0,0],
  [0,-1.7,4.1,0,0,0],
  [1,-1.3,4.1,0,0,0],
  [1.8,-0.1,4.1,-1.0003996293652171,-2.6885743157215471,-0.72685375771090921],
  [0,1.5,3.2,1.0926834893111241,2.0289658614043877,0.8693869151532162],
  [-1.6,0.8,4.3,-0.8944770685034521,-2.6959058424480431,-1.1984222239511726],
]

starts = channels.map{|x,y,z,a,b,c| [x,y,z,0,0,0]}


      [1, 2].each do |method|
10.times do
  seed = rand(100000000)
  %w[needle_steering_10701].each do |pg_name|
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
          max_sequential_solves: 20,
          first_run_only: 1,
          total_curvature_limit: 1.57,
          data_dir: "../../data",
          start_vec: starts.map{|x| x.join(",")},
          goal_vec: channels.map{|x| x.join(",")},
          start_position_error_relax_x: [2.5] * starts.size,
          start_position_error_relax_y: [2.5] * starts.size,
          start_position_error_relax_z: [0.1] * starts.size,
          start_orientation_error_relax: [0.1744] * starts.size,
          goal_distance_error_relax: [0] * starts.size,

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

        puts 

        ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

        result = `#{command}`

        Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 10702, description: "8 channels test"
      end
    end
  end
end
