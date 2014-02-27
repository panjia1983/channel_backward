require 'parallel'
require_relative 'model'

cnt = 0

points = []


def dis(p1, p2)
  p1.zip(p2).map{|x,y| (x-y) ** 2}.inject(:+) ** 0.5
end

def min_dis(pts)
  pts.combination(2).map{|pta, ptb| dis(pta, ptb)}.min
end

File.open('new_points_10000.txt').read.split("\n").map(&:split).each do |goal_trans_x, goal_trans_y, goal_trans_z|
  cnt += 1
  if cnt > 300
    break
  end
  points << [goal_trans_x.to_f, goal_trans_y.to_f, goal_trans_z.to_f]
end

200.times do

  pts = nil
  while pts.nil? or min_dis(pts) < 0.7
    pts = points.sample(5)
  end

  seed = rand(100000000)
  %w[needle_steering_special].each do |pg_name|
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
          max_sequential_solves: 10,
          first_run_only: 1,
          data_dir: "../../data",
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

        5.times do
          command += " --start_vec=-7.5,5.75,0,0,1.57,0"
          command += " --start_position_error_relax_x=0.05"
          command += " --start_position_error_relax_y=2.5"
          command += " --start_position_error_relax_z=1.25"
          command += " --start_orientation_error_relax=0.08"
          command += " --goal_distance_error_relax=0.125"
        end

        pts.each do |pt|
          x,y,z = pt
          command += " --goal_vec=#{x},#{y},#{z},0,1.57,0"
        end

        ENV["TRAJOPT_LOG_THRESH"] = "FATAL"

        result = `#{command}`

        Record.create! exp_options.merge command: command, result: result, pg_name: pg_name, problem: :needle_steering, version: 10602, description: "5 needles test"
      end
    end
  end

end
