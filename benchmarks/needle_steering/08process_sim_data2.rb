require_relative 'model'
require 'csv'

selectorr = Record.where(version: 10802)

needles = selectorr

puts "#{selectorr.count} records in total"

selectorr.where(run_time: nil).each do |record|
begin
  record.run_time = record.result.scan(/elapsed time: (.*)$/).first.first.to_f
  record.save!
rescue
  puts record.result
  puts record.command
end
end

#selectorr.where(collision_status: nil).each do |record|
selectorr.each do |record|
  record.collision_status = eval(record.result.scan(/collision status: (.*)$/).first.first)
  record.save!
end

#selectorr.where(distance_to_goals: nil).each do |record|
selectorr.each do |record|
  record.distance_to_goals = eval(record.result.scan(/distance to goals: (.*)$/).first.first)
  record.save!
end
selectorr.each do |record|
  record.distance_to_goal = eval(record.result.scan(/distance to goals: (.*)$/).first.first).first
  record.save!
end

#needles.where(collision_free_cnt: nil).each do |record|
#  record.collision_free_cnt = record.result.scan(/is not in collision/).count
#  record.save!
#end
#
#needles.where(collision_free_dis: nil).each do |record|
#  record.collision_free_dis = record.result.scan(/is not in collision. The distance to goal is (\d+\.\d+)/).map{|x| x.first.to_f}
#  record.save!
#end

#CSV.open("results_30.csv", "w") do |csv|
#  csv << %w[pg_name method separate_planning_first simultaneous_planning average_collision_free_count average_collision_free_distance_to_goal average_run_time]
runtime =[]
succnt= []
dists = []

[1, 10].each do |collision_clearance_coeff|
  ["needle_steering_small_noise"].each do |pg_name|
    [1, 2].each do |method|
      [[0, 1], [1, 0]].each do |separate_planning_first, simultaneous_planning|
        range = {
          pg_name: pg_name,
          method: method,
          separate_planning_first: separate_planning_first,
          simultaneous_planning: simultaneous_planning,
          collision_clearance_coeff: collision_clearance_coeff,
        }
        puts range
        cnt = needles.where(range).count
        success_cnt = needles.where(range).where(collision_status: [0]).where(:distance_to_goal.lt => 0.125).count
        collision_free_cnt = needles.where(range).pluck(:collision_status).map(&:sum).select{|x| x==0}.count#where(converged:true).count
        distance_to_goals = needles.where(range).where(collision_status: [0]).pluck(:distance_to_goals).map(&:sum).sum
        #total_collision_free_cnt = needles.where(range).pluck(:collision_free_cnt).sum
        #total_collision_free_dis = needles.where(range).pluck(:collision_free_dis).flatten(1).sum
        total_run_time = needles.where(range).pluck(:run_time).sum
        #puts "avg collision free cnt: #{total_collision_free_cnt * 1.0 / cnt}"
        #puts "avg collision free dis: #{total_collision_free_dis * 1.0 / total_collision_free_cnt}"
        #puts "avg run time: #{total_run_time * 1.0 / cnt}"
        puts "avg run time: #{(total_run_time*1.0/cnt).round(3)}"
        puts "avg collision free cnt: #{(collision_free_cnt*1.0/cnt).round(3)}"
        puts "avg distance to goals: #{(distance_to_goals*1.0/cnt).round(3)}"
        puts "avg success cnt: #{(success_cnt*1.0/cnt).round(3)}"
        runtime << (total_run_time*1.0/cnt).round(3)
        succnt << (collision_free_cnt*1.0/cnt).round(3)
        dists << (distance_to_goals*1.0/cnt).round(3)
        #csv << [pg_name, method, separate_planning_first, simultaneous_planning, total_collision_free_cnt * 1.0 / cnt, 
        #  total_collision_free_dis * 1.0 / total_collision_free_cnt,
        #  total_run_time * 1.0 / cnt
        #]
      end
    end
  end
end
puts runtime.join("&")
puts succnt.map{|x| "#{(x*100).round(3)}\\%"}.join("&")
puts dists.join("&")
#  csv << %w[pg_name method separate_planning_first simultaneous_planning converged_percentage average_run_time]
#  channels.pluck(:pg_name).uniq.each do |pg_name|
#    [1, 2].each do |method|
#      [[1, 0]].each do |separate_planning_first, simultaneous_planning|
#        range = {
#          pg_name: pg_name,
#          method: method,
#          separate_planning_first: separate_planning_first,
#          simultaneous_planning: simultaneous_planning,
#        }
#        cnt = channels.where(range).count
#        total_converged_cnt = channels.where(range).where(converged: true).count
#        total_run_time = channels.where(range).pluck(:run_time).sum
#        #puts range.inspect
#        #puts "converged percentage: #{total_converged_cnt * 1.0 / cnt}"
#        #puts "avg run time: #{total_run_time * 1.0 / cnt}"
#        csv << [pg_name, method, separate_planning_first, simultaneous_planning, total_converged_cnt * 1.0 / cnt,
#          total_run_time * 1.0 / cnt
#        ]
#      end
#    end
#  end

#end
