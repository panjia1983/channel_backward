require_relative 'model'
require 'csv'

selectorr = Record.where(version: 10607)

needles = selectorr

puts "#{selectorr.count} records in total"

selectorr.where(run_time: nil).each do |record|
  record.run_time = record.result.scan(/elapsed time: (.*)$/).first.first.to_f
  record.save!
end

selectorr.where(converged: nil).each do |record|
  record.converged = record.result.scan(/status: converged/).count > 0
  record.save!
end

selectorr.where(n_multi_iterations: nil).each do |record|
  record.n_multi_iterations = record.result.scan(/n_multi_iterations: (.*)/).first.first.to_i#count > 0
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
convcnt = []

#needles.pluck(:start_position_error_relax_x).uniq.sort.each do |start_position_error_relax_x|
  ["needle_steering_10607"].each do |pg_name|
    [1, 2].each do |method|
      [[0, 1], [1, 0]].each do |separate_planning_first, simultaneous_planning|
        range = {
          pg_name: pg_name,
          method: method,
          separate_planning_first: separate_planning_first,
          simultaneous_planning: simultaneous_planning,
          #start_position_error_relax_x: start_position_error_relax_x,#start_orientation_error_relax: start_orientation_error_relax,
        }
        puts range
        cnt = needles.where(range).count
        converged_cnt = needles.where(range).where(converged:true).count
        #total_collision_free_cnt = needles.where(range).pluck(:collision_free_cnt).sum
        #total_collision_free_dis = needles.where(range).pluck(:collision_free_dis).flatten(1).sum
        total_run_time = needles.where(range).pluck(:run_time).sum
        total_multi_iterations = needles.where(range).pluck(:n_multi_iterations).sum
        #puts "avg collision free cnt: #{total_collision_free_cnt * 1.0 / cnt}"
        #puts "avg collision free dis: #{total_collision_free_dis * 1.0 / total_collision_free_cnt}"
        #puts "avg run time: #{total_run_time * 1.0 / cnt}"
        puts "avg run time: #{(total_run_time*1.0/cnt).round(3)}"
        puts "avg converged cnt: #{(converged_cnt*1.0/cnt).round(3)}"
        puts "avg multi iterations: #{(total_multi_iterations*1.0/cnt).round(3)}"
        runtime << (total_run_time*1.0/cnt).round(3)
        convcnt << (converged_cnt*1.0/cnt).round(3)

        #csv << [pg_name, method, separate_planning_first, simultaneous_planning, total_collision_free_cnt * 1.0 / cnt, 
        #  total_collision_free_dis * 1.0 / total_collision_free_cnt,
        #  total_run_time * 1.0 / cnt
        #]
      end
    end
  end
  #end
puts runtime.join("&")
  puts convcnt.map{|x| "#{(x*100).round(3)}\\%"}.join("&")

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
