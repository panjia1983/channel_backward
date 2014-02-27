require_relative 'model'
require 'csv'

selectorr = Record.where(version: 10702)

needles = selectorr

puts "#{selectorr.count} records in total"

selectorr.where(run_time: nil).each do |record|
begin
  record.run_time = record.result.scan(/elapsed time: (\d+\.\d+)/).first.first.to_f
  record.save!
rescue
  puts record.result
end
end
#
selectorr.where(converged: nil).each do |record|
  record.converged = record.result.scan(/status: converged/).count > 0
  record.save!
end
#
selectorr.where(converged:true).where(twist_costs: nil).each do |record|
  record.twist_costs = eval(record.result.scan(/twist costs: (.*)$/).first.first)
  record.save!
end

selectorr.where(converged:true).where(path_length_costs: nil).each do |record|
  record.path_length_costs = eval(record.result.scan(/path length costs: (.*)$/).first.first)
  record.save!
end

selectorr.where(converged:true).where(clearance_costs: nil).each do |record|
  record.clearance_costs = eval(record.result.scan(/clearance costs: (.*)$/).first.first)
  record.save!
end

selectorr.where(converged:true).where(n_multi_iterations: nil).each do |record|
  record.n_multi_iterations = record.result.scan(/n_multi_iterations: (.*)/).first.first.to_i#count > 0
  record.save!
end

#needles.where(converged:true).where(collision_free_cnt: nil).each do |record|
#  record.collision_free_cnt = record.result.scan(/is not in collision/).count
#  record.save!
#end

#needles.where(collision_free_dis: nil).each do |record|
#  record.collision_free_dis = record.result.scan(/is not in collision. The distance to goal is (\d+\.\d+)/).map{|x| x.first.to_f}
#  record.save!
#end

#CSV.open("results_30.csv", "w") do |csv|
#  csv << %w[pg_name method separate_planning_first simultaneous_planning average_collision_free_count average_collision_free_distance_to_goal average_run_time]

#needles.pluck(:start_position_error_relax_x).uniq.sort.each do |start_position_error_relax_x|

runtime =[]
sdruntime = []
convcnt = []
pathcosts = []
sdpathcosts = []
twistcosts = []
sdtwistcosts = []

  selectorr.pluck(:pg_name).uniq.each do |pg_name|
    [2].each do |method|
      [[1, 0]].each do |separate_planning_first, simultaneous_planning|
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
        run_times = needles.where(range).where(converged:true).pluck(:run_time)
        path_costs = needles.where(range).where(converged:true).pluck(:path_length_costs).map(&:sum)
        twist_costs = needles.where(range).where(converged:true).pluck(:twist_costs).map(&:sum)
        clearance_costs = needles.where(range).where(converged:true).pluck(:clearance_costs).map(&:sum)

        #total_run_time = run_times.sum
        #total_multi_iterations = needles.where(range).pluck(:n_multi_iterations).sum
        #total_path_costs = path_costs.sum
        #total_twist_costs = twist_costs.sum
        #total_clearance_costs = clearance_costs.sum

        #puts "avg collision free cnt: #{total_collision_free_cnt * 1.0 / cnt}"
        #puts "avg collision free dis: #{total_collision_free_dis * 1.0 / total_collision_free_cnt}"
        #puts "avg run time: #{total_run_time * 1.0 / cnt}"
        #puts "avg converged run time: #{(total_run_time*1.0/converged_cnt).round(3)}"
        #puts "avg converged cnt: #{(converged_cnt*1.0/cnt).round(3)}"
        #puts "avg multi iterations: #{(total_multi_iterations*1.0/cnt).round(3)}"
        runtime << run_times.mean.round(3)
        sdruntime << run_times.standard_deviation.round(3)
        convcnt << (converged_cnt*1.0/cnt).round(3)
        pathcosts << (path_costs.mean).round(3)#(total_path_costs*1.0/converged_cnt*2).round(3)
        sdpathcosts << path_costs.standard_deviation.round(3)
        twistcosts << twist_costs.mean.round(3)#(total_twist_costs*1.0/converged_cnt).round(3)
        sdtwistcosts << twist_costs.standard_deviation.round(3)

        #ccosts << -(clearance_costs.mean/collision_clearance_coeff*2).round(3)#-(total_clearance_costs*1.0/converged_cnt / collision_clearance_coeff*2).round(3)
        #sdccosts << (clearance_costs.standard_deviation/collision_clearance_coeff*2).round(3)


        #csv << [pg_name, method, separate_planning_first, simultaneous_planning, total_collision_free_cnt * 1.0 / cnt,
        #  total_collision_free_dis * 1.0 / total_collision_free_cnt,
        #  total_run_time * 1.0 / cnt
        #]
      end
    end
  end
  #end
puts "runtime"
  puts runtime.zip(sdruntime).map{|x,y| "\\pbox{15cm}{$#{x}$\\\\$\\pm#{y}$}"}.join("&")
  puts "convcnt"
  puts convcnt.map{|x| "#{(x*100).round(2)}\\%"}.join("&")
  puts "pathcosts"
  puts pathcosts.zip(sdpathcosts).map{|x,y| "\\pbox{15cm}{$#{x}$\\\\$\\pm#{y}$}"}.join("&")
  puts "twistcosts"
  puts twistcosts.zip(sdtwistcosts).map{|x,y| "\\pbox{15cm}{$#{x}$\\\\$\\pm#{y}$}"}.join("&")


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
