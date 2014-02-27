require 'mongoid'

Mongoid.load!("./mongoid.yml", :development)

class Record
  include Mongoid::Document

  field :status
  field :run_time
  field :n_merit_increases
  field :n_qp_solves
  field :cost
  field :trust_shrink_ratio
  field :trust_expand_ratio
  field :converged_cnt
  field :converged
  field :collision_free_dis
  field :collision_free_cnt
  field :collision_status
  field :distance_to_goals
  field :distance_to_goal
  field :n_multi_iterations
  field :collision_clearance_coeff
  field :twist_costs
  field :path_length_costs
  field :clearance_costs
  field :clearance_diff
  field :use_colocation_correction
end

module Enumerable

    def sum
      self.inject(0){|accum, i| accum + i }
    end

    def mean
      self.sum/self.length.to_f
    end

    def sample_variance
      m = self.mean
      sum = self.inject(0){|accum, i| accum + (i-m)**2 }
      sum/(self.length - 1).to_f
    end

    def standard_deviation
      return Math.sqrt(self.sample_variance)
    end

end 
