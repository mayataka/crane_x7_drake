#ifndef CRANE_X7_DRAKE__CRANE_X7_MANIPULATION_HPP_
#define CRANE_X7_DRAKE__CRANE_X7_MANIPULATION_HPP_

#include <memory>
#include <string>

#include "drake/systems/framework/builder.h"
#include "drake/systems/analysis/simulator.h"


namespace crane_x7_drake {

class CraneX7Manipulation {
public:
  CraneX7Manipulation();
  ~CraneX7Manipulation();

  void buildManipulation();

private:
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_; 
  double time_step_;

};

} // namespace crane_x7_drake


#endif // CRANE_X7_DRAKE__CRANE_X7_MANIPULATION_HPP_ 