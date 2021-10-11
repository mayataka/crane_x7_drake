#include "crane_x7_manipulation/crane_x7_manipulation.hpp"


namespace crane_x7_drake {

CraneX7Manipulation::CraneX7Manipulation() {
}


CraneX7Manipulation::~CraneX7Manipulation() {
}


void CraneX7Manipulation::buildManipulation() {
  plant_ = drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0 /+ time_step +/);
plant.DoFoo(...);
}


} // namespace crane_x7_drake