from pydrake.all import LeafSystem, BasicVector, JacobianWrtVariable
import numpy as np


class DiffIKController(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("crane_x7")
        self._G = plant.GetBodyByName("crane_x7_gripper_base_link").body_frame()
        self._W = plant.world_frame()

        self.DeclareVectorInputPort("crane_x7_state", BasicVector(18))
        self.DeclareVectorOutputPort("crane_x7_velocity", BasicVector(9), 
                                     self.CalcOutput)

    def CalcOutput(self, context, output):
        q = self.get_input_port().Eval(context)[0:9]
        self._plant.SetPositions(self._plant_context, self._iiwa, q)
        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kQDot, 
            self._G, [0,0,0], self._W, self._W)
        J_G = J_G[:,0:7] # Ignore gripper terms

        V_G_desired = np.array([0,    # rotation about x
                                -.1,  # rotation about y
                                0,    # rotation about z
                                0,    # x
                                -.05, # y
                                -.1]) # z
        v = np.linalg.pinv(J_G).dot(V_G_desired)
        output.SetFromVector(np.concatenate([v, np.zeros(2)]))

        