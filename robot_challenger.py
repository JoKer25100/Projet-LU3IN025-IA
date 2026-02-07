# Projet "robotique" IA&Jeux 2025
#
# Binome:
#  Prénom Nom No_étudiant/e : _________
#  Prénom Nom No_étudiant/e : _________
#
# check robot.py for sensor naming convention
# all sensor and motor value are normalized (from 0.0 to 1.0 for sensors, -1.0 to +1.0 for motors)

from robot import * 
import math
nb_robots = 0

class Robot_player(Robot):

    team_name = "Challenger"  # vous pouvez modifier le nom de votre équipe
    robot_id = -1             # ne pas modifier. Permet de connaitre le numéro de votre robot.
    memory = 0                # vous n'avez le droit qu'a une case mémoire qui doit être obligatoirement un entier

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots+=1
        super().__init__(x_0, y_0, theta_0, name="Robot "+str(self.robot_id), team=self.team_name)
        self.param = [-0.13248843084020384, 1.6349839470483905, -2.187444562422601, 1.374552163249403, 0.21580525859103986, -0.03252974342242439, 0.9287265227365898, -0.5648002623113457, 0.11004282567894529]

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        # 4.
        translation = math.tanh(self.param[0] + self.param[1] * sensors[0] + self.param[2] * sensors[2] + self.param[3] * sensors[7])
        rotation = math.tanh(self.param[4] + self.param[5] * sensors[0] + self.param[6] * sensors[2] + self.param[7] * sensors[7] + (random.random() - 0.5) * self.param[8])
        
        return translation, rotation, False

