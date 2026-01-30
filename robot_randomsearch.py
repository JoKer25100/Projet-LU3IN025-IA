from robot import * 
import math

nb_robots = 0
debug = False

class Robot_player(Robot):

    team_name = "Optimizer"
    robot_id = -1
    iteration = 0

    param = []
    bestParam = []
    it_per_evaluation = 400
    trial = 0

    x_0 = 0
    y_0 = 0
    theta_0 = 0 # in [0,360]

    #Perso
    replay = False
    it_before_replay = 500
    replay_reset = 1000

    bestScore = -10000
    bestid = -1
    score = 0

    last_x = 0
    last_y = 0
    last_rotation = 0

    #Perso

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a",evaluations=0,it_per_evaluation=0):
        global nb_robots

        #Perso
        self.last_x = x_0
        self.last_y = y_0
        self.last_rotation = theta_0
        #Perso

        self.robot_id = nb_robots
        nb_robots+=1
        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0
        self.param = [random.randint(-1, 1) for i in range(8)]
        self.it_per_evaluation = it_per_evaluation
        super().__init__(x_0, y_0, theta_0, name=name, team=team)



    def reset(self):
        self.score = 0
        self.last_x = self.x_0
        self.last_y = self.y_0
        self.last_rotation = 0
        super().reset()

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # cet exemple montre comment générer au hasard, et évaluer, des stratégies comportementales
        # Remarques:
        # - la liste "param", définie ci-dessus, permet de stocker les paramètres de la fonction de contrôle
        # - la fonction de controle est une combinaison linéaire des senseurs, pondérés par les paramètres (c'est un "Perceptron")
        
        #Perso
        if(self.last_x != self.x or self.last_y != self.y):
             self.score += math.sqrt((self.x-self.last_x)**2+(self.y-self.last_y)**2) * (1 - abs(self.last_rotation))
        self.last_x = self.x
        self.last_y = self.y
        #Perso

        # toutes les X itérations: le robot est remis à sa position initiale de l'arène avec une orientation aléatoire
        if self.iteration % self.it_per_evaluation == 0:
                if self.iteration > 0:
                    print ("\tparameters           =",self.param)
                    print ("\ttranslations         =",self.log_sum_of_translation,"; rotations =",self.log_sum_of_rotation) # *effective* translation/rotation (ie. measured from displacement)
                    print ("\tdistance from origin =",math.sqrt((self.x-self.x_0)**2+(self.y-self.y_0)**2))
                    #Perso
                    print (f"\t score essai {self.trial} : {self.score}")
                    if (self.replay == False):
                        if self.score > self.bestScore:
                            self.bestScore = self.score
                            self.bestParam = self.param[:]
                            self.bestid = self.trial
                        print ("\tbest score actuel",self.bestScore)
                        print ("\tbest param actuel", self.bestParam)
                        print ("\tbest id actuel", self.bestid)
                    #Perso
                if (self.trial == self.it_before_replay):
                    print("Start Mode Infini")
                    self.replay = True
                    self.it_per_evaluation = self.replay_reset
                    self.param = self.bestParam

                if (self.replay == False):
                    self.param = [random.randint(-1, 1) for i in range(8)] 
                self.trial = self.trial + 1
                print ("Trying strategy no.",self.trial)
                self.iteration = self.iteration + 1

                return 0, 0, True # ask for reset

        # fonction de contrôle (qui dépend des entrées sensorielles, et des paramètres)
        translation = math.tanh ( self.param[0] + self.param[1] * sensors[sensor_front_left] + self.param[2] * sensors[sensor_front] + self.param[3] * sensors[sensor_front_right] )
        rotation = math.tanh ( self.param[4] + self.param[5] * sensors[sensor_front_left] + self.param[6] * sensors[sensor_front] + self.param[7] * sensors[sensor_front_right] )
        
        #Perso
        self.last_rotation = rotation
        #Perso

        if debug == True:
            if self.iteration % 100 == 0:
                print ("Robot",self.robot_id," (team "+str(self.team_name)+")","at step",self.iteration,":")
                print ("\tsensors (distance, max is 1.0)  =",sensors)
                print ("\ttype (0:empty, 1:wall, 2:robot) =",sensor_view)
                print ("\trobot's name (if relevant)      =",sensor_robot)
                print ("\trobot's team (if relevant)      =",sensor_team)

        self.iteration = self.iteration + 1        

        return translation, rotation, False
