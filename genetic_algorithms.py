from robot import * 
import math, random

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
    it_before_replay = 3000
    replay_reset = 1000

    bestScore = -100000000
    bestid = -1
    score = 0

    last_trans = 0
    last_rot = 0

    eval_par_iter=3
    essai=0

    no_change = 0.
    #Perso

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a",evaluations=0,it_per_evaluation=0):
        global nb_robots

        self.robot_id = nb_robots
        nb_robots += 1
        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0
        self.visited_cells = set()

        self.param =[0.8324708463678093, 0.8082919947609261, 1.96488437963744, 1.8982481891621745, 0.26958828599787027, 1.3558079256634408, -0.47594092823398554, -1.2090685610330016, 1.660301143992991]#[random.uniform(-1, 1) for i in range(9)]
        
        self.bestParam = self.param[:] 
        
        self.it_per_evaluation = it_per_evaluation
        super().__init__(x_0, y_0, theta_0, name=name, team=team)
        
        self.last_trans = 0
        self.last_rot = 0



    def reset(self):
        self.visited_cells = set()
        self.last_trans = 0 
        self.last_rot= 0
        self.theta_0=random.uniform(0,360)
        super().reset()

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # cet exemple montre comment générer au hasard, et évaluer, des stratégies comportementales
        # Remarques:
        # - la liste "param", définie ci-dessus, permet de stocker les paramètres de la fonction de contrôle
        # - la fonction de controle est une combinaison linéaire des senseurs, pondérés par les paramètres (c'est un "Perceptron")

        # toutes les X itérations: le robot est remis à sa position initiale de l'arène avec une orientation aléatoire
        
        # CORRECTION 3 (MAJEURE) : Le calcul du score doit être EN HAUT !
        # Avant, il était après le return, donc le dernier pas était perdu.
        """
        self.score += (self.log_sum_of_translation - self.last_trans) * (1 - abs(self.log_sum_of_rotation - self.last_rot))
        self.last_trans = self.log_sum_of_translation
        self.last_rot = self.log_sum_of_rotation
        """
        grid_x = int(self.x / 2)
        grid_y = int(self.y / 2)

        # On ajoute la case à l'ensemble (les doublons sont ignorés automatiquement par le set)
        self.visited_cells.add((grid_x, grid_y))
        
        if self.iteration % self.it_per_evaluation == 0:
            if self.iteration > 0:
                print ("\tparameters           =",self.param)
                print ("\ttranslations         =",self.log_sum_of_translation,"; rotations =",self.log_sum_of_rotation) # *effective* translation/rotation (ie. measured from displacement)
                dist_run = math.sqrt((self.x-self.x_0)**2+(self.y-self.y_0)**2)
                print ("\tdistance from origin =", dist_run)
                #Perso
                self.score = len(self.visited_cells)
                print (f"\t score essai {self.trial} : {self.score}")
                if (self.replay == False):
                    self.essai+=1
                    if (self.essai == self.eval_par_iter):
                        if self.score > self.bestScore:
                            self.no_change =0
                            self.bestScore = self.score
                            self.bestParam = self.param[:]
                            self.bestid = self.trial
                        self.no_change += 1
                        self.trial = self.trial + 1
                        print ("\tbest score actuel",self.bestScore)
                        print ("\tbest param actuel", self.bestParam)
                        print ("\tbest id actuel", self.bestid)
                        self.param = self.bestParam[:]
                        self.param[random.randint(0,8)] += random.gauss(0, 0.3+ self.no_change/200)
                        self.essai = 0
                        self.score = 0

                #Perso
            if (self.trial == self.it_before_replay):
                print("Mode Infini")
                self.replay = True
                self.it_per_evaluation = self.replay_reset
                self.param = self.bestParam

            print ("Trying strategy no.",self.trial, ",", self.essai)
            self.iteration = self.iteration + 1

            return 0, 0, True # ask for reset

        # fonction de contrôle (qui dépend des entrées sensorielles, et des paramètres)
        translation = math.tanh ( self.param[0] + self.param[1] * sensors[sensor_front_left] + self.param[2] * sensors[sensor_front] + self.param[3] * sensors[sensor_front_right] )
        rotation = math.tanh ( self.param[4] * sensors[sensor_left] + self.param[5] * sensors[sensor_front_left] + self.param[6] * sensors[sensor_right] + self.param[7] * sensors[sensor_front_right] + self.param[8]* (random.random()-0.5) )
        
        if debug == True:
            if self.iteration % 100 == 0:
                print ("Robot",self.robot_id," (team "+str(self.team_name)+")","at step",self.iteration,":")
                print ("\tsensors (distance, max is 1.0)  =",sensors)
                print ("\ttype (0:empty, 1:wall, 2:robot) =",sensor_view)
                print ("\trobot's name (if relevant)      =",sensor_robot)
                print ("\trobot's team (if relevant)      =",sensor_team)

        self.iteration = self.iteration + 1        

        return translation, rotation, False
