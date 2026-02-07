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
    it_before_replay = 1500
    replay_reset = 1000

    bestScore = -100000000
    bestid = -1
    score = 0

    last_trans = 0
    last_rot = 0

    eval_par_iter=3
    essai=0

    #Perso

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a",evaluations=0,it_per_evaluation=0):
        global nb_robots

        self.robot_id = nb_robots
        nb_robots += 1
        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0
        
        # CORRECTION 1 : Si vous utilisez 9 paramètres (avec le bruit), mettez range(9)
        self.param = [random.uniform(-1, 1) for i in range(9)]
        
        # CORRECTION 2 : Initialiser bestParam dès le début pour éviter les crashs
        self.bestParam = self.param[:] 
        
        self.it_per_evaluation = it_per_evaluation
        super().__init__(x_0, y_0, theta_0, name=name, team=team)
        
        # Initialisation correcte des variables de suivi
        self.last_trans = 0
        self.last_rot = 0



    def reset(self):
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
        self.score += (self.log_sum_of_translation - self.last_trans) * (1 - abs(self.log_sum_of_rotation - self.last_rot))
        self.last_trans = self.log_sum_of_translation
        self.last_rot = self.log_sum_of_rotation
        
        if self.iteration % self.it_per_evaluation == 0:
            if self.iteration > 0:
                print ("\tparameters           =",self.param)
                print ("\ttranslations         =",self.log_sum_of_translation,"; rotations =",self.log_sum_of_rotation) # *effective* translation/rotation (ie. measured from displacement)
                print ("\tdistance from origin =",math.sqrt((self.x-self.x_0)**2+(self.y-self.y_0)**2))
                #Perso
                print (f"\t score essai {self.trial} : {self.score}")
                if (self.replay == False):
                    self.essai+=1
                    if (self.essai == self.eval_par_iter):
                        if self.score > self.bestScore:
                            self.bestScore = self.score
                            self.bestParam = self.param[:]
                            self.bestid = self.trial
                        self.trial = self.trial + 1
                        print ("\tbest score actuel",self.bestScore)
                        print ("\tbest param actuel", self.bestParam)
                        print ("\tbest id actuel", self.bestid)
                        self.param = self.bestParam[:]
                        self.param[random.randint(0,8)] += random.gauss(0, 0.3)
                        self.essai = 0
                        self.score = 0

                #Perso
            if (self.trial == self.it_before_replay):
                print("Start Mode Infini")
                self.replay = True
                self.it_per_evaluation = self.replay_reset
                self.param = self.bestParam

            print ("Trying strategy no.",self.trial, ",", self.essai)
            self.iteration = self.iteration + 1

            return 0, 0, True # ask for reset

        # fonction de contrôle (qui dépend des entrées sensorielles, et des paramètres)
        noise = (random.random() - 0.5) * self.param[8] 
        translation = math.tanh(self.param[0] + self.param[1] * sensors[0] + self.param[2] * sensors[2] + self.param[3] * sensors[7])
        rotation = math.tanh(self.param[4] + self.param[5] * sensors[0] + self.param[6] * sensors[2] + self.param[7] * sensors[7] + noise)
        
        if debug == True:
            if self.iteration % 100 == 0:
                print ("Robot",self.robot_id," (team "+str(self.team_name)+")","at step",self.iteration,":")
                print ("\tsensors (distance, max is 1.0)  =",sensors)
                print ("\ttype (0:empty, 1:wall, 2:robot) =",sensor_view)
                print ("\trobot's name (if relevant)      =",sensor_robot)
                print ("\trobot's team (if relevant)      =",sensor_team)

        self.iteration = self.iteration + 1        

        return translation, rotation, False
