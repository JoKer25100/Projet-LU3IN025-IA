# Projet "robotique" IA&Jeux 2025
#
# Binome:
#  Prénom Nom No_étudiant/e : Eric Xia 21315461
#  Prénom Nom No_étudiant/e : Lyes Kerar 21303638
#
# check robot.py for sensor naming convention
# all sensor and motor value are normalized (from 0.0 to 1.0 for sensors, -1.0 to +1.0 for motors)

from robot import * 
import math
nb_robots = 0

class Robot_player(Robot):

    team_name = "Chinoilgérie"  # vous pouvez modifier le nom de votre équipe
    robot_id = -1             # ne pas modifier. Permet de connaitre le numéro de votre robot.
    memory = 0                # vous n'avez le droit qu'a une case mémoire qui doit être obligatoirement un entier

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots+=1
        super().__init__(x_0, y_0, theta_0, name="Robot "+str(self.robot_id), team=self.team_name)

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        # # ====== 111111111111111 ======= : Braitenberg avancé 
        if (self.robot_id == 0):
        
            param = [2.583531080918049, 1.573652752928715, -0.013245493633874755, 1.1851956702927966, 0.6795780684809061, 2.3776050366976027, -1.651513145809548, -0.9421604469228071, 2.056316523042659]
            
            last_x = self.memory & 0x7FF             # 11 bits (0x7FF = 2047)
            last_y = (self.memory >> 11) & 0x7FF     # 11 bits
            stuck_count = (self.memory >> 22) & 0x3F # 6 bits (0x3F = 63)
            state_panic = (self.memory >> 28) & 0xF  # 4 bits pour le nbr de tours en PANIQUE (max 15)


            state = 1

            if (state_panic > 0):  
                state = 2 #Si on est tjr en état de panique

            else:
                # Distance parcourue depuis le dernier tour
                dist_moved = math.sqrt((self.x - last_x)**2 + (self.y - last_y)**2)

                # Si on a bougé de moins de 2 de distance, on incrémente le compteur de blocage
                if dist_moved < 2.0:
                    stuck_count += 1
                else:
                    stuck_count = 0 # Sinon, on reset le compteur

                # On vérifie si on est dans un espace confiné
                nbr_obstacle = 0
                for i in range (0,8):
                    if (sensor_view[i] == 1):
                        nbr_obstacle += 1 # On compte le nombre de mur proche
                
                if (nbr_obstacle >= 5): # Si il y a au moins 2 murs, on est dans un couloir
                    state = 0
                else: 
                    state = 1 # Non, sinon logique


                # Si le compteur dépasse 10 , on active la PANIQUE
                if stuck_count > 45:
                    state = 2 # Mode Panique
                    stuck_count = 0 # reset
                    state_panic = 15 # On initialise le compteur d'etat bloqué
            
            translation = 0
            rotation = 0
            
            # Déplacements en environnement confiné (ex: labyrinthe)
            if state == 0:
                param = [2.583531080918049, 1.7535697585711447, -0.013245493633874755, 3.676352490496156, 2.2860174150178665, 2.314132125663055, -1.651513145809548, -0.9421604469228071, 1.9764587899426929]
            
            # Déplacements en environnement ouvert (ex: parc)
            if state == 1:
                param = [0.8324708463678093, 0.8082919947609261, 1.96488437963744, 1.8982481891621745, 0.26958828599787027, 1.3558079256634408, -0.47594092823398554, -1.2090685610330016, 1.660301143992991]

            # PANIQUE (Dégagement)
            elif state == 2:
                # On inverse toute les datas et on prie
                param = [-2.583531080918049, 1.573652752928715, -0.013245493633874755, 1.1851956702927966, 0.6795780684809061, 2.3776050366976027, -1.651513145809548, -0.9421604469228071, 2.056316523042659]
                
                stuck_count = 0 # On remet à 0 au cas où
                state_panic -= 1 # On décremente aussi 

            translation = math.tanh ( param[0] + param[1] * sensors[sensor_front_left] + param[2] * sensors[sensor_front] + param[3] * sensors[sensor_front_right] )
            rotation = math.tanh ( param[4] * sensors[sensor_left] + param[5] * sensors[sensor_front_left] + param[6] * sensors[sensor_right] + param[7] * sensors[sensor_front_right] + param[8]* (random.random()-0.5) )
            # On sauvegarde
            new_x = int(self.x)
            new_y = int(self.y)
            
            # On s'assure que ça ne dépasse pas 4095 (limite 12 bits)
            new_x = min(new_x, 2047)
            new_y = min(new_y, 2047)
            state_panic = min(state_panic, 15)
            stuck_count = min(stuck_count, 63)

            # (STATE << 28) | (COMPTEUR << 24) | (Y << 12) | X
            self.memory = (state_panic << 28) | (stuck_count << 22) | (new_y << 11) | new_x

        # ====== 2222222222222 ======= : Braitenberg Pur
        if (self.robot_id == 1):
            # Pur Braitenberg
            param = [2.4698686069070717, 2.083970640697243, -1.130872189858404, 0.6828508393833606, 1.264939634430942, 3.2779609346564578, -0.9428865106368316, -2.034742818248398]
            translation = math.tanh ( param[0] + param[1] * sensors[sensor_front_left] + param[2] * sensors[sensor_front] + param[3] * sensors[sensor_front_right] )
            rotation = math.tanh ( param[4] * sensors[sensor_left] + param[5] * sensors[sensor_front_left] + param[6] * sensors[sensor_right] + param[7] * sensors[sensor_front_right] )

        #Lyes :
        # ====== 33333333333333 ======= : Braitenberg linéaire

        if (self.robot_id == 2):
            
            last_x = self.memory & 0x7FF             # 11 bits (0x7FF = 2047)
            last_y = (self.memory >> 11) & 0x7FF     # 11 bits
            stuck_count = (self.memory >> 22) & 0x3F # 6 bits (0x3F = 63)
            state_panic = (self.memory >> 28) & 0xF  # 4 bits pour le nbr de tours en PANIQUE (max 15)

            state = 1

            if (state_panic > 0):  
                state = 0 #Si on est tjr en état de panique

            else:
                # Distance parcourue depuis le dernier tour
                dist_moved = math.sqrt((self.x - last_x)**2 + (self.y - last_y)**2)

                # Si on a bougé de moins de 2 de distance, on incrémente le compteur de blocage
                if dist_moved < 2.0:
                    stuck_count += 1
                else:
                    stuck_count = 0 # Sinon, on reset le compteur
                
                # Si le compteur dépasse 10 , on active la PANIQUE
                if stuck_count > 45:
                    state = 0 # Mode Panique
                    stuck_count = 0 # reset
                    state_panic = 15 # On initialise le compteur d'etat bloqué

            if state == 1 : 
                translation = 0.6
                rotation = (sensors[sensor_front_left] - sensors[sensor_front_right] + 0.5 * (sensors[sensor_left] - sensors[sensor_right]))
                #Compare l’espace libre à gauche et à droite et va là ou y en à le plus

                if sensors[sensor_front] < 0.25:# Détecte obstacle très proche
                    rotation += (sensors[sensor_left] - sensors[sensor_right]) * 1.2 #rotation plus rapide côté libre
                    translation = 0.3 #ralenti pour pas crash
                
                # poids angulaires des capteurs (sens trigonométrique)
                angular_weights = {
                    sensor_front: 0.0,
                    sensor_front_left: +0.5,
                    sensor_left: +1.0,
                    sensor_rear_left: +0.5,
                    sensor_rear: 0.0,
                    sensor_rear_right: -0.5,
                    sensor_right: -1.0,
                    sensor_front_right: -0.5
                }

                for i in range(8):
                    #robots
                    if sensor_view[i] == 2:
                        # robot adverse uniquement
                        if sensor_robot[i] != self.team_name:
                            proximity = 1.0 - sensors[i]  # plus proche => plus fort

                            #avancer vers l'ennemi
                            translation += proximity * 0.8

                            #tourner vers l'ennemi
                            rotation += proximity * angular_weights[i]

                # bornage des valeurs
                translation = max(-1.0, min(1.0, translation))
                rotation = max(-1.0, min(1.0, rotation))
            
            else: 
                param = [-2.583531080918049, 1.573652752928715, -0.013245493633874755, 1.1851956702927966, 0.6795780684809061, 2.3776050366976027, -1.651513145809548, -0.9421604469228071, 2.056316523042659]
                
                stuck_count = 0 # On remet à 0 au cas où
                state_panic -= 1 # On décremente aussi 

                translation = math.tanh ( param[0] + param[1] * sensors[sensor_front_left] + param[2] * sensors[sensor_front] + param[3] * sensors[sensor_front_right] )
                rotation = math.tanh ( param[4] * sensors[sensor_left] + param[5] * sensors[sensor_front_left] + param[6] * sensors[sensor_right] + param[7] * sensors[sensor_front_right] + param[8]* (random.random()-0.5) )

            # On sauvegarde
            new_x = int(self.x)
            new_y = int(self.y)
            
            # On s'assure que ça ne dépasse pas 4095 (limite 12 bits)
            new_x = min(new_x, 2047)
            new_y = min(new_y, 2047)
            state_panic = min(state_panic, 15)
            stuck_count = min(stuck_count, 63)

            # (STATE << 28) | (COMPTEUR << 24) | (Y << 12) | X
            self.memory = (state_panic << 28) | (stuck_count << 22) | (new_y << 11) | new_x

        # ====== 4444444444444444 ======= : Arbre de comportement
        if (self.robot_id == 3):
            # Exploration par défaut
            last_x = self.memory & 0x7FF             # 11 bits (0x7FF = 2047)
            last_y = (self.memory >> 11) & 0x7FF     # 11 bits
            stuck_count = (self.memory >> 22) & 0x3F # 6 bits (0x3F = 63)
            state_panic = (self.memory >> 28) & 0xF  # 4 bits pour le nbr de tours en PANIQUE (max 15)

            state = 1

            if (state_panic > 0):  
                state = 0 #Si on est tjr en état de panique

            else:
                # Distance parcourue depuis le dernier tour
                dist_moved = math.sqrt((self.x - last_x)**2 + (self.y - last_y)**2)

                # Si on a bougé de moins de 2 de distance, on incrémente le compteur de blocage
                if dist_moved < 2.0:
                    stuck_count += 1
                else:
                    stuck_count = 0 # Sinon, on reset le compteur
                
                # Si le compteur dépasse 10 , on active la PANIQUE
                if stuck_count > 55:
                    state = 0 # Mode Panique
                    stuck_count = 0 # reset
                    state_panic = 15 # On initialise le compteur d'etat bloqué

            if state == 1 : 
                translation = 0.7
                rotation = (
                    sensors[sensor_front_left] - sensors[sensor_front_right]
                    + 0.5 * (sensors[sensor_left] - sensors[sensor_right])
                )

                # évite les murs
                if sensors[sensor_front] < 0.3:
                    rotation += (sensors[sensor_left] - sensors[sensor_right]) * 1.5
                    translation = 0.3

                # --- FUITE DES ENNEMIS ---
                if sensor_view is not None and sensor_robot is not None:
                    for i in range(8):
                        if sensor_view[i] == 2 and sensor_robot[i] != self.team_name:
                            proximity = 1.0 - sensors[i]

                            # Fuite = signe inverse
                            translation -= proximity * 0.5
                            rotation -= proximity * 1.2 * (
                                1 if i in [sensor_left, sensor_front_left] else
                            -1 if i in [sensor_right, sensor_front_right] else 0
                            )

                # Bornage
                translation = max(-1.0, min(1.0, translation))
                rotation = max(-1.0, min(1.0, rotation))

            else: 
                param = [-2.583531080918049, 1.573652752928715, -0.013245493633874755, 1.1851956702927966, 0.6795780684809061, 2.3776050366976027, -1.651513145809548, -0.9421604469228071, 2.056316523042659]
                
                stuck_count = 0 # On remet à 0 au cas où
                state_panic -= 1 # On décremente aussi 

                translation = math.tanh ( param[0] + param[1] * sensors[sensor_front_left] + param[2] * sensors[sensor_front] + param[3] * sensors[sensor_front_right] )
                rotation = math.tanh ( param[4] * sensors[sensor_left] + param[5] * sensors[sensor_front_left] + param[6] * sensors[sensor_right] + param[7] * sensors[sensor_front_right] + param[8]* (random.random()-0.5) )

            # On sauvegarde
            new_x = int(self.x)
            new_y = int(self.y)
            
            # On s'assure que ça ne dépasse pas 4095 (limite 12 bits)
            new_x = min(new_x, 2047)
            new_y = min(new_y, 2047)
            state_panic = min(state_panic, 15)
            stuck_count = min(stuck_count, 63)

            # (STATE << 28) | (COMPTEUR << 24) | (Y << 12) | X
            self.memory = (state_panic << 28) | (stuck_count << 22) | (new_y << 11) | new_x
        
        return translation, rotation, False

