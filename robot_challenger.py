# robot_challenger.py
# Projet "robotique" IA & Jeux 2025
#
# Binome:
#  Prénom Nom No_étudiant/e : Farah BELAIDOUNI 21311575
#  Prénom Nom No_étudiant/e : Mehdi RABEHI 21208689

from robot import *
import math
import random

nb_robots = 0

class Robot_player(Robot):

    team_name = "YO'S TEAM"
    robot_id = -1
    memory = 0       # UNE SEULE CASE MEMOIRE AUTORISEE (sert à gérer les rotations de déblocage)
    turn_dir = +1    # Direction du virage en mémoire (1 = gauche, -1 = droite)

    # Paramètres Braitenberg optimisés (TP2)
    # 4 premiers : influence sur la translation
    # 4 derniers : influence sur la rotation
    bestParam = [1, 1, 0, 1, -1, 1, 1, -1]

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1
        super().__init__(x_0, y_0, theta_0,
                         name="Robot "+str(self.robot_id),
                         team=self.team_name)

    # -----------------------------------
    # Braitenberg TP2 optimisé (Robot 0)
    # Robot rapide pour couvrir le terrain
    # -----------------------------------
    def braitenberg_optimized(self, sensors):
        fl = sensors[sensor_front_left]
        f  = sensors[sensor_front]
        fr = sensors[sensor_front_right]

        # Calcul translation et rotation via tanh (valeurs entre -1 et 1)
        t = math.tanh(self.bestParam[0]
                     + self.bestParam[1]*fl
                     + self.bestParam[2]*f
                     + self.bestParam[3]*fr)
        r = math.tanh(self.bestParam[4]
                     + self.bestParam[5]*fl
                     + self.bestParam[6]*f
                     + self.bestParam[7]*fr)

        # Vitesse élevée presque tout le temps
        translation = 0.90 + 0.10 * max(0.0, t)

        # Rotation + petit bruit pour éviter les cycles
        rotation = r + (random.random()-0.5)*0.05
        return max(0.0, min(1.0, translation)), max(-1.0, min(1.0, rotation))

    # -----------------------------------
    # Wall-follow simple et modulaire
    # Sert à couvrir les murs et les couloirs
    # -----------------------------------
    def braitenberg_wallfollow(self, sensors, hand):
        f  = sensors[sensor_front]
        l  = sensors[sensor_left]
        r  = sensors[sensor_right]
        fl = sensors[sensor_front_left]
        fr = sensors[sensor_front_right]

        # Vitesse constante élevée
        translation = 0.9
        target = 0.55  # distance idéale au mur

        # Suivi de mur selon la main
        if hand == +1:  # main gauche
            follow = 1.8 * (l - target)
            bias = +0.25
        else:           # main droite
            follow = -1.8 * (r - target)
            bias = -0.25

        # Évitement frontal
        avoid = 2.2 * (fl - fr)

        # Rotation finale = suivi mur + évitement + biais
        rotation = follow + avoid + bias

        # Si mur proche devant → accélérer pour tourner plus vite
        if f < 0.25:
            translation = 1

        # Limitation de la rotation pour éviter les oscillations
        rotation = max(-0.5, min(0.5, rotation))

        return translation, rotation

    # -----------------------------------
    # Chasseur / Pénétrateur
    # Cherche et pousse les ennemis
    # -----------------------------------
    def braitenberg_chase(self, sensors, sensor_view, sensor_team, fallback_hand=-1):

        # Si un ennemi est détecté, priorité à la chasse
        if sensor_view is not None and sensor_team is not None:
            if sensor_view[sensor_front] == 2 and sensor_team[sensor_front] != self.team_name:
                return 1.0, 0.0  # fonce tout droit
            if sensor_view[sensor_left] == 2 and sensor_team[sensor_left] != self.team_name:
                return 0.6, 0.5  # tourne vers la gauche
            if sensor_view[sensor_right] == 2 and sensor_team[sensor_right] != self.team_name:
                return 0.6, -0.5 # tourne vers la droite

        # Sinon, comportement normal : suivi de mur
        return self.braitenberg_wallfollow(sensors, hand=fallback_hand)

    # -----------------------------------
    # Explorateur libre (robot 3)
    # Se déplace dans les zones ouvertes
    # -----------------------------------
    def braitenberg_explorer(self, sensors):
        f  = sensors[sensor_front]
        fl = sensors[sensor_front_left]
        fr = sensors[sensor_front_right]
        l  = sensors[sensor_left]
        r  = sensors[sensor_right]

        # Vitesse proportionnelle à l'espace devant
        t = 0.2 + 0.6 * f

        # Rotation vers la zone la plus libre + bruit
        rot = 0.25 * (l + fl) - 0.25 * (r + fr) + (random.random() - 0.5) * 0.6

        t = max(0.2, min(1.0, t))
        rot = max(-1.0, min(1.0, rot))
        return t, rot

    # -----------------------------------
    # Step principal (architecture subsomption)
    # Gère les priorités des comportements
    # -----------------------------------
    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):
        f  = sensors[sensor_front]
        fl = sensors[sensor_front_left]
        fr = sensors[sensor_front_right]
        l  = sensors[sensor_left]
        r  = sensors[sensor_right]
        rear = sensors[sensor_rear]

        # --------------------------------------------------
        # (0) ROTATION EN COURS (mémoire active)
        # Sert à sortir d’un blocage
        # --------------------------------------------------
        if self.memory != 0:
            rotation = 1.0 if self.memory > 0 else -1.0
            self.memory -= 1 if self.memory > 0 else -1
            return 0.25, rotation, False

        # --------------------------------------------------
        # (1) ANTI-COLLISION PRIORITAIRE
        # Si mur trop proche, on déclenche un déblocage
        # --------------------------------------------------
        if f < 0.10 or fl < 0.08 or fr < 0.08 or l < 0.08 or r < 0.08:
            self.memory = 12
            rot = +1.0 if fl < fr else -1.0
            return 0.3, rot, False

        # --------------------------------------------------
        # (2) SEPARATION ALLIES
        # Évite que deux robots de la même équipe se bloquent
        # --------------------------------------------------
        if sensor_view is not None and sensor_team is not None:
            if sensor_view[sensor_front] == 2 and sensor_team[sensor_front] == self.team_name:
                self.turn_dir = +1 if (random.random() < 0.5) else -1
                self.memory = 10 * self.turn_dir
                rotation = 1.0 if self.memory > 0 else -1.0
                return 0.25, rotation, False

        # --------------------------------------------------
        # (3) ROBOT 0 = COMPORTEMENT TP2 OBLIGATOIRE
        # Explorateur rapide
        # --------------------------------------------------
        if self.robot_id == 0:
            t, rot = self.braitenberg_optimized(sensors)
            rot += (random.random() - 0.5) * 0.04
            return t, rot, False

        # --------------------------------------------------
        # (4) REPARTITION DES ROLES
        # Chaque robot a une mission différente
        # --------------------------------------------------
        if self.robot_id == 3:
            # Robot suiveur de mur gauche
            t, rot = self.braitenberg_wallfollow(sensors, hand=+1)

        elif self.robot_id == 1:
            # Robot chasseur
            t, rot = self.braitenberg_chase(sensors, sensor_view, sensor_team, fallback_hand=-1)

        else:
            # Robot explorateur libre
            t, rot = self.braitenberg_explorer(sensors)

        # Petit bruit pour éviter synchronisation parfaite
        rot += (random.random() - 0.5) * 0.03

        return t, rot, False
