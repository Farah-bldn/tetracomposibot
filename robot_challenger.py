# Projet "robotique" IA&Jeux 2025
#
# Binome:
#  Prénom Nom No_étudiant/e : Farah BELAIDOUNI 21311575
#  Prénom Nom No_étudiant/e : Mehdi RABEHI
#
# check robot.py for sensor naming convention
# all sensor and motor value are normalized (from 0.0 to 1.0 for sensors, -1.0 to +1.0 for motors)
from robot import *
import math
import random

nb_robots = 0

class Robot_player(Robot):

    team_name = "Challenger"
    robot_id = -1
    memory = 0  # seul entier autorisé

    # Meilleurs poids TP2 (robot #0 doit les utiliser)
    bestParam = [1, 1, 0, 1, -1, 1, 1, -1]

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a"):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1
        super().__init__(x_0, y_0, theta_0, name="Robot "+str(self.robot_id), team=self.team_name)

    # -----------------------
    # Braitenberg optimisé (TP2) - robot 0
    # -----------------------
    def braitenberg_optimized(self, sensors):
        fl = sensors[sensor_front_left]
        f  = sensors[sensor_front]
        fr = sensors[sensor_front_right]

        t = math.tanh(self.bestParam[0] + self.bestParam[1]*fl + self.bestParam[2]*f + self.bestParam[3]*fr)
        r = math.tanh(self.bestParam[4] + self.bestParam[5]*fl + self.bestParam[6]*f + self.bestParam[7]*fr)

        #vitesse élevée quasi tout le temps, sauf si un mur est proche devant (t < 0 => translation = 0.90 au lieu de 1.00)
        translation = 0.90 + 0.10 * max(0.0, t)
        rotation = 1.0 * r
        return max(0.0, min(1.0, translation)), max(-1.0, min(1.0, rotation))

    # -----------------------
    # Braitenberg 1 : wall-follow / coverage (main gauche ou main droite selon robot_id)
    # -----------------------
    def braitenberg_wallfollow(self, sensors, hand=+1):
        # hand = +1 => main gauche ; hand = -1 => main droite
        f  = sensors[sensor_front]
        fl = sensors[sensor_front_left]
        fr = sensors[sensor_front_right]
        l  = sensors[sensor_left]
        r  = sensors[sensor_right]

        # avancer quasi tout le temps
        translation = 0.95

        # objectif: rester à une distance moyenne du mur du côté choisi
        target = 0.55
        if hand == +1:
            # si l < target => trop près du mur gauche => rotation négative (vers la droite)
            follow = 1.6 * (l - target)
            bias = +0.12
        else:
            follow = -1.6 * (r - target)
            bias = -0.12

        # évitement frontal (sinon on se plante dans un coin)
        avoid = 2.2*(fl - fr)

        rotation = avoid + follow + bias

        # si le mur est proche devant, ralentir un peu pour “tourner propre”
        if f < 0.25:
            translation = 0.70

        return max(0.0, min(1.0, translation)), max(-1.0, min(1.0, rotation))

    # -----------------------
    # Braitenberg 2 : chase adversaire (si vu) sinon wall-follow
    # -----------------------
    def braitenberg_chase(self, sensors, sensor_view, sensor_team, fallback_hand):
        # Cherche un robot adverse dans les directions utiles
        candidates = [sensor_front_left, sensor_front, sensor_front_right, sensor_left, sensor_right]
        best_dir = None
        best_dist = 1.0

        if sensor_view is not None and sensor_team is not None:
            for d in candidates:
                if sensor_view[d] == 2 and sensor_team[d] != "n/a" and sensor_team[d] != self.team_name:
                    if sensors[d] < best_dist:
                        best_dist = sensors[d]
                        best_dir = d

        if best_dir is None:
            return self.braitenberg_wallfollow(sensors, hand=fallback_hand)

        # tourner vers l’ennemi
        if best_dir in [sensor_front_left, sensor_left]:
            rotation = +0.9
        elif best_dir in [sensor_front_right, sensor_right]:
            rotation = -0.9
        else:
            rotation = 0.0

        translation = 1.0
        return translation, rotation

    # -----------------------
    # Subsumption + anti-stuck + séparation alliés
    # -----------------------
    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        f  = sensors[sensor_front]
        fl = sensors[sensor_front_left]
        fr = sensors[sensor_front_right]

        # (0) MODE DÉBLOCAGE: si memory > 0, on tourne sur place quelques pas
        if self.memory > 0:
            self.memory -= 1
            # sens différent selon robot_id pour casser les symétries
            sign = +1 if (self.robot_id % 2 == 0) else -1
            return 0.15, 1.0 * sign, False
    
                # évite de relancer un déblocage immédiatement après
        if self.memory == 0 and random.random() < 0.02:
            return 1.0, (random.random()-0.5)*0.2, False

        # (1) anti-collision / anti-coin : si trop près devant => déclenche déblocage
        if f < 0.10 or fl < 0.08 or fr < 0.08:
            self.memory = 14
            return 0.20, (+1.0 if fl < fr else -1.0), False

        # (2) séparation alliés : si un allié est juste devant => on “décroche”
        if sensor_view is not None and sensor_team is not None:
            if sensor_view[sensor_front] == 2 and sensor_team[sensor_front] == self.team_name:
                self.memory = 10
                return 0.30, (+1.0 if (self.robot_id % 2 == 0) else -1.0), False

        # (3) robot 0 = comportement TP2 (obligatoire)
        if self.robot_id == 0:
            t, r = self.braitenberg_optimized(sensors)
            # petit bruit pour éviter les cycles parfaits
            r += (random.random() - 0.5) * 0.04
            return t, r, False

        # (4) rôles : deux main gauche, deux main droite + un chasseur
        # main: +1 gauche, -1 droite
        hand = +1 if (self.robot_id in [1, 3]) else -1

        # robot 2 = chasseur (sinon wall-follow)
        if self.robot_id == 2:
            t, r = self.braitenberg_chase(sensors, sensor_view, sensor_team, fallback_hand=hand)
        else:
            t, r = self.braitenberg_wallfollow(sensors, hand=hand)

        # micro bruit (désynchronisation)
        r += (random.random() - 0.5) * 0.03
        return t, r, False
