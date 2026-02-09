from robot import *
import math

nb_robots = 0
debug = False

class Robot_player(Robot):

    team_name = "RandomSearch"
    robot_id = -1
    iteration = 0

    param = []
    bestParam = []
    it_per_evaluation = 400
    trial = 0

    x_0 = 0
    y_0 = 0
    theta_0 = 0 # in [0,360]

    def __init__(self, x_0, y_0, theta_0, name="n/a", team="n/a", evaluations=0, it_per_evaluation=0):
        global nb_robots
        self.robot_id = nb_robots
        nb_robots += 1

        self.x_0 = x_0
        self.y_0 = y_0
        self.theta_0 = theta_0

        # --- paramètres TP (budget)
        self.evaluations = evaluations if evaluations != 0 else 500
        self.it_per_evaluation = it_per_evaluation if it_per_evaluation != 0 else 400

        # --- individu courant
        self.param = [random.randint(-1, 1) for i in range(8)]

        # --- suivi meilleur
        self.bestScore = -1e18
        self.bestTrial = -1
        self.bestParam = []

        # --- score effectif (pas à pas)
        self.score = 0.0
        self.prevT = 0.0
        self.prevR = 0.0

        # --- phase 0: recherche / phase 1: replay du meilleur
        self.phase = 0
        self.demo_len = 1000
        self.demo_iter = 0

        super().__init__(x_0, y_0, theta_0, name=name, team=team)

    def reset(self):
        super().reset()
        # reset score tracking (important pour comparer correctement)
        self.score = 0.0
        self.prevT = 0.0
        self.prevR = 0.0

    def step(self, sensors, sensor_view=None, sensor_robot=None, sensor_team=None):

        # --- mise à jour score (effectif) : somme_t (dT * (1 - dR))
        # dT = déplacement réel durant ce pas (0 si collision / mur)
        # dR = abs(rotation) réel durant ce pas (déjà en abs dans log_sum_of_rotation)
        dT = self.log_sum_of_translation - self.prevT
        dR = self.log_sum_of_rotation - self.prevR
        self.prevT = self.log_sum_of_translation
        self.prevR = self.log_sum_of_rotation
        self.score += dT * (1.0 - dR)

        # --- toutes les X itérations: fin d'une évaluation
        if self.iteration > 0 and self.iteration % self.it_per_evaluation == 0:

            # affichage "prof" conservé
            print("\tparameters           =", self.param)
            print("\ttranslations         =", self.log_sum_of_translation, "; rotations =", self.log_sum_of_rotation)
            print("\tdistance from origin =", math.sqrt((self.x - self.x_0)**2 + (self.y - self.y_0)**2))

            # score + best
            print("\tscore                =", self.score)

            # sauvegarde du meilleur
            if self.score > self.bestScore:
                self.bestScore = self.score
                self.bestParam = self.param.copy()
                self.bestTrial = self.trial

            print("\tbestScore            =", self.bestScore, "; bestTrial =", self.bestTrial)
            print("\tbestParam            =", self.bestParam)

            # passage à l'évaluation suivante
            self.trial += 1

            # fin du budget => replay du meilleur
            if self.phase == 0 and self.trial >= self.evaluations:
                print("=== Budget exhausted. Replaying best strategy forever. ===")
                self.phase = 1
                self.param = self.bestParam.copy()
                self.demo_iter = 0
            else:
                # nouvelle stratégie aléatoire (phase recherche)
                if self.phase == 0:
                    self.param = [random.randint(-1, 1) for i in range(8)]
                    print("Trying strategy no.", self.trial)

            # reset score tracking pour la prochaine évaluation / prochain bloc demo
            self.score = 0.0
            self.prevT = 0.0
            self.prevR = 0.0

            self.iteration += 1
            return 0, 0, True  # ask for reset

        # --- phase demo: reset toutes les 1000 itérations, et boucle
        if self.phase == 1:
            self.demo_iter += 1
            if self.demo_iter % self.demo_len == 0:
                # reset demandé : le meilleur comportement est "montré" à l'infini
                return 0, 0, True

        # --- fonction de contrôle (perceptron)
        translation = math.tanh(
            self.param[0]
            + self.param[1] * sensors[sensor_front_left]
            + self.param[2] * sensors[sensor_front]
            + self.param[3] * sensors[sensor_front_right]
        )

        rotation = math.tanh(
            self.param[4]
            + self.param[5] * sensors[sensor_front_left]
            + self.param[6] * sensors[sensor_front]
            + self.param[7] * sensors[sensor_front_right]
        )

        if debug == True:
            if self.iteration % 100 == 0:
                print("Robot", self.robot_id, " (team " + str(self.team_name) + ")", "at step", self.iteration, ":")
                print("\tsensors (distance, max is 1.0)  =", sensors)
                print("\ttype (0:empty, 1:wall, 2:robot) =", sensor_view)
                print("\trobot's name (if relevant)      =", sensor_robot)
                print("\trobot's team (if relevant)      =", sensor_team)

        self.iteration += 1
        return translation, rotation, False
