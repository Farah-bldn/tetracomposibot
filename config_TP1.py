# Configuration file.

import arenas

# general -- first three parameters can be overwritten with command-line arguments (cf. "python tetracomposibot.py --help")

display_mode = 0
arena = 1
position = False 
max_iterations = 501 #401*500

# affichage

display_welcome_message = False
verbose_minimal_progress = True # display iterations
display_robot_stats = False
display_team_stats = False
display_tournament_results = False
display_time_stats = True

# initialization : create and place robots at initial positions (returns a list containing the robots)

import robot_braitenberg_avoider as gg
import subsomption as gb
import subsomption as ga
import subsomption as gc

def initialize_robots(arena_size=-1, particle_box=-1): # particle_box: size of the robot enclosed in a square
    #x_center = arena_size // 2 - particle_box / 2
    y_center = arena_size // 2 - particle_box / 2
    robots = []
    robots.append(gc.Robot_player(4, y_center, 0, name="My Robot", team="A"))
    robots.append(ga.Robot_player(7, y_center, 0, name="My Robot", team="B"))
    robots.append(gb.Robot_player(10, y_center, 0, name="My Robot", team="C"))
    robots.append(gg.Robot_player(13, y_center, 0, name="My Robot", team="D"))
    return robots