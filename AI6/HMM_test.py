# Author: Ping-Jung Liu
# Date: November 2nd 2017
# COSC 76 Assignment 5: Hidden Markov Model
# Acknowledgement: Professor Devin Balkom for providing structues and suggestions 

from Maze import Maze
from HMM import HMM
import random
import math

def print_result(maze, result):

	prob_ind = 0
	for j in range(0, len(maze.map)):

		if not maze.map[j] == "#":
			print(result[len(result) - 1][prob_ind], end=" ")
			if (result[len(result) - 1][prob_ind] * 1000)%10 == 0:
				print(" ", end="")
			prob_ind = prob_ind + 1
		else:
			print("#####", end=" ")

		if (j + 1)%maze.width == 0:
			print("\n")

	print("-------------------------------------")

def test_HMM(maze, start_loc, step_num):

	step = 0
	sensor_reading = []
	location = [start_loc]

	print("step: " + str(step) + "\n")
	print("current location: " + str(start_loc) + "\n")
	print(maze)
	hmm = HMM(maze, sensor_reading, location)
	f = hmm.filter()
	print_result(maze, f)

	while step < step_num:

		step = step + 1
		move = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
		print("move: " + str(move) + "\n")
		new_loc = (location[step - 1][0] + move[0], location[step - 1][1] + move[1])

		if maze.is_floor(new_loc[0], new_loc[1]):
			location.append(new_loc)
		else:
			location.append(location[step - 1])

		all_color = ["r", "g", "y", "b"]
		color = maze.color_at(location[step][0], location[step][1])
		all_color.remove(color)
		if random.random() > 0.88:
			color = random.choice(all_color)
		sensor_reading.append(color)

		hmm = HMM(maze, sensor_reading, location)
		f = hmm.filter()

		print("step: " + str(step) + "\n")
		print("current location: " + str(location[step]) + "\n")
		print("sensor reading: " + str(color) + "\n")
		print(maze)
		print_result(maze, f)



#######################################################

maze = Maze("maze2.maz")

start_loc = (0, 0)

test_HMM(maze, start_loc, 10)



