# Author: Ping-Jung Liu
# Date: November 2nd 2017
# COSC 76 Assignment 5: Hidden Markov Model
# Acknowledgement: Professor Devin Balkom for providing structues and suggestions 

from Maze import Maze
import math

class HMM:

	def __init__(self, maze, sensor_reading, actual_pos):

		self.maze = maze
		self.sensor_reading = ["X"] + sensor_reading
		self.actual_pos = actual_pos
		self.transition = self.build_transition()
		self.sensor = self.build_sensor()
		self.result = self.initialize_result()

	# transition model
	def build_transition(self):

		# initialize an all 0s matrix of size num_state * num_state
		matrix = self.build_matrix(self.maze.num_state(), self.maze.num_state())

		# loop through all locations
		for x in range(0, self.maze.width):
			for y in range(0, self.maze.height):
				state_num = self.maze.state_num(x, y)
				# ignore walls
				if state_num == -1:
					continue

				# find the neighbors
				neighbors = self.maze.find_neighbors(x, y)
				wall_num = 5 - len(neighbors)
				# P of staying at current location
				matrix[state_num][state_num] = wall_num/4

				# P of moving to neighbor locations
				for neighbor in neighbors:
					if not neighbor == (x, y):
						matrix[state_num][self.maze.state_num(neighbor[0], neighbor[1])] = 0.25

		return matrix

	# sensor model, note sensor models are all diagonal matrices
	def build_sensor(self):

		maze = self.maze
		# build a num_state*num_state matrix for all four colors
		red = self.build_matrix(self.maze.num_state(), self.maze.num_state())
		green = self.build_matrix(self.maze.num_state(), self.maze.num_state())
		yellow = self.build_matrix(self.maze.num_state(), self.maze.num_state())
		blue = self.build_matrix(self.maze.num_state(), self.maze.num_state())

		for x in range(0, self.maze.width):
			for y in range(0, self.maze.height):
				state_num = self.maze.state_num(x, y)

				if state_num == -1:
					continue

				# initialize to 0.04, the possibility of wrong color
				red[state_num][state_num] = 0.04
				green[state_num][state_num] = 0.04
				yellow[state_num][state_num] = 0.04
				blue[state_num][state_num] = 0.04

				# if a state is a color, change the probability to the right color prob: 0.88
				if maze.color_at(x, y) == "r":
					red[state_num][state_num] = 0.88
				elif maze.color_at(x, y) == "g":
					green[state_num][state_num] = 0.88
				elif maze.color_at(x, y) == "y":
					yellow[state_num][state_num] = 0.88
				elif maze.color_at(x, y) == "b":
					blue[state_num][state_num] = 0.88

		return (red, green, yellow, blue)

	# norm(sensor_model * transition' * result)
	def filter(self):

		trans_transition = self.transpose(self.transition)

		for i in range(1, len(self.sensor_reading)):
			self.result[i] = self.m_times_vec(trans_transition, self.result[i - 1])

			if self.sensor_reading[i] == "r":
				self.result[i] = self.m_times_vec(self.sensor[0], self.result[i])
			elif self.sensor_reading[i] == "g":
				self.result[i] = self.m_times_vec(self.sensor[1], self.result[i])
			elif self.sensor_reading[i] == "y":
				self.result[i] = self.m_times_vec(self.sensor[2], self.result[i])
			elif self.sensor_reading[i] == "b":
				self.result[i] = self.m_times_vec(self.sensor[3], self.result[i])

		for v in self.result:
			v = self.normalize(v) 

		# round to 1/1000
		self.clean(self.result)
		return self.result

	# forward-backward smoothing 
	def fb_smoothing(self, filter_result):

		# initialize back and result lists
		back = []
		for i in range(0, self.maze.num_state()):
			back.append(1)
		result = []
		for i in range(0, len(filter_result)):
			result.append(0)

		# follow the formula, starts from the end
		t = len(self.sensor_reading) - 1

		while t >= 1:
			# v_times_v multiply the elements of corresponding index
			result[t] = self.v_times_v(filter_result[t], back)
			result[t] = self.normalize(result[t])
			back = self.b(back, t - 1)
			t = t - 1

		result = result[1:len(result)]
		self.clean(result)
		return result

	# transition * sensor_model * result
	def b(self, back, t):

		temp = []

		if self.sensor_reading[t] == "r":
			temp = self.m_times_vec(self.sensor[0], back)
		elif self.sensor_reading[t] == "g":
			temp = self.m_times_vec(self.sensor[1], back)
		elif self.sensor_reading[t] == "y":
			temp = self.m_times_vec(self.sensor[2], back)
		elif self.sensor_reading[t] == "b":
			temp = self.m_times_vec(self.sensor[3], back)

		result = self.m_times_vec(self.transition, temp)
		return result 

	# initialize the search result
	def initialize_result(self):

		result = []
		for i in range(0, len(self.sensor_reading)):
			result.append([])
		for i in range(0, self.maze.num_state()):
			result[0].append(1/self.maze.num_state())
		return result

	# build a x*y matrix
	def build_matrix(self, x, y):

		temp = []
		for i in range(0, y):
			tempp = []
			for j in range(0, x):
				tempp.append(0)
			temp.append(tempp)
		return temp

	# round all the numbers in a matrix to 1/1000
	def clean(self, m):
		
		for i in range(0, len(m[0])):
			for j in range(0, len(m)):
				m[j][i] = (math.floor(1000 * m[j][i]) + 1)/1000


	# make sure a list of probabilities add up to 1
	def normalize(self, v):

		summ = 0
		for i in range(0, len(v)):
			summ = summ + v[i]
		for i in range(0, len(v)):
			v[i] = v[i]/summ
		return v

	# v_times_v multiply the elements of corresponding index
	def v_times_v(self, v1, v2):

		result = []
		for i in range(0, len(v1)):
			result.append(v1[i] * v2[i])
		return result

	# matrix times vector
	def m_times_vec(self, m, v):

		result = []
		for i in range(0, len(m)):
			summ = 0
			for j in range(0, len(v)):
				summ = summ + m[i][j] * v[j]
			result.append(summ)
		return result

	# transpose a matrix
	def transpose(self, matrix):

		width = len(matrix[0])
		height = len(matrix)

		trans = self.build_matrix(height, width)
		for i in range(0, height):
			for j in range(0, width):
				trans[j][i] = matrix[i][j]
		return trans

	# print the resulting probability distribution in nicer ways
	def print_results(self, filter_result):

		flag = False
		if len(filter_result) < len(self.sensor_reading):
			filter_result = [0] + filter_result
		for i in range(0, len(filter_result)):

			print("step: " + str(i))
			print("observed color: " + self.sensor_reading[i])
			print("actual position: " + str(self.actual_pos[i]) + "\n")
			print(self.maze)

			if filter_result[0] == 0 and not flag:
				print("NNNAAA")
				print("-------------------------------------")
				flag = True
				continue

			prob_ind = 0
			for j in range(0, len(self.maze.map)):

				if not self.maze.map[j] == "#":
					print(filter_result[i][prob_ind], end=" ")
					if (filter_result[i][prob_ind] * 1000)%10 == 0:
						print(" ", end="")
					prob_ind = prob_ind + 1
				else:
					print("#####", end=" ")

				if (j + 1)%self.maze.width == 0:
					print("\n")

			print("-------------------------------------")



if __name__ == "__main__":

	test_maze = Maze("maze2.maz")
	test = HMM(test_maze, ["r", "y", "b", "y", "g"], [(1, 3), (0, 3), (0, 2), (1, 2), (0, 2), (0, 1)])

	filter_result = test.filter()
	fb_result = test.fb_smoothing(filter_result)

	 









