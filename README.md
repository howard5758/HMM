# Author: Ping-Jung Liu
# Date: November 2nd 2017
# Acknowledgement: Professor Devin Balkom for providing structues and suggestions 

***README***

Files in this directory:

- HMM.pdf

- HMM.md

- HMM.py

- HMM_test.py

- Maze.py

- maze1.maz

- maze2.maz

- maze3.maz

################################################################################
To test the functionalities of filtering and forward-backward smoothing:
################################################################################
- open HMM.py and go to line 255, the main function

- open the maze you wish to test with:
  
  test_maze = Maze("maze2.maz")

- initialize HMM with:
  
  test = HMM(test_maze, list of sensor readings, list of actual locations)

  sample:
  
  test = HMM(test_maze, ["r", "y", "b", "y", "g"], [(1, 3), (0, 3), (0, 2), (1, 2), (0, 2), (0, 1)])
  (note there is one more actual location than color, because the former includes the start location)

- perform filtering and forward-backward smoothing:

  filter_result = test.filter()
  fb_result = test.fb_smoothing(filter_result)
  (note fb_smoothing needs the result of filter)

- print the results in a decent way with:

  test.print_results(filter_result)
    or	
  test.print_results(fb_result)

##################################################################################
To test a real robot moving and returning color observations
##################################################################################
- open HMM_test.py and go to line 75

- open the maze you wish to test:

  maze = Maze("maze2.maz")

- use the command:

  test_HMM(maze, start_locaction, number of steps)

You could test this with whatever number of steps you prefer. I did not include foward-backward in the result because its strengh shines in probability distribution of past states, but this test generates the latest probability distribution using existing sensor_readings each step, which neglect all the good calculations of forward-backward.
