# Import necessary libraries and modules
import tkinter.filedialog
import json
import pygame  
import random  
import time
import math
import sys  
import heapq  
from collections import deque  
from Constants import *
from queue import PriorityQueue
from collections import defaultdict

# Initialize pygame
pygame.init()

# Initialize the display with the specified width and height
screen = pygame.display.set_mode((GRID_WIDTH * CELL_SIZE, GRID_HEIGHT * CELL_SIZE))

pygame.display.set_caption("Multi-Goal Path-Finding in Underwater Environments")
icon = pygame.image.load('submarine.png')
pygame.display.set_icon(icon)

obstacles = [] # Obstacle positions list
goals_reached = [] # Goal list
selected_algorithms = [None, None] # Selected Algorithms index guide: (0: Path Finding, 1: Path Planning)

class GeneticAlgorithm:
    def __init__(self, goals, population_size, mutation_rate, num_generations):
        self.goals = goals  # List of goals that the AUV needs to reach
        self.population_size = population_size  # Number of individuals in each generation
        self.mutation_rate = mutation_rate  # Probability of mutation
        self.num_generations = num_generations  # Number of generations to run the algorithm

    def run(self):
        # Initialize the population with random individuals
        population = self._initialize_population()
        for i in range(self.num_generations):
            # Generate a new population
            population = self._generate_new_generation(population)
            # Find the best individual in the population (with minimum fitness)
            best_individual = min(population, key=self._fitness)
            print(f"Generation {i+1}, Best individual: {[goal.get_position() for goal in best_individual]}, Fitness: {self._fitness(best_individual)}")
        # Return the best individual from the final generation
        return min(population, key=self._fitness)

    def _initialize_population(self):
        # Create a population of individuals, each individual is a random permutation of the goals
        return [random.sample(self.goals, len(self.goals)) for _ in range(self.population_size)]

    def _generate_new_generation(self, population):
        next_generation = []
        for _ in range(self.population_size):
            # Select two parents from the population
            parent1, parent2 = self._select(population), self._select(population)
            # Create a child through crossover
            child = self._crossover(parent1, parent2)
            # With a certain probability, mutate the child
            if random.random() < self.mutation_rate:
                child = self._mutate(child)
            # Add the child to the next generation
            next_generation.append(child)
        return next_generation

    def _select(self, population):
        # Select 5 individuals from the population and return the one with minimum fitness
        return random.choice(
            sorted(random.sample(population, k=5), key=self._fitness)
        )

    def _crossover(self, parent1, parent2):
        # Perform one-point crossover: Take the first part from parent1 and the remaining part from parent2
        crossover_point = random.randint(1, len(self.goals) - 1)
        child = parent1[:crossover_point]
        child += [goal for goal in parent2 if goal not in child]
        return child

    def _mutate(self, individual):
        # Swap two random goals in the individual
        index1, index2 = random.sample(range(len(self.goals)), 2)
        individual[index1], individual[index2] = individual[index2], individual[index1]
        return individual

    def _fitness(self, individual):
        # The fitness of an individual is the sum of distances between consecutive goals in the individual
        return sum(heuristic(goal1.get_position(), goal2.get_position()) for goal1, goal2 in zip(individual, individual[1:]))
          
class Grid:
    def __init__(self, width, height):
        self.width = width  #setting the width of the grid
        self.height = height  #setting the height of the grid

    def in_bounds(self, id):
        (x, y) = id  #Get x and y coordinates
        #Returns True if coordinate inside the grid and False otherwise
        return 0 <= x < self.width and 0 <= y < self.height 

    def passable(self, id):
        # checks if a given point doesn't contain an obstacle
        # returns True if there's no obstacle at the point and False otherwise
        return id not in auv.detected_obstacles 

    def neighbors(self, id):
        (x, y) = id  # unpack the tuple to get x and y coordinates
        # defines potential neighboring points (up, down, left, right, and diagonals)
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1),  #Up, down, left, right
                     (x-1, y-1), (x+1, y+1), (x-1, y+1), (x+1, y-1)]  #Diagonals
        # filter out neighbors that are out of bounds
        neighbors = filter(self.in_bounds, neighbors)
        # filter out neighbors that are not passable (contain an obstacle)
        neighbors = filter(self.passable, neighbors)
        # returns valid neighbors
        return neighbors
        
class AUV:
    def __init__(self, x, y):
        self.x = x  # setting the initial x-coordinate of the AUV
        self.y = y  # setting the initial y-coordinate of the AUV
        self.detected_obstacles = set()  # initializing an empty set to store detected obstacles      

    def move(self, dx, dy):
        self.x += dx  # update the x-coordinate by dx
        self.y += dy  # update the y-coordinate by dy
        self.detect_obstacles()  # call the detect_obstacles function after moving

    def detect_obstacles(self):
        for dx in range(-1, DETECTION_RADIUS):  # iterating over a range of relative x-coordinates
            for dy in range(-1, DETECTION_RADIUS):  # iterating over a range of relative y-coordinates
                new_x = self.x + dx  # compute the x-coordinate
                new_y = self.y + dy  # compute the y-coordinate
                if (new_x, new_y) in obstacles:  # if the coordinate is in the obstacles list
                    self.detected_obstacles.add((new_x, new_y))  # add it to the detected obstacles set
    
    def reset_detected_obstacles(self):
        # clears the detected obstacles set
        self.detected_obstacles.clear()
  
    def get_position(self):
        # returns the current position of the AUV as a tuple
        return self.x, self.y
    
    def set_position(self, x, y):
        # sets the position of the AUV to the input x and y coordinates
        self.x = x
        self.y = y

    def find_closest_goal(self, goals):
        closest_goal = None
        shortest_path_length = float('inf')  # Initialize shortest path length as infinity
        for goal in goals:
            path, _ = a_star_search(self.get_position(), goal.get_position())  # Use the A* algorithm
            if path:
                path_length = len(path)
                if path_length < shortest_path_length:
                    shortest_path_length = path_length
                    closest_goal = goal

        return closest_goal

class Goal:
    def __init__(self, x, y):
        self.x = x #setting the initial x-coordinate of goal
        self.y = y #setting the initial y-coordinate of goal

    def get_position(self):
        #returns the current position of the goal as a tuple
        return self.x, self.y

def grid_search(goals, time_limit):
    
    grid = Grid(GRID_WIDTH, GRID_HEIGHT)

    start_time = time.time()

    # Define the parameter space
    population_sizes = [50, 100, 200]
    mutation_rates = [0.01, 0.05, 0.1]
    num_generations = [500, 1000, 2000]

    best_params = None
    best_fitness = float('inf')
    num_goals = len(goals)
    fitnesses = []

    # Iterate over all combinations of parameters
    for pop_size in population_sizes:
        for mut_rate in mutation_rates:
            for num_gen in num_generations:
                # Check if the time limit has been reached
                if time.time() - start_time > time_limit:
                    print("Time limit reached, stopping grid search.")
                    break
                print(f"Running GA with population size {pop_size}, mutation rate {mut_rate}, and {num_gen} generations")
                ga = GeneticAlgorithm(goals, pop_size, mut_rate, num_gen)
                best_individual = ga.run()
                fitness = ga._fitness(best_individual)
                fitnesses.append(fitness)
                # If this is the best fitness we've seen so far, update the best parameters
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_params = (pop_size, mut_rate, num_gen)
            else:
                continue  # only executed if the inner loop did NOT break
            break  # only executed if the inner loop DID break
        else:
            continue  # only executed if the inner loop did NOT break
        break  # only executed if the inner loop DID break

    # Calculate average fitness
    avg_fitness = sum(fitnesses) / len(fitnesses) if fitnesses else float('inf')

    # Stop the timer
    end_time = time.time()

    # Calculate time taken
    time_taken = end_time - start_time

    print(f"Best parameters are population size {best_params[0]}, mutation rate {best_params[1]}, and {best_params[2]} generations")

    # Open a text file in write mode and store the best parameters
    with open('best_parameters.txt', 'w') as file:
        file.write(f"Number of goals: {num_goals}\n")
        file.write(f"Map size: {grid.width} x {grid.height}\n")
        file.write(f"Time taken for grid search: {time_taken} seconds\n")
        file.write(f"Average fitness: {avg_fitness}\n")
        file.write(f"Best parameters are: population size {best_params[0]}, mutation rate {best_params[1]}, and {best_params[2]} generations\n")

    return best_params

def heuristic(current, goal):
    dx = abs(current[0] - goal[0])
    dy = abs(current[1] - goal[1])
    return math.sqrt(2) * min(dx, dy) + abs(dx - dy)

def a_star_search(start, goal):
    # Create the grid
    grid = Grid(GRID_WIDTH, GRID_HEIGHT)
    # Initialize the frontier with the start position
    frontier = []
    heapq.heappush(frontier, (0, start))
    # Initialize the start position
    came_from = {start: None}
    # Initialize the start position to have a cost of zero
    cost_so_far = {start: 0}
    # Initialize the list to store explored nodes
    explored = []

    # Continue until there are no more nodes to explore
    while frontier:
        # Choose the node with the lowest total cost
        _, current = heapq.heappop(frontier)
        explored.append(current)
        

        # If this node is the goal, we're done
        if current == goal:
            break

        # Visit all neighbors of the current node
        for neighbor in grid.neighbors(current):
            
            # Calculate the new cost to reach the neighbor
            new_cost = cost_so_far[current] + 1
            # If we've found a shorter path to this neighbor, update our path and costs
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    # Construct the path
    current = goal
    path = []
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()  

    return path, explored

def dijkstra_search(start, goal):
    # Create a grid object 
    grid = Grid(GRID_WIDTH, GRID_HEIGHT)

    
    frontier = deque()
    frontier.append(start)

    # The dictionary of where we came from for each location
    came_from = {}
    came_from[start] = None

    # The cost to reach each location from the start location
    cost_so_far = {}
    cost_so_far[start] = 0  # The cost to reach the start is 0

    # List of explored nodes
    explored = []


    # Continue while there's more to explore
    while frontier:
        # Get the next location to explore and remove it from the frontier
        current = frontier.popleft()
        explored.append(current)  # Add current node to explored nodes

        # If we're at the goal, we're done
        if current == goal:
            break

        # Check all the neighboring tiles
        for next in grid.neighbors(current):

            # It costs 1 to move to a neighboring tile
            new_cost = cost_so_far[current] + 1

            # If we've never been to this location before, or we found a shorter
            # way to get here, then we update our cost map and add it to the frontier
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.append(next)
                came_from[next] = current

    # Reconstruct the path from the goal back to the start
    path = []
    current = goal
    while current != start:  # From the goal, go backwards to the start
        path.append(current)  # Add the current location to the path
        current = came_from[current]  # Go to where we came from at this location
    path.append(start)  # Finally, add the start location
    path.reverse()  # Reverse the path, so it goes from start to goal

    return path, explored

def create_random_goals(num_goals, grid_width, grid_height):
    # Initialize empty list of goals
    goals = []
    # Loop for the number of goals desired
    for _ in range(num_goals):
        # Generate random x and y coordinates within the grid
        x = random.randint(0, grid_width - 1)
        y = random.randint(0, grid_height - 1)
        # Create a new goal at the random coordinates and add to the list
        goals.append(Goal(x, y))
    # Return the list of goals
    return goals

def create_obstacles(density):
    global obstacles

    # Calculate the number of obstacles based on the grid size and desired density
    num_obstacles = int(GRID_WIDTH * GRID_HEIGHT * density)
    obstacle_positions = set()  # Set of positions where obstacles are located

    def add_blob(x, y, size):
        # Recursive function to add a "blob" of obstacles
        if size == 0:
            return
        if (x, y) not in obstacle_positions and 0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT:
            obstacle_positions.add((x, y))  # Add current position to the obstacles
            # Add blobs in the four directions
            add_blob(x + 1, y, size - 1)
            add_blob(x - 1, y, size - 1)
            add_blob(x, y + 1, size - 1)
            add_blob(x, y - 1, size - 1)

    # Keep adding blobs until we have the desired number of obstacles
    while len(obstacle_positions) < num_obstacles:
        x = random.randint(0, GRID_WIDTH - 1)
        y = random.randint(0, GRID_HEIGHT - 1)
        blob_size = random.randint(3, 4)  # Random size for the blob
        add_blob(x, y, blob_size)  # Add a blob at the random position

    # Update the global obstacles list
    obstacles = list(obstacle_positions)

def initialize_auv_and_goals(num_goals):
    global auv, goals, obstacles

    # Create a list of available positions that don't contain obstacles
    available_positions = [(x, y) for x in range(GRID_WIDTH) for y in range(GRID_HEIGHT) if (x, y) not in obstacles]

    # Create the AUV agent with a random starting position from the available positions
    start_x, start_y = random.choice(available_positions)
    auv = AUV(start_x, start_y)

    # Remove the AUV's starting position from the available positions
    available_positions.remove((start_x, start_y))

    # Create random goals and sort them based on a custom sorting criterion
    goal_positions = random.sample(available_positions, num_goals)
    goals = [Goal(x, y) for x, y in goal_positions]

def setup_environment(grid_size):
    global GRID_WIDTH, GRID_HEIGHT, screen
    GRID_WIDTH, GRID_HEIGHT = grid_size
    screen = pygame.display.set_mode((GRID_WIDTH * CELL_SIZE, GRID_HEIGHT * CELL_SIZE))

def draw_search(nodes):
    for x, y in nodes:
        rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(screen, (0, 255, 0, 100), rect)  # RGB for light green

def draw_grid():
    for x in range(0, GRID_WIDTH * CELL_SIZE, CELL_SIZE):
        for y in range(0, GRID_HEIGHT * CELL_SIZE, CELL_SIZE):
            rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, BG_COLOR, rect)
            # Draw cell borders
            pygame.draw.lines(screen, BORDER_COLOR, False, [(x, y), (x + CELL_SIZE, y)], 1)
            pygame.draw.lines(screen, BORDER_COLOR, False, [(x, y), (x, y + CELL_SIZE)], 1)
            if y == GRID_HEIGHT * CELL_SIZE - CELL_SIZE:
                pygame.draw.lines(screen, BORDER_COLOR, False, [(x, y + CELL_SIZE), (x + CELL_SIZE, y + CELL_SIZE)], 1)
            if x == GRID_WIDTH * CELL_SIZE - CELL_SIZE:
                pygame.draw.lines(screen, BORDER_COLOR, False, [(x + CELL_SIZE, y), (x + CELL_SIZE, y + CELL_SIZE)], 1)

# Function to draw the AUV on the screen
def draw_auv():
    x, y = auv.get_position()
    rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
    pygame.draw.rect(screen, AUV_COLOR, rect)

# Function to visualize the AUV's obstacle detection range
def draw_auv_detection_range():
    
    s = pygame.Surface((DETECTION_RADIUS*2*CELL_SIZE, DETECTION_RADIUS*2*CELL_SIZE), pygame.SRCALPHA)

    pygame.draw.circle(s, (255,255,255,128), (DETECTION_RADIUS*CELL_SIZE, DETECTION_RADIUS*CELL_SIZE), DETECTION_RADIUS*CELL_SIZE)

    auv_position = auv.get_position()
    screen.blit(s, ((auv_position[0]*CELL_SIZE)-(DETECTION_RADIUS*CELL_SIZE), (auv_position[1]*CELL_SIZE)-(DETECTION_RADIUS*CELL_SIZE)))

# Function to draw the goals on the screen
def draw_goals():
    for i, goal in enumerate(goals):  # Iterate over each goal with its index
        x, y = goal.get_position()  # Get the position of the goal
        rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)  # Create a rectangle for the goal
        pygame.draw.rect(screen, GOAL_COLOR, rect)  # Draw the goal rectangle on the screen

def draw_obstacles():
    
    for x, y in obstacles:
        rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(screen, OBSTACLE_COLOR, rect)

# Function to draw the path on the screen
def draw_path(path):
    for x, y in path:
        rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(screen, (0, 100, 0), rect)

def draw_button(text, x, y, w, h, color, hover_color):
    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()

    if x + w > mouse[0] > x and y + h > mouse[1] > y:
        pygame.draw.rect(screen, hover_color, (x, y, w, h))
        if click[0] == 1:
            return True
    else:
        pygame.draw.rect(screen, color, (x, y, w, h))

    font = pygame.font.Font(None, FONT_SIZE)
    text_surface = font.render(text, True, TEXT_COLOR)
    text_rect = text_surface.get_rect()
    text_rect.center = (x + w // 2, y + h // 2)
    screen.blit(text_surface, text_rect)

    return False

def save_environment(filename):
    # Open a file with the given filename in write mode
    with open(filename, "w") as file:
        # Dump a dictionary containing the environment details into the file as a JSON
        json.dump({
            "grid_width": GRID_WIDTH,  # Store the grid width
            "grid_height": GRID_HEIGHT,  # Store the grid height
            "auv_pos": auv.get_position(),  # Store the AUV's position
            "goal_positions": [goal.get_position() for goal in goals],  # Store the positions of all goals
            "obstacle_positions": obstacles  # Store the positions of all obstacles
        }, file)

def load_environment(filename):
    global GRID_WIDTH, GRID_HEIGHT, auv, goals, obstacles, env_name
    env_name = filename
    # Open a file with the given filename in read mode
    with open(filename, "r") as file:
        # Load the JSON data from the file into a dictionary
        data = json.load(file)
        # Retrieve the grid dimensions from the dictionary
        GRID_WIDTH, GRID_HEIGHT = data["grid_width"], data["grid_height"]
        # Set up the environment with the retrieved grid dimensions
        setup_environment((GRID_WIDTH, GRID_HEIGHT))
        # Create the AUV at the position retrieved from the dictionary
        auv = AUV(*data["auv_pos"])
        # Create the goals at the positions retrieved from the dictionary
        goals = [Goal(x, y) for x, y in data["goal_positions"]]
        # Set the obstacles at the positions retrieved from the dictionary
        obstacles = [(x, y) for x, y in data["obstacle_positions"]] 

def reset_environment(auv_original_x, auv_original_y):
    # Reset the AUV's position to its original position
    auv.set_position(auv_original_x, auv_original_y)
    
def main():
    
    total_moves = []  # Store total moves/path length of the AUV for each run
    total_times = []  # Store total computation time of the search algorithms for each run
    ga_times = [] # Store Genetic Algorithm run times
    num_runs = 20

    auv_original_x, auv_original_y = auv.get_position()
    

    for run in range(num_runs):

        if selected_algorithms[1] == 1:

            # Run the grid search to find the best parameters #float('inf') to run for unlimited time
            best_params = [100, 0.1 , 1000]
            #best_params = grid_search(goals, time_limit=10)

            # Now use the best parameters to run the Genetic Algorithm
            ga = GeneticAlgorithm(goals, best_params[0], best_params[1], best_params[2])

            ga_start_time = time.time()
            goal_order = ga.run()
            ga_end_time = time.time()

            ga_times.append(ga_end_time - ga_start_time)

            print("Goal order: ", [goal.get_position() for goal in goal_order])

        moves = 0  # Count moves for this run
        total_time = 0  # Count total time for this run

        # Reset environment
        reset_environment(auv_original_x, auv_original_y)

        # Reset AUV Obstacle Memory
        auv.reset_detected_obstacles()

        # Clear the screen
        screen.fill(BG_COLOR)

        # Draw the grid, AUV, and goals
        draw_grid()
        draw_auv()
        draw_goals()
        draw_obstacles()
        draw_auv_detection_range()
        
        if selected_algorithms[1] == 1:
            goals_reached = goal_order[:]
        else:
            goals_reached = goals[:]       
        path = None

        pygame.font.init()
        font = pygame.font.Font(None, 36)

        # Game loop
        running = True
        while running:
            # Event handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    pygame.quit()
                    sys.exit()


            # Calculate path using a pathfinding algorithm
            if goals_reached:
                
                if selected_algorithms[1] == 1:
                    closest_goal = goals_reached[0]
                else:
                    closest_goal = auv.find_closest_goal(goals_reached)

                # If there's no path or a position in the path is blocked, recalculate the path
                if not path or any(position in auv.detected_obstacles for position in path):

                    if selected_algorithms[0] == 1:
                        start_time = time.time()
                        path, explored = a_star_search(auv.get_position(), closest_goal.get_position())
                        end_time = time.time()
                    elif selected_algorithms[0] == 2:
                        start_time = time.time()
                        path, explored = dijkstra_search(auv.get_position(), closest_goal.get_position())
                        end_time = time.time()

                    total_time += end_time - start_time
                    

                # Clear the screen
                screen.fill(BG_COLOR)
                draw_grid()
                draw_search(explored)
                draw_path(path)
                draw_goals()
                draw_obstacles()
                draw_auv()
                draw_auv_detection_range()

                # Display the number of goals reached
                goals_reached_count = len(goals) - len(goals_reached)
                total_goals = len(goals)
                text_surface = font.render(f"Goals reached: {goals_reached_count}/{total_goals}", True, (255, 255, 255))
                screen.blit(text_surface, (screen.get_width() / 2 - text_surface.get_width() / 2, 0))

                # Move the AUV along the path
                if path:
                    next_position = path.pop(1)
                    x, y = auv.get_position()
                    x_move = next_position[0] - x
                    y_move = next_position[1] - y
                    auv.move(x_move, y_move)
                    moves += 1 
                    # If reached a goal, remove it from the list
                    if auv.get_position() == closest_goal.get_position():
                        if selected_algorithms[1] == 1:
                            goals_reached.pop(0)
                        else:
                            goals_reached.remove(closest_goal)
                        path = None  # Reset path when a goal is reached

            # Update the display
            pygame.display.flip()

            # Cap the frame rate
            pygame.time.Clock().tick(100000)

            # If all goals are reached, break the game loop
            if not goals_reached:
                running = False

        # Record path length and computation time
        total_moves.append(moves)  # Store total moves for this run(total path length)
        total_times.append(total_time)  # Store total time for this run

        

    #Write the data to a txt file
    with open('experiment_data.txt', 'a') as f:

        # Write the enviroment used at the top of each experiment
        f.write(f'Enviroment: {env_name}\n')

        # Write the name of the algorithms used
        if selected_algorithms[1] == 1:
            f.write("Path-Planning: Genetic Algorithm\n")
        else:
            f.write("Path-Planning: Greedy Search\n")

        if selected_algorithms[0] == 1:
            f.write("Path-Finding: A* Search\n")
        else:
            f.write("Path-Finding: Dijkstra\n")

        for i in range(num_runs):
            f.write(f'Run {i+1}:\n')
            f.write(f'Total moves/path length: {total_moves[i]}\n')
            f.write(f'Total computation time: {total_times[i]}\n')
            # Write GA run time if GA was used
            if selected_algorithms[1] == 1:
                f.write(f'Genetic Algorithm computation time: {ga_times[i]}\n')

            f.write('\n')

        # Compute averages
        average_moves = sum(total_moves) / num_runs
        average_time = sum(total_times) / num_runs

        # Compute GA average time if GA was used
        if selected_algorithms[1] == 1:
            average_ga_time = sum(ga_times) / num_runs
            f.write(f'Average Genetic Algorithm computation time: {average_ga_time}\n')

        # Write averages to file
        f.write(f'Average moves: {average_moves}\n')
        f.write(f'Average computation time: {average_time}\n')

        # Write a dashed line to indicate the end of this experiment's results
        f.write('----------------------------------------\n')
        

    #Quit the game
    pygame.quit()
    sys.exit()

def menu():
    menu_running = True

    selected_size = None
    selected_goals = None
    selected_density = None
    save_requested = False
    path_finding_clicked = False
    path_planning_clicked = False

    while menu_running:
        screen.fill(BG_COLOR)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                menu_running = False
                pygame.quit()
                sys.exit()

        
        ga_button_clicked = draw_button("GA", 400, 50, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        greedy_button_clicked = draw_button("Greedy", 200, 50, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        a_star_button_clicked = draw_button("A*", 410, 125, 75, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        dijkstra_button_clicked = draw_button("Dijkstra", 210, 125, 75, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        low_density_button_clicked = draw_button("Low Density (0.1)", 80, 400, 150, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        medium_density_button_clicked = draw_button("Medium Density (0.25)", 280, 400, 150, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        high_density_button_clicked = draw_button("High Density (0.4)", 480, 400, 150, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        small_button_clicked = draw_button("Small(50 x 50)", 150, 200, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        medium_button_clicked = draw_button("Medium(75 x 75)", 300, 200, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        large_button_clicked = draw_button("Large(150 x 150)", 450, 200, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        goals_10_button_clicked = draw_button("10 Goals", 150, 300, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        goals_20_button_clicked = draw_button("20 Goals", 300, 300, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)
        goals_30_button_clicked = draw_button("30 Goals", 450, 300, 100, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        
        load_button_clicked = draw_button("Load Environment", 150, 570, 400, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        save_button_clicked = draw_button("Save Environment", 150, 650, 400, 50, BUTTON_COLOR, BUTTON_HOVER_COLOR)

        if save_button_clicked:
            save_requested = True

        if goals_10_button_clicked:
            selected_goals = 10
            initialize_auv_and_goals(selected_goals)
        elif goals_20_button_clicked:
            selected_goals = 20
            initialize_auv_and_goals(selected_goals)
        elif goals_30_button_clicked:
            selected_goals = 30
            initialize_auv_and_goals(selected_goals)

        if small_button_clicked:
            selected_size = SMALL
        elif medium_button_clicked:
            selected_size = MEDIUM
        elif large_button_clicked:
            selected_size = LARGE

        if low_density_button_clicked:
            selected_density = OBSTACLE_DENSITY_LOW
        elif medium_density_button_clicked:
            selected_density = OBSTACLE_DENSITY_MEDIUM
        elif high_density_button_clicked:
            selected_density = OBSTACLE_DENSITY_HIGH

        if  a_star_button_clicked:
            selected_algorithms[0] = 1
            path_finding_clicked = True
        elif dijkstra_button_clicked:
            selected_algorithms[0] = 2
            path_finding_clicked = True

        if ga_button_clicked:
            selected_algorithms[1] = 1
            path_planning_clicked = True
        elif greedy_button_clicked:
            selected_algorithms[1] = 2
            path_planning_clicked = True 


        if selected_size and selected_goals and selected_density is not None and path_finding_clicked and path_planning_clicked:
            setup_environment(selected_size)
            create_obstacles(selected_density)
            initialize_auv_and_goals(selected_goals)
            if save_requested:
                filename = tkinter.filedialog.asksaveasfilename(defaultextension=".json")
                if filename:
                    save_environment(filename)
                    save_requested = False
            main()

        if load_button_clicked and path_finding_clicked and path_planning_clicked:
            filename = tkinter.filedialog.askopenfilename()
            if filename:
                load_environment(filename)
                main()
                
        pygame.display.flip()
        pygame.time.Clock().tick(1000)

if __name__ == "__main__":
    menu()


    