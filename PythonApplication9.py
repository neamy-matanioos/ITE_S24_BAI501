import heapq

class Node:
    """Represents a node in the A* search space."""
    def __init__(self, path, cost, heuristic):
        self.path = path  # Current path taken
        self.cost = cost  # Cost of the path
        self.heuristic = heuristic  # Estimated cost to complete the tour

    def __lt__(self, other):
        """Comparison method for priority queue."""
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def a_star_tsp(graph, start):
    """Finds the shortest tour using the A* algorithm."""
    n = len(graph)
    visited = set()
    priority_queue = []
    
    # Initialize the starting node
    initial_node = Node(path=[start], cost=0, heuristic=0)
    heapq.heappush(priority_queue, initial_node)

    while priority_queue:
        current_node = heapq.heappop(priority_queue)
        current_path = current_node.path
        current_cost = current_node.cost

        # If all cities have been visited, return to the starting city
        if len(current_path) == n:
            total_cost = current_cost + graph[current_path[-1]][start]
            return current_path + [start], total_cost

        last_city = current_path[-1]
        for next_city in range(n):
            if next_city not in current_path:
                new_cost = current_cost + graph[last_city][next_city]
                heuristic = estimate_heuristic(graph, next_city, current_path)
                new_node = Node(path=current_path + [next_city], cost=new_cost, heuristic=heuristic)
                heapq.heappush(priority_queue, new_node)

    return None, float('inf')

def estimate_heuristic(graph, current_city, path):
    """Estimates the heuristic cost to complete the tour."""
    unvisited = set(range(len(graph))) - set(path)
    return sum(min(graph[current_city][j] for j in unvisited) for current_city in unvisited)

def get_distance_matrix(num_cities):
    """Prompts the user to input the distance matrix."""
    graph = []
    for i in range(num_cities):
        while True:
            try:
                distances = list(map(int, input(f"Enter distances from city {i} to other cities (space-separated): ").split()))
                if len(distances) != num_cities:
                    raise ValueError("The number of distances must match the number of cities.")
                graph.append(distances)
                break
            except ValueError as e:
                print(f"Invalid input: {e}. Please try again.")
    return graph

def main():
    """Main function to run the A* TSP algorithm."""
    while True:
        try:
            num_cities = int(input("Enter the number of cities: "))
            if num_cities <= 0:
                raise ValueError("Number of cities must be a positive integer.")
            break
        except ValueError as e:
            print(f"Invalid input: {e}. Please try again.")

    graph = get_distance_matrix(num_cities)

    while True:
        try:
            start_city = int(input(f"Enter the starting city (0 to {num_cities - 1}): "))
            if start_city < 0 or start_city >= num_cities:
                raise ValueError("Starting city must be within the valid range.")
            break
        except ValueError as e:
            print(f"Invalid input: {e}. Please try again.")

    # Run A* algorithm
    tour, total_cost = a_star_tsp(graph, start_city)
    
    if tour:
        print("Optimal tour:", " -> ".join(map(str, tour)))
        print("Total cost:", total_cost)
    else:
        print("No tour found.")

if __name__ == "__main__":
    main()