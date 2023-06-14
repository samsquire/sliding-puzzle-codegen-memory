import heapq
from collections import defaultdict
import copy


class Assignment():
  def __init__(self, instruction, assignments):
    self.assignments = assignments

start_state = { 
  "memory": [
    0, 0, 0, 0
  ],
  "rax": 0,
  "rcx": 1,
  "rdx": 2,
  "rbx": 3,
  "rsp": -1
}

end_state = {
  "memory": [
    3, 1, 2, 0
  ],
  "rax": 3,
  "rcx": 2,
  "rdx": 1,
  "rbx": 0,
  "rsp": -1
}


#step1 = find_neighbours(start_state)
# print(step1)

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path

class Node():
  def __init__(self, state, fScore, instruction):
    self.state = state
    self.fScore = fScore
    self.neighbourscreated = False
    self.neighbours = []
    self.instruction = instruction

  def __lt__(self, other):
    return fScore[self] < fScore[other]
  def __repr__(self):
    return self.instruction

fScore = defaultdict(lambda: float('inf'))


def find_neighbours(node, goal):
  if node.neighbourscreated:
    return node.neighbours
    
  node.neighbourscreated = True
  candidates = []

 
    # candidates.append(Node(movement, node.fScore, "mov ${}, %{}".format(item, "rax")))

  if node.state["memory"] != goal.state["memory"]:
    for memory_location, value_in_memory in enumerate(goal.state["memory"]):
      if node.state["memory"][memory_location] != goal.state["memory"][memory_location]:
          for memory_location_key, memory_location_register in node.state.items():
            
            # We found a memory location in a register
            if memory_location_key != "memory":
   
              if memory_location_register == memory_location:
                for current_key, current_value in node.state.items():
                  if current_key != "memory":
                    # We found a register that matches the desired memory value in memory
                    if current_value == value_in_memory:
                        # print("found wanted value {}".format(value_in_memory))
                        movement = copy.deepcopy(node.state)
                        
                        movement["memory"][memory_location] = value_in_memory
                        # movement["rax"] = -1
                        candidates.append(Node(movement, node.fScore, "mov %{}, (%{})".format(memory_location_key, current_key)))
                        found = True
                        break
                if found:
                  break
          if found:
            break
      
    
  for key, value in node.state.items():
    
    if key == "memory":
      continue
      
    if value == -1:
      for register in ["rax", "rcx", "rdx", "rbx", "rsp"]:
        movement = copy.deepcopy(node.state)
        movement[key] = movement[register]
        movement[register] = -1
        candidates.append(Node(movement, node.fScore, "mov %{}, %{}".format(register, key)))
        
  node.neighbours = candidates
  # print(candidates)
  return candidates

start_node = Node(start_state, fScore, "start")
end_node = Node(end_state, fScore, "end")

def h(start, goal):
  total = 0
  for key, value in start.state.items():
    if key == "memory":
      for index, item in enumerate(start.state[key]):
        if goal.state[key][index] != goal.state[key][index]:
          total = total + (goal.state[key][index] - goal.state[key][index])
    else:
      
        if value != goal.state[key]:
          total = total + 1
 
  return total

def same(current, goal):
  if current.state == goal.state:
    return True
  return False

# A* finds a path from start to goal.
# h is the heuristic function. h(n) estimates the cost to reach goal from node n.
def A_Star(start, goal, h, fScore):
    # The set of discovered nodes that may need to be (re-)expanded.
    # Initially, only the start node is known.
    # This is usually implemented as a min-heap or priority queue rather than a hash-set.
    openSet = [start]

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from the start
    # to n currently known.
    cameFrom = {}

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = defaultdict(lambda: float('inf'))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    # how cheap a path could be from start to finish if it goes through n.
    fScore[start] = h(start, goal)
    

    while len(openSet) > 0:
        # This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current = heapq.heappop(openSet) # the node in openSet having the lowest fScore[] value
        heapq.heappush(openSet, current)
        if same(current, goal):
            return reconstruct_path(cameFrom, current)

        openSet.remove(current)
        for neighbour in find_neighbours(current, goal):
            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + 1# d(current, neighbor)
            if tentative_gScore < gScore[neighbour]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbour] = current
                gScore[neighbour] = tentative_gScore
                fScore[neighbour] = tentative_gScore + h(neighbour, goal)
                if neighbour not in openSet:
                    heapq.heappush(openSet, neighbour)

    # Open set is empty but goal was never reached
    return "failure"

search = A_Star(start_node, end_node, h, fScore)
print(search)
print(search[-1].state)