import heapq
from collections import defaultdict

class Assignment():
  def __init__(self, instruction, assignments):
    self.assignments = assignments

start_state = {
  "rax": 1,
  "rcx": 2,
  "rdx": 3,
  "rbx": 4,
  "rsp": -1
}

end_state = {
  "rax": 4,
  "rcx": 3,
  "rdx": 2,
  "rbx": 1,
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


def find_neighbours(node):
  if node.neighbourscreated:
    return node.neighbours
    
  node.neighbourscreated = True
  candidates = []
  for key, value in node.state.items():
    if value == -1:
      for register in ["rax", "rcx", "rdx", "rbx", "rsp"]:
        movement = dict(node.state)
        movement[key] = movement[register]
        movement[register] = -1
        candidates.append(Node(movement, node.fScore, "mov %{}, %{}".format(register, key)))
  node.neighbours = candidates
  return candidates

start_node = Node(start_state, fScore, "start")
end_node = Node(end_state, fScore, "end")

def h(start, goal):
  total = 0
  for key, value in start.state.items():
    if value != goal.state[key]:
      total = total + 1
  return total


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
        if current.state == goal.state:
            return reconstruct_path(cameFrom, current)

        openSet.remove(current)
        for neighbour in find_neighbours(current):
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