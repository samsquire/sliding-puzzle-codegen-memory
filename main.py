import heapq
from collections import defaultdict
import copy
import multiprocessing
import random
lifetime = [0, 1, 2, 3, -1]

registers = ["rax", "rcx", "rdx", "rbx", "rsp", "rdi", "rbp"]
start_state = {
  "memory": [0, 0, 0, 0],
  "rax": 0,
  "rbx": 1,
  "rcx": 2,
  "rdx": 3,
  "rsp": -1,
  "rdi": -1,
  "rbp": -1
}

end_state = {
  "memory": [3, 1, 2, -1],
  "rax": 3,
  "rbx": 2,
  "rcx": 1,
  "rdx": 0,
  "rsp": 6,
  "rdi": -1,
  "rbp": -1
}
# mov %rsp %rax

#step1 = find_neighbours(start_state)
# print(step1)

class Function:
  def __init__(self, name, input, output):
    self.name = name
    self.input = input
    self.output = output

minus_1_to_four = Function("minus1", -1, 4)
four_to_five = Function("fourtofive", 4, 5)
five_to_six = Function("fivetosix", 5, 6)

functions = [minus_1_to_four, four_to_five, five_to_six]

def index_functions(functions):
  index = {}
  for function in functions:
    if function.input not in index:
      index[function.input] = []
    index[function.input].append(function)
  return index

function_index = index_functions(functions)

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

  def __eq__(self, other):
    return self.state == other.state

  def __hash__(self):
    
    return hash(str(self.state))

  def __lt__(self, other):
    if other not in fScore:
      other_f = float('inf')
    else:
      other_f = fScore[other]
    if self not in fScore:
      my_f = float('inf')
    else:
      my_f = fScore[self]

    return my_f < other_f

  def __repr__(self):
    return self.instruction


fScore = {}

from collections import Counter
def find_neighbours(node, goal, function_index):

  if node.neighbourscreated:
    return node.neighbours

  node.neighbourscreated = True
  candidates = []
  instructions = []
  movement = dict(node.state)
  movement["memory"] = list(node.state["memory"])
  found = False
  found_function = False
  # candidates.append(Node(movement, node.fScore, "mov ${}, %{}".format(item, "rax")))

  for register in registers:
    if node.state[register] == goal.state[register]:
      continue
    c = Counter(filter(lambda x: not type(x) == list, node.state.values()))
    
    d = Counter(filter(lambda x: not type(x) == list, goal.state.values()))
    if c[movement[register]] == 1 and d[movement[register]] > 0:
            continue
    movement3 = dict(movement)
    movement3["memory"] = list(node.state["memory"])
  
    movement3[register] = -1
    # movement["rax"] = -1
    candidates.append(
      Node(
        movement3, node.fScore,
        "mov $-1, %{}".format(register)))
  
  for source_index, source_register in enumerate(registers):
    
    if node.state[source_register] in function_index:
      # we have found a source parameter
      for candidate_function in function_index[node.state[source_register]]: 
        for destination_index, destination_register in enumerate(registers):
          if node.state[destination_register] == goal.state[destination_register]:
            continue
          for key, value in movement.items():
            if value == candidate_function.output:
              # we already have this function's output available
              continue
          # found_valid = False
          # for impossible_function in function_index[candidate_function.output]:
          #   if impossible_function.output == goal.state[destination_register]:
          #     found_valid = True

          # if not found_valid:
          #   continue
          c = Counter(filter(lambda x: not type(x) == list, node.state.values()))
          d = Counter(filter(lambda x: not type(x) == list, goal.state.values()))
          if c[movement[destination_register]] == 1 and d[movement[destination_register]] > 0:
            continue
          movement = dict(node.state)
          movement["memory"] = list(node.state["memory"])
          movement[destination_register] = candidate_function.output
          
          # print(movement[destination_register])
          looking = True
          instructions = []
          instructions.append("call %{}({}) -> {}".format(candidate_function.name, node.state[source_register], candidate_function.output))
          # while looking:
          #   if movement[destination_register] not in function_index or len(function_index[movement[destination_register]]) == 0:
          #     looking = False
          #     break
          #   for candidate_function in     function_index[movement[destination_register]]:
          #     movement[destination_register] = candidate_function.output
          #     instructions.append("call %{}({}) -> {}".format(candidate_function.name, movement[destination_register], candidate_function.output))
          
          # movement["rax"] = -1
          candidates.append(
            Node(
              movement, node.fScore, " ".join(instructions)
              ))
          found_function = True
          break
        if found_function:
          break
    if found_function:
      break
          

  if node.state["memory"] != goal.state["memory"]:
    for memory_location, value_in_memory in enumerate(goal.state["memory"]):
      if node.state["memory"][memory_location] != goal.state["memory"][
          memory_location]:
        for memory_location_key, memory_location_register in node.state.items(
        ):

          # We found a memory location in a register
          if memory_location_key != "memory":

            if memory_location_register == memory_location:
              for current_key, current_value in node.state.items():
                if current_key != "memory":
                  # We found a register that matches the desired memory value in memory
                  if current_value == value_in_memory:
                    # print("found wanted value {}".format(value_in_memory))
                  
                    
                    movement["memory"][memory_location] = value_in_memory
                    # movement["rax"] = -1
                    moveinstruction =  "mov %{}, (%{})".format(memory_location_key,
                                                current_key)
                    instructions.append(moveinstruction)
                    candidates.append(
                     Node(
                       movement, node.fScore,
                           " ".join(instructions)))
                    found = True
                    break
              if found:
                break
        if found:
          break

  for key, value in node.state.items():

    if key == "memory":
      continue

 
    for register in registers:
      c = Counter(filter(lambda x: not type(x) == list, node.state.values()))
      d = Counter(filter(lambda x: not type(x) == list, goal.state.values()))
      if c[movement[key]] == 1 and d[movement[key]] > 0:
        continue
      
      movement2 = dict(node.state)
 
      movement2["memory"] = list(movement["memory"])
      movement2[key] = movement2[register]
      # movement2[register] = -1
      myinstructions = []
      myinstructions.append("mov %{}, %{}".format(register, key))
      candidates.append(
      Node(movement2, node.fScore, " ".join(myinstructions)))
        

  node.neighbours = candidates
  
  # random.shuffle(candidates)
  # print(candidates)
  return candidates

start_node = Node(start_state, fScore, "start")
end_node = Node(end_state, fScore, "end")


def h(start, goal):
  total = 0
  for key, value in start.state.items():
    if key == "memory":
      for index, item in enumerate(start.state[key]):
        if start.state[key][index] != goal.state[key][index]:
          # total = total + (goal.state[key][index] - goal.state[key][index])
          total = total + 1
        
    else:

      if value != goal.state[key]:
        total = total + 1
      

 
  
  
  return total


def same(current, goal):
  if current.state == goal.state:
    return True
  return False


def update_node(my_id, openSet, neighbour, gScore, myCameFrom, fScore, current,
                queue, queues, end):
  tentative_gScore = gScore[current] + 1  # d(current, neighbor)
  # print(tentative_gScore)
  if tentative_gScore < gScore[neighbour]:
    # This path to neighbor is better than any previous one. Record it!
    # print("one has lower cost")
    myCameFrom[neighbour] = current
    gScore[neighbour] = tentative_gScore
    fScore[neighbour] = tentative_gScore + h(neighbour, end)
    if neighbour not in openSet:
      heapq.heappush(openSet, neighbour)
      for index, queue in enumerate(queues):
        if my_id != index:
          # pass
            queue.put((my_id, "update", (neighbour, gScore[neighbour],
                                        fScore[neighbour], True)))
    else:
      for index, queue in enumerate(queues):
        if my_id != index:
          # pass
            queue.put((my_id, "update", (neighbour, gScore[neighbour],
                                        fScore[neighbour], False)))


# A* finds a path from start to goal.
# h is the heuristic function. h(n) estimates the cost to reach goal from node n.
def A_Star(start, goal, h, fScore, worker_len, _function_index):

  def a_worker(my_id, my_queue, parent, queues, function_index):
    # parent.put(("hello", 6))
    origin = None
    myCameFrom = {}
    openSet = []
    end = None
    received = my_queue.get()
    my_queue.task_done()
    print(received)
    if received:

      _my_id, work, args = received
      

      if work == "start":
        origin = args[0]
        end = args[1]
      if work == "newnode":
        origin = args[0]
        end = args[1]
        heapq.heappush(openSet, args[0])



    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = defaultdict(lambda: float('inf'))
    gScore[origin] = 0

    # For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
    # how cheap a path could be from start to finish if it goes through n.
    fScore[origin] = h(origin, end)
    print(fScore[origin])
    thread_running = True
    found = False
    while thread_running:
      # print("thread running loop")

      for i in range(0, 30):
        try:
          # print("receive test")

          received = my_queue.get_nowait()
          
          # print("Process received {}".format(received))
          if received:
            my_queue.task_done()
            _my_id, work, args = received
            
            if work == "finish":
              thread_running = False
              
              # try:
              #     while not my_queue.empty():
              #       my_queue.get_nowait()
              # except:
              #     pass
              # my_queue.close()
              print("thread received stop message")

              parent.put((my_id, "finished", (myCameFrom, None)))
              return
            if work == "newnode":
              current = args[1]
              update_node(my_id, openSet, args[0], gScore, myCameFrom, fScore, current, my_queue, queues, end)

              # heapq.heappush(openSet, args[0])
            if work == "update":
              # print("updating values")
              neighbour = args[0]
              gScore[neighbour] = args[1]
              fScore[neighbour] = args[2]
              # heapq.heappop(neighbour) # the node in openSet having the lowest fScore[] value

             
              # heapq.heappush(openSet, neighbour)
              # heapq.heapify(openSet)

        except Exception as e:
        
          pass


      while len(openSet) > 0:
        # print("openset loop")
        for i in range(0, 30):
          try:
            received = my_queue.get_nowait()
            my_queue.task_done()
            # print("Process received {}".format(received))
            if received:
              _my_id, work, args = received
              if work == "finish":

                # print("received request to finish")


                thread_running = False
                # try:
                #   while not my_queue.empty():
                #     my_queue.get_nowait()
                # except:
                #   pass
                # my_queue.close()
                print("thread received stop message {}".format(my_id))
                parent.put((my_id, "finished", (myCameFrom, None)))
                return
              if work == "newnode":
                current = args[1]
                update_node(my_id, openSet, args[0], gScore, myCameFrom,
                            fScore, current, my_queue, queues, end)
                # heapq.heappush(openSet, args[0])
              if work == "update":

                neighbour = args[0]
                gScore[neighbour] = args[1]
                fScore[neighbour] = args[2]
                # heapq.heapify(openSet)
                # heapq.heappush(openSet, neighbour)
          except Exception as e:

            pass

        # This operation can occur in O(Log(N)) time if openSet is a min-heap or a priority queue
        current = heapq.heappop(openSet) # the node in openSet having the lowest fScore[] value
        # heapq.heappush(openSet, current)
        # print(current.state)
        if same(current, end):
            print("found goal")
            for index, queue in enumerate(queues):
              if index != my_id:

                queue.put((index, "finish", 0))
                print("asked {} to stop".format(index))
            parent.put((my_id, "finished", (myCameFrom, current)))
            thread_running = False
            found = True
            
           
            # print("returning")
            break
            
            
            

     
        # openSet.remove(current)
        for index, neighbour in enumerate(find_neighbours(current, end, function_index)):
          
          if worker_len > index % worker_len == my_id:

            for index, queue in enumerate(queues):
              if index != my_id:
                queue.put((my_id, "newnode", (neighbour, current)))
          else:

            # print("running")
            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current

            update_node(my_id, openSet, neighbour, gScore, myCameFrom, fScore,
                        current, my_queue, queues, end)

    if not found:
      parent.put((my_id, "finished", (myCameFrom, None)))
    print("finished worker method")

  queues = []
  workers = []
  manager = multiprocessing.Manager()

  parent = manager.Queue()
  for i in range(0, worker_len):
    queue = manager.Queue()
    queues.append(queue)
  for i in range(0, worker_len):
    print("creating worker {}".format(i))

    worker = multiprocessing.Process(target=a_worker,
                                     args=(i, queues[i], parent, queues, _function_index))
    workers.append(worker)
    worker.start()
    # The set of discovered nodes that may need to be (re-)expanded.
    # Initially, only the start node is known.
    # This is usually implemented as a min-heap or priority queue rather than a hash-set.
  queues[0].put((0, "newnode", (start, goal)))
  for index, item in enumerate(queues[1:]):
    item.put((index, "start", (start, goal)))

  running = True
  counter = 0
  results = []
  comeFrom = {}
  current = None
  while running:
      received = parent.get()
      parent.task_done()
      _my_id, work, args = received
      if received:
        # print("Received {} {}".format(work, args))
        if work == "finished":
            # print("a thread finished")
            counter = counter + 1
            results.append(args)
            if counter == len(workers):
              print("all threads finished")
              
              
              running = False
              # while not parent.empty():
              #   parent.get()
              # parent.close()
              
           
              
            continue
        
        # print("Done {}".format(received))
        

  chosen_result = None
  for result in results:
    comeFrom.update(result[0])
    if result[1] is not None:
      chosen_result = result[1]

  print("finished control loop")
  # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from the start
  # to n currently known.

  for queue in queues:
    pass  # queue.close()
  print("asked all queues to stop")

  for index, worker in enumerate(workers):
    print("trying to join worker {}".format(index))
    worker.join()
    print("joined worker")
  if chosen_result:
    return reconstruct_path(comeFrom, chosen_result)
  # Open set is empty but goal was never reached
  return "failure"


if __name__ == "__main__":
  search = A_Star(start_node, end_node, h, fScore, 8, function_index)
  print(search)
  print(search[-1].state)
