# sliding-puzzle-codegen-memory

 * Live demo: https://replit.com/@Chronological/SlidingPuzzle3

This is a experimental project that shows generating assembly code based on the movement of values through time.

This uses A* graph search algorithm to generate assembly code. I call it program synthesis. In ChatGPT you can get ChatGPT to generate code, so this kind of already exists but this is a much more straightforward approach.

See this GIF https://giphy.com/embed/uOb99bhj98Mgi0UeBl

This is a sliding puzzle. Programming is often just moving data into the right places to get the computer to do the right thing. If you call a a function with the right arguments, you get the expected result. This program is based on the intuition that programming is largely just logistics.


My program synthesis/codegeneration is for searching through state space of a program to allocate variables to values and arrange state for function calls. It is meant to automate the boring part of programming which is boilerplate and moving things into place. It learns the hidden states - the function calls to get the register and memory to be what they should be.

My program takes two states: a beginning state and and end state, including memory locations and infers the instructions used to reach the end state. Functional programmers love types, I use the idea of types but value tracing.
```
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

```

If you run the program with these values, this generates the following sequence of instructions. The A* search works out that it has to move things into memory and then call multiple functions in a row to get to the end state. It "slides things around until it looks right"

```
[start, mov %rax, (%rdx), mov %rbx, (%rbx), mov %rcx, (%rcx), mov %rdx, (%rsp), mov %rax, %rsp, mov %rdx, %rax, mov %rsp, %rdx, mov %rcx, %rsp, mov %rbx, %rcx, mov %rsp, %rbx, call minus1(rdi=-1) -> rsp=4, call fourtofive(rsp=4) -> rsp=5, call fivetosix(rsp=5) -> rsp=6]
```

The program worked out that it had to `mov %rax, (%rdx)` because %rax contains 0 and %rdx is meant to contain 0.

```
  # These functions take in an argument of value type given by that number and return a value type given by the second paramter.
  minus_1_to_four = Function("minus1", -1, 4)
  four_to_five = Function("fourtofive", 4, 5)
  five_to_six = Function("fivetosix", 5, 6)
```

# Understanding the code

Some interesting parts of the code:
|Function|Description|
|---|---|
|def find_neighbours|This is where we generate dynamic neighbours. The neighbours are then chunked with a modulo operator depending on what thread we're on|
|def A_Star):|This function is called by the main thread, it starts multiple multiprocessing `Process` that run `a_worker` in a separate OS process. |
|a_worker|Communicates to the A_star main method with a multitprocessing Queue.|

The main method and the a_worker have a simple protocol for communication over queues.
* a tuple with "finished" as the first argument means the worker is finished, this is sent by the worker to the main thread.
* The main thread sends a "newnode" request to each worker to tell them to start finding paths from this node.

# Parallel A*

This repository implements parallel A*. We take advantage that we can produce different set of neighbours per thread.

# Ideas

* We can analyse the lexer stream of a codebase and generate a graph that resembles the probability of next likelihood token.
