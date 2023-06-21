# sliding-puzzle-codegen

This is a experimental project that shows generating assembly code based on the movement of values through time.

See this GIF https://giphy.com/embed/uOb99bhj98Mgi0UeBl

I recently wrote a program that uses the A* graph search algorithm to generate assembly code. I call it program synthesis. In ChatGPT you can get ChatGPT to generate code, so this kind of already exists.

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
  # These functions take in an argument of value type given by that number and return a value type given by the second paramter.
  minus_1_to_four = Function("minus1", -1, 4)
  four_to_five = Function("fourtofive", 4, 5)
  five_to_six = Function("fivetosix", 5, 6)
```

This generates the following sequence of instructions.

```
[start, mov %rax, (%rdx), mov %rbx, (%rbx), mov %rcx, (%rcx), mov %rdx, (%rsp), mov %rax, %rsp, mov %rdx, %rax, mov %rsp, %rdx, mov %rcx, %rsp, mov %rbx, %rcx, mov %rsp, %rbx, call minus1(rdi=-1) -> rsp=4, call fourtofive(rsp=4) -> rsp=5, call fivetosix(rsp=5) -> rsp=6]
```

# Parallel A*

This repository implements parallel A*. We take advantage that we can produce different set of neighbours per thread.

# Ideas

* We can analyse the lexer stream of a codebase and generate a graph that resembles the probability of next likelihood token.
