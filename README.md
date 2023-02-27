# TravelingSalesman.jl
Implementations of several algorithms for the traveling salesman problem in Julia

## Installation
From the Julia REPL execute
```julia
julia> using Pkg
julia> Pkg.add(url="https://github.com/claud10cv/TravelingSalesman.jl.git")
```

## Basic usage
### Load the package
```julia
julia> using TravelingSalesman
```

### Generate a random Euclidean instance of `n` nodes using a seed `s` (random seed if omitted)
```julia
julia> data = TravelingSalesman.generate_random(n, s)
```

### Execute several TSP algorithms
```julia
julia> res = solve_tsp_mtz(data) # solve the TSP using the MTZ-formulation
julia> res = solve_tsp_tree(data) # solve the TSP using the Spanning-tree formulation
```
