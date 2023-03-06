using JuMP, Gurobi, MathOptInterface
using LinearAlgebra
using Graphs

function solve_tsp_dfj(data::TSPData)::Solution
    nnodes = size(data.cost, 1)
    m = Model(Gurobi.Optimizer)
    @variable(m, x[1 : nnodes, 1 : nnodes], Bin)
    @objective(m, Min, dot(data.cost, x))
    @constraint(m, inflow[i in 1 : nnodes], sum(x[:, i]) == 1)
    @constraint(m, outflow[i in 1 : nnodes], sum(x[i, :]) == 1)
    
    function heuristic_callback(cb_data)
        xval = callback_value.(cb_data, x)
        # apply a very simple rounding heuristic
        let 
            sol = simple_rounding_heuristic(data, xval)
            if !isnothing(sol)
                yval = zeros(Int64, nnodes, nnodes)
                for k in 1 : nnodes
                    i, j = sol.from[k], sol.to[k]
                    yval[i, j] = 1
                end
                status = MOI.submit(m, MOI.HeuristicSolution(cb_data), vec(x), vec(yval))
                println("Simple rounding heuristic found a solution, status = $status")
            end
        end
    end

    function lazy_callback(cb_data)
        status = callback_node_status(cb_data, m)
        if status == MOI.CALLBACK_NODE_STATUS_FRACTIONAL
            # `callback_value(cb_data, x)` is not integer (to some tolerance).
            # If, for example, your lazy constraint generator requires an
            # integer-feasible primal solution, you can add a `return` here.
            return
        elseif status == MOI.CALLBACK_NODE_STATUS_INTEGER
            # `callback_value(cb_data, x)` is integer (to some tolerance).
        else
            @assert status == MOI.CALLBACK_NODE_STATUS_UNKNOWN
            # `callback_value(cb_data, x)` might be fractional or integer.
        end
        xval = callback_value.(cb_data, x)
        cut = separate_integer_capcut(data, xval)
        if !isnothing(cut) && !isempty(cut)
            println("Integer capcut found = $cut")
            con = @build_constraint(sum(x[i, j] for i in cut, j in 1 : nnodes if !in(j, cut)) >= 1)
            MOI.submit(m, MOI.LazyConstraint(cb_data), con)
        end
    end

    MOI.set(m, MOI.HeuristicCallback(), heuristic_callback)
    MOI.set(m, MOI.LazyConstraintCallback(), lazy_callback)

    optimize!(m)

    from = Int64[]
    to = Int64[]
    cost = 0
    if termination_status(m) == MOI.OPTIMAL
        xval = round.(Int64, value.(x))
        for i in 1 : nnodes, j in 1 : nnodes
            if xval[i, j] == 1
                push!(from, i)
                push!(to, j)
                cost += data.cost[i, j]
            end
        end
    end
    return Solution(from, to, cost)
end

function separate_integer_capcut(data::TSPData, x)::Union{Nothing, Set{Int64}}
    nnodes = size(x, 1)
    y = round.(Int64, x)
    if maximum(abs.(x - y)) > 1e-6 # solution is fractional!
        return nothing
    end
    # compute the connected components induced by y
    g = Graphs.SimpleGraph(nnodes)
    for i in 1 : nnodes, j in i + 1 : nnodes
        if y[i, j] + y[j, i] >= 1
            add_edge!(g, i, j)
        end
    end
    connected_comps = connected_components(g)
    if length(connected_comps) <= 1 #graph is connected, found nothing!
        return nothing
    else # return the cut of the first connected component
        return Set(connected_comps[1])
    end
end
    

function simple_rounding_heuristic(data::TSPData, x)::Union{Nothing, Solution}
    nnodes = size(x, 1)
    y = round.(Int64, x)
    # check that y forms a connected graph
    g = Graphs.SimpleGraph(nnodes)
    for i in 1 : nnodes, j in i + 1 : nnodes
        if y[i, j] + y[j, i] >= 1
            add_edge!(g, i, j)
        end
    end
    if !is_connected(g) # not connected so we return nothing
        return nothing
    else
        covered = falses(nnodes)
        covered[1] = true
        route = [1]
        while length(route) < nnodes
            last = route[end]
            infloop = true
            for j in 1 : nnodes
                if !covered[j]
                    if y[last, j] + y[j, last] == 1
                        push!(route, j)
                        covered[j] = true
                        infloop = false
                        break
                    end
                end
            end
            if infloop # infinite loop so we cananot detect a solution here, return nothing
                return nothing
            end
        end
        push!(route, 1) # close the loop
        #build a solution to return
        let
            to = []
            from = []
            cost = 0
            for k in 1 : nnodes
                i, j = route[k], route[k + 1]
                push!(from, i)
                push!(to, j)
                cost += data.cost[i, j]
            end
            return Solution(from, to, cost)
        end
    end
    return nothing # we shouldn't have arrived here, but it means we found nothing, so return nothing
end
