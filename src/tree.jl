using LinearAlgebra
function solve_tsp_tree(data::TSPData)::Solution
    nnodes = size(data.cost, 1)
    m = Model(Gurobi.Optimizer)
    @variable(m, x[1 : nnodes, 1 : nnodes], Bin)
    @variable(m, y[1 : nnodes, 1 : nnodes] >= 0)
    @objective(m, Min, dot(data.cost, x))
    @constraint(m, inflow[i in 1 : nnodes], sum(x[:, i]) == 1)
    @constraint(m, outflow[i in 1 : nnodes], sum(x[i, :]) == 1)
    @constraint(m, connectivity, sum(y[1, :]) == nnodes - 1)
    @constraint(m, forest[i in 2 : nnodes], sum(y[:, i]) - sum(y[i, :]) == 1)
    @constraint(m, link[i in 1 : nnodes, j in 1 : nnodes], y[i, j] - (nnodes - 1) * x[i, j] <= 0)
    optimize!(m)
    xval = round.(Int64, value.(x))
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