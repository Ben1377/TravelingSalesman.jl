using Random
function generate_random(n::Int64, seed::UInt64 = rand(UInt64))::TSPData
    rng = MersenneTwister(seed)
    pos = rand(rng, 0 : 1000, n, 2)
    cost = 1000000 * ones(Int64, n, n)
    for i in 1 : n - 1, j in i + 1 : n
        d = distance(pos[i, 1], pos[i, 2], pos[j, 1], pos[j, 2])
        cost[i, j] = d
        cost[j, i] = d
    end
    return TSPData(pos, cost)
end
