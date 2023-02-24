struct TSPData
    pos::Matrix{Int64}
    cost::Matrix{Int64}
end

struct Solution
    from::Vector{Int64}
    to::Vector{Int64}
    cost::Int64
end