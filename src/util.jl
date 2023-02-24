function distance(x1::Int64, y1::Int64, x2::Int64, y2::Int64)::Int64
    return ceil(Int64, sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)))
end
