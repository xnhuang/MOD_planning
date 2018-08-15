mutable struct Customer
    customer_id :: Int
    origin :: String
    destination :: String
    max_wait_time :: Float64
    max_delay_time :: Float64
    rebalance :: Int
    ignore :: Int
    assign :: Int
    in_pool_time :: Float64 
    pickup_time :: Float64
    delivery_time :: Float64
    virtual
end
Customer(customer_id::Int,
         origin::String,
         destination::String,
         max_wait_time::Float64,
         max_delay_time::Float64,
         virtual) = Customer(customer_id, origin, destination, max_wait_time, max_delay_time, 0,0,0,0,-1,-1,virtual)
