mutable struct Vehicle
    vehicle_id :: Int
    capacity :: Int
    location
    onboard
    trip
    on_link_time
    link_travel_time
    current_location_list
    pickup_location
    dropoff_location
    rebalance :: Int
    departed :: Int
    trajectory
    in_use :: Int
    distance_traveled :: Float64
end
Vehicle(vehicle_id::Int, capacity::Int, location, onboard, trip) = Vehicle(vehicle_id, capacity, location, onboard, trip, 0,0,0,0,[location],0)
function depart_vehicle(vehicle::Vehicle, map)
    nothing
end
function park_vehicle(vehicle::Vehicle)
    nothing
end
