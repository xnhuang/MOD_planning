NUMSTUDY = 200
CURSTUDY = 0
include("constants.jl")
include("/media/mSATA/UM/Upper routing simulation/utils.jl")
using PyCall
using DataFrames
using CSV


@pyimport sys
@pyimport os
unshift!(PyVector(pyimport("sys")["path"]), os.path[:join](SUMO_HOME, "tools"))
unshift!(PyVector(pyimport("sys")["path"]), "/media/mSATA/UM/Simulation")
@pyimport mvn_gmr
@pyimport traci
@pyimport traci.constants as tc
@pyimport sumolib
@pyimport numpy as np

hevgmm = mvn_gmr.GMM(np.array(GMMHEVMEANS), np.array(GMMHEVCOVS), np.array(GMMHEVWEIGHTS))
evgmm = mvn_gmr.GMM(np.array(GMMEVMEANS), np.array(GMMEVCOVS), np.array(GMMEVWEIGHTS))
hevgmr = mvn_gmr.GMR(hevgmm)
evgmr = mvn_gmr.GMR(evgmm)

vehicleStatus = Dict()
backgroundStatus = Dict()
netStatus = Dict()

ANNARBOR = sumolib.net[:readNet]("/media/mSATA/UM/Simulation/f_AnnArbor1.6.net.xml")
EDGES = ANNARBOR[:getEdges]()
NET = Dict{String,Array{Any}}()
for e in EDGES
    NET[e[:getID]()]  = [-1, -1, "-1"]
end
NETLOOKUP = Dict()
for e in EDGES
    NETLOOKUP[e[:getID]()]  = e
end
link_grades = Dict()
for (idx,ln) in enumerate(data_grades[:link])
    link_grades[ln] = idx
end
NETGRADE = Dict()
NETDIST = Dict()
NETSET = Set()

open("/media/mSATA/UM/Simulation/finalclusters.txt") do f
    this_zone = -1
    for line in eachline(f)
        res = split(line)
        if res[1] == "Zone"
            this_zone = parse(Int, res[2])
        elseif res[1] == "Links"
            for lidx in res[2:end]
                ln = data_links[:link][parse(Int, lidx)]
                if string(ln, "-0") in keys(NET)
                    push!(NETSET, string(ln, "-0"))
                    NET[string(ln, "-0")] = [this_zone, ln, string(ln, "-0")]
                    NETDIST[string(ln, "-0")] = NETLOOKUP[string(ln, "-0")][:getLength]()
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-0")] = data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-0")] = mean(data_grades[:meangrade])
                    end
                end
                if string(ln, "-1") in keys(NET)
                    push!(NETSET, string(ln, "-1"))
                    NET[string(ln, "-1")] = [this_zone, ln, string(ln, "-1")]
                    NETDIST[string(ln, "-1")] = NETLOOKUP[string(ln, "-1")][:getLength]()
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-1")] = -data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-1")] = mean(data_grades[:meangrade])
                    end
                end
                if string(ln, "-1-Pocket") in keys(NET)
                    NET[string(ln, "-1-Pocket")] = [this_zone, ln, string(ln, "-1")]
                    NETDIST[string(ln, "-1-Pocket")] = NETLOOKUP[string(ln, "-1-Pocket")][:getLength]()
                    NETDIST[string(ln, "-1")] += NETDIST[string(ln, "-1-Pocket")]
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-1-Pocket")] = -data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-1-Pocket")] = mean(data_grades[:meangrade])
                    end
                end
                if string(ln, "-0-Pocket") in keys(NET)
                    NET[string(ln, "-0-Pocket")] = [this_zone, ln, string(ln, "-0")]
                    NETDIST[string(ln, "-0-Pocket")] = NETLOOKUP[string(ln, "-0-Pocket")][:getLength]()
                    NETDIST[string(ln, "-0")] += NETDIST[string(ln, "-0-Pocket")]
                    if ln in keys(link_grades)
                        NETGRADE[string(ln, "-0-Pocket")] = data_grades[:meangrade][link_grades[ln]]
                    else
                        NETGRADE[string(ln, "-0-Pocket")] = mean(data_grades[:meangrade])
                    end
                end
            end
        end
    end
end
function _checkBackInitialPositions(vehicleID, start, dest, edge, pos, time, speed)
 if !(vehicleID in keys(backgroundStatus))
   backgroundStatus[vehicleID] = BackStatus(Int(time/1000), [edge], [speed], [Int(time/1000)], Int[], [speed], Float64[], Float64[], Float64[], false)
 end
end
function _checkInitialPositions(vehicleID, start, dest, edge, time)
 if !(vehicleID in keys(vehicleStatus))
  # a new vehicle not in the analysis list
  global CURSTUDY
  if CURSTUDY < NUMSTUDY && start == edge
   # make sure this new vehicle is the first 200 cars and at the start of its route
   vehicleStatus[vehicleID] = Status(start, dest, Int(time/1000), edge, false)
   CURSTUDY += 1
  end
 end
end
time_link_cost = Dict{Int, Dict{String, Array{Float64, 1}}}()
time_link_count = Dict{Int, Dict{String, Int}}()
function doStep(datas)
    traci.simulationStep()
    subscribes = traci.vehicle[:getSubscriptionResults]()
    time = traci.simulation[:getCurrentTime]()

    if time%(60*15*1000) == 0
        time_link_cost[time/1000] = Dict{String, Array{Float64, 1}}()
        time_link_count[time/1000] = Dict{String, Int}()


        for (veh, subs) in subscribes
            edge = traci.vehicle[:getRoadID](veh)
            speed = traci.vehicle[:getSpeed](veh)
            try
                if NETLOOKUP[edge][:getPriority]() >= 6
                    spdlim = NETLOOKUP[edge][:getSpeed]()
                    datas["bar2"][NET[edge][1]] += 1
                    datas["bar3"][NET[edge][1]] += speed/spdlim
                    datas["bar3count"][NET[edge][1]] += 1
                end
                if backgroundStatus[veh].edge[end] != edge && !(contains(backgroundStatus[veh].edge[end], edge)||contains(edge, backgroundStatus[veh].edge[end]))
                    push!(backgroundStatus[veh].edge, edge)
                    push!(backgroundStatus[veh].endtime, backgroundStatus[veh].step)
                    push!(backgroundStatus[veh].starttime, Int(time/1000))
                    push!(backgroundStatus[veh].endspeed, backgroundStatus[veh].speed[end])
                    push!(backgroundStatus[veh].startspeed, speed)
                    push!(backgroundStatus[veh].meanspeed, mean(backgroundStatus[veh].speed))
                    push!(backgroundStatus[veh].stdspeed, isnan(std(backgroundStatus[veh].speed))?0.0:std(backgroundStatus[veh].speed))
                    backgroundStatus[veh].speed = Float64[]
                end
                backgroundStatus[veh].step = Int(time/1000)
                push!(backgroundStatus[veh].speed, speed)
            end
        end
        cost = Dict("edge"=>[],"meanspd"=>[],"meangrade"=>[],"stdspd"=>[],"initsoc"=>[],"spdlim"=>[],"dynamic"=>[])

        for vid in keys(backgroundStatus)
            for (idx, e) in enumerate(backgroundStatus[vid].edge)
                if idx <= length(backgroundStatus[vid].endtime)
                    if e in keys(NET)
                        push!(cost["edge"], NET[e][3])
                        push!(cost["meanspd"], backgroundStatus[vid].meanspeed[idx])
                        push!(cost["meangrade"], NETGRADE[e])
                        push!(cost["stdspd"], backgroundStatus[vid].stdspeed[idx])
                        push!(cost["initsoc"], 0.7)
                        push!(cost["spdlim"], NETLOOKUP[e][:getSpeed]())
                        push!(cost["dynamic"], ((backgroundStatus[vid].endspeed[idx])^2-(backgroundStatus[vid].startspeed[idx])^2)/NETDIST[e])
                    end
                end
            end
            if backgroundStatus[vid].arrived
                delete!(backgroundStatus, vid)
            else
                backgroundStatus[vid].edge = [backgroundStatus[vid].edge[end]]
                backgroundStatus[vid].endtime = []
                backgroundStatus[vid].starttime = [backgroundStatus[vid].starttime[end]]
                backgroundStatus[vid].endspeed = []
                backgroundStatus[vid].startspeed = [backgroundStatus[vid].startspeed[end]]
                backgroundStatus[vid].meanspeed = []
                backgroundStatus[vid].stdspeed = []
            end
        end

        for edge in NETSET
            time_link_cost[time/1000][edge] = zeros(Float64, 5)
            time_link_cost[time/1000][edge][1] = NETDIST[edge]
            time_link_count[time/1000][edge] = 0
            if !(edge in cost["edge"])
                push!(cost["edge"], edge)
                push!(cost["meanspd"], traci.edge[:getLastStepMeanSpeed](edge))
                push!(cost["meangrade"], NETGRADE[edge])
                push!(cost["stdspd"], 0.0)
                push!(cost["initsoc"], 0.7)
                push!(cost["spdlim"], NETLOOKUP[edge][:getSpeed]())
                push!(cost["dynamic"], 0.0)
            end
        end

        costresult = compute_energy(cost["meanspd"],
                                        cost["meangrade"],
                                        cost["stdspd"],
                                        cost["initsoc"],
                                        cost["spdlim"],
                                        cost["dynamic"])
        for (idx, ln) in enumerate(cost["edge"])
            time_link_cost[time/1000][ln][2] += costresult[2][idx]/(8.8*0.05)
            time_link_cost[time/1000][ln][3] += costresult[2][idx]
            time_link_cost[time/1000][ln][4] += costresult[1][idx]
            time_link_cost[time/1000][ln][5] += cost["meanspd"][idx]
            time_link_count[time/1000][ln] += 1
        end
    else
        for (veh, subs) in subscribes
            edge = traci.vehicle[:getRoadID](veh)
            speed = traci.vehicle[:getSpeed](veh)
            if veh in keys(backgroundStatus)
                if backgroundStatus[veh].edge[end] != edge && !(contains(backgroundStatus[veh].edge[end], edge)||contains(edge, backgroundStatus[veh].edge[end]))
                 push!(backgroundStatus[veh].edge, edge)
                 push!(backgroundStatus[veh].endtime, backgroundStatus[veh].step)
                 push!(backgroundStatus[veh].starttime, Int(time/1000))
                 push!(backgroundStatus[veh].endspeed, backgroundStatus[veh].speed[end])
                 push!(backgroundStatus[veh].startspeed, speed)
                 push!(backgroundStatus[veh].meanspeed, mean(backgroundStatus[veh].speed))
                 push!(backgroundStatus[veh].stdspeed, isnan(std(backgroundStatus[veh].speed))?0.0:std(backgroundStatus[veh].speed))
                 backgroundStatus[veh].speed = Float64[]
                end
                backgroundStatus[veh].step = Int(time/1000)
                push!(backgroundStatus[veh].speed, speed)
            end
        end
    end

    departed = traci.simulation[:getSubscriptionResults]()[tc.VAR_DEPARTED_VEHICLES_IDS]
    arrived = traci.simulation[:getSubscriptionResults]()[tc.VAR_ARRIVED_VEHICLES_IDS]
    for v in departed
        traci.vehicle[:subscribe](v)
        #subs = traci.vehicle[:getSubscriptionResults](v)
        route = traci.vehicle[:getRoute](v)
        speed = traci.vehicle[:getSpeed](v)
        pos = traci.vehicle[:getPosition](v)
        start = route[1]
        dest = route[end]
        edge = traci.vehicle[:getRoadID](v)
        _checkBackInitialPositions(v, start, dest, edge, pos, time, speed)
        _checkInitialPositions(v, start, dest, edge, time)
    end
    for v in arrived
        if v in keys(backgroundStatus)
            if backgroundStatus[v].arrived==false
                backgroundStatus[v].arrived = true
                push!(backgroundStatus[v].endtime, backgroundStatus[v].step)
                push!(backgroundStatus[v].endspeed, backgroundStatus[v].speed[end])
                push!(backgroundStatus[v].meanspeed, mean(backgroundStatus[v].speed))
                push!(backgroundStatus[v].stdspeed, std(backgroundStatus[v].speed))
            end
        end
        if v in keys(vehicleStatus)
            if vehicleStatus[v].arrived == false
                vehicleStatus[v].arrived = true
                 global CURSTUDY -= 1
            end
        end
    end
    if time%(60*30*1000) == 0
        for step in sort(collect(keys(time_link_cost)))
            for ln in keys(time_link_cost[step])
                push!(datas["vehicleObs"], [ln, step, time_link_cost[step][ln][1]/(time_link_cost[step][ln][5]/time_link_count[step][ln] + 0.000001),
                                                     time_link_cost[step][ln][2]/time_link_count[step][ln],
                                                     time_link_cost[step][ln][3]/time_link_count[step][ln],
                                                     time_link_cost[step][ln][4]/time_link_count[step][ln],
                                                     time_link_cost[step][ln][1]])
            end
        end
        CSV.write("/media/mSATA/UM/Upper routing simulation/SUMOdata/sumoweights_$(time/1000).csv", datas["vehicleObs"])
        datas["vehicleObs"] = DataFrame(Link=String[], Step=Int[], TravelTime=Float64[], SOC=Float64[], EV=Float64[], HEV=Float64[], Dist=Float64[])
        global time_link_cost = Dict{Int, Dict{String, Array{Float64, 1}}}()
        global time_link_count = Dict{Int, Dict{String, Int}}()
        for v in keys(vehicleStatus)
            if vehicleStatus[v].arrived
                push!(datas["CvehicleObs"], [v, vehicleStatus[v].start, vehicleStatus[v].dest, vehicleStatus[v].step])
                delete!(vehicleStatus, v)
            end

        end
        CSV.write("/media/mSATA/UM/Upper routing simulation/SUMOdata/sumovehicles_$(time/1000).csv", datas["CvehicleObs"])
        datas["CvehicleObs"] = DataFrame(VehID = String[], Start = String[], Dest = String[], StartTime = Int[])
    end
end
function main(T)
 sumoExe = SUMO
 sumoConfig = SUMO_CONFIG
 traci.start([sumoExe, "-c", sumoConfig])
 traci.simulation[:subscribe](varIDs=(tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS))
 datas = Dict{String, Any}()
 datas["vehicleObs"] = DataFrame(Link=String[], Step=Int[], TravelTime=Float64[], SOC=Float64[], EV=Float64[], HEV=Float64[], Dist=Float64[])
 datas["CvehicleObs"] = DataFrame(VehID =String[], Start = String[], Dest = String[], StartTime = Int[])

 #traci.vehicle[:subscribe]("1", (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
 t = 0
 while t<T
  doStep(datas)
  t+=1
 end
 traci.close()
end
