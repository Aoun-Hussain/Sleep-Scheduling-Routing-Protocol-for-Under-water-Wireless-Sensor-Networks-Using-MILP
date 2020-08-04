import random
from pulp import *
import time
import math
from networkx.algorithms.approximation.steinertree import steiner_tree 
import networkx as nx



def Optimize(rad, sen):
    width = 100.0              ##100 by 100 meters of square field
    R = float(rad)             ##Radius of communication of each node
    relayConstraint = 121        ##Total relays to be deployed
    sensorList = []
    relayList = []
    

    ##Populating sensorList and relayList
    for i in range(sen):
        sensorList.append((round(random.uniform(0.0,width),6),round(random.uniform(0.0,width),6)))
        # print(sensorList)

    row = 0
    col = 0
    while row <= width:
        while col <= width:
            relayList.append((float(row),float(col)))
            col += 10
        row += 10
        col = 0
        
    ##Calculates the Euclidean distance
    def distance(a,b):
        return math.sqrt((a**2) + (b**2)) 

    ##Calculates the Connectivity matrix for sensor x relay layer
    def Connection_s_r(SN,RN):
        Neighbor_Sensor_Relay = [[[0] for i in range(len(RN))] for j in range(len(SN))]
        for i in range(len(SN)):
            for j in range(len(RN)):
                dist = distance(abs(SN[i][0] - RN[j][0]), abs(SN[i][1]-RN[j][1]))
                if dist <= R:
                    Neighbor_Sensor_Relay[i][j] = 1
                elif dist > R:
                    Neighbor_Sensor_Relay[i][j] = 0
        
        return  Neighbor_Sensor_Relay       
        
    n_s_r = (Connection_s_r(sensorList,relayList))
 
    ###print('Neighbor Matrix of Sensor x Relay Layer', '\n')            
    ###for i in n_s_r:
    ###    print(i)
    
    ##Calculates the Connectivity matrix for relay x relay layer
    def Connection_r_r(RN):
        Neighbor_Relay_Relay = [[[0] for i in range(len(RN))] for j in range(len(RN))]  
        for i in range(len(RN)):
            for j in range(len(RN)):
                dist = distance(abs(RN[i][0] - RN[j][0]), abs(RN[i][1] - RN[j][1]))
                if i == j:
                    Neighbor_Relay_Relay[i][j] = 0
                elif dist <= R:
                    Neighbor_Relay_Relay[i][j] = 1
                elif dist > R:
                    Neighbor_Relay_Relay[i][j] = 0
        
        return  Neighbor_Relay_Relay       
        
    n_r_r = (Connection_r_r(relayList))
 
    ###print('Neighbor Matrix of Relay x Relay Layer', '\n')            
    ###for i in n_r_r:
    ###    print(i)

    ##Calculates the Data link flow matrix for sensor x relay layer
    def Link_s_r(SN,RN):
        bandwidth = 100.0
        Linkflow_Sensor_Relay = [[[0] for i in range(len(RN))] for j in range(len(SN))]
        for i in range(len(SN)):
            for j in range(len(RN)):
                Linkflow_Sensor_Relay[i][j] = bandwidth
        
        return Linkflow_Sensor_Relay

    l_s_r = (Link_s_r(sensorList,relayList))
    
    ###print('Data Link Flow Matrix of Sensor x Relay Layer', '\n')
    ###for i in l_s_r:
    ###    print(i)

    ##Calculates the Data link flow matrix for relay x relay layer
    def Link_r_r(RN):
        bandwidth = 200.0
        Linkflow_Relay_Relay = [[[0] for i in range(len(RN))] for j in range(len(RN))]
        for i in range(len(RN)):
            for j in range(len(RN)):
                if i != j:
                    Linkflow_Relay_Relay[i][j] = bandwidth
                else:
                    Linkflow_Relay_Relay[i][j] = 0
        
        return Linkflow_Relay_Relay

    l_r_r = (Link_r_r(relayList))
    
    ###print('Data Link Flow Matrix of Relay x Relay Layer', '\n')
    ###for i in l_r_r:
    ###    print(i)
            
    ##Calculates the Energy matrix for sensor x relay layer
    def Energy_s_r(SN,RN):
        bandwidth = 100.0
        Energy_Sensor_Relay = [[[0] for i in range(len(RN))] for j in range(len(SN))]
        E_radio_S = 50.0 * (10 ** (-9))
        E_radio_R = 100.0 * (10 ** (-9))
        Transmit_amplifier = 100.0 * (10 ** (-12))
        for i in range(len(SN)):
            for j in range(len(RN)):
                dist = distance(abs(SN[i][0] - RN[j][0]), abs(SN[i][1] - RN[j][1]))
                energy_sensor_tx = float(bandwidth * (E_radio_S + (Transmit_amplifier * (dist **2))))  ##energy used when sensor transmits data
                energy_relay_rx = float(bandwidth * E_radio_R)                                         ##energy used when relay receives data 
                total_energy = energy_sensor_tx + energy_relay_rx
                Energy_Sensor_Relay[i][j] = total_energy                                           ##Only 1/4 sensors will be active per time unit for every relay
                
        return Energy_Sensor_Relay
    
    e_s_r = (Energy_s_r(sensorList,relayList))
    
    # print('Energy Matrix of Sensor x Relay layer', '\n')
    # for i in e_s_r:
    #    print(i)

    ##Calculates the Energy matrix for relay x relay layer
    def Energy_r_r(RN):
        bandwidth = 200.0
        Energy_Relay_Relay = [[[0] for i in range(len(RN))] for j in range(len(RN))]
        E_radio_R = 100.0 * (10 ** (-9))
        Transmit_amplifier = 100.0 * (10 ** (-12))
        for i in range(len(RN)):
            for j in range(len(RN)):
                dist = distance(abs(RN[i][0] - RN[j][0]), abs(RN[i][1] - RN[j][1]))
                energy_relay_tx = float(bandwidth * (E_radio_R + (Transmit_amplifier * (dist**2))))  ##energy used when relay transmits data
                energy_relay_rx = float(bandwidth * E_radio_R)                                       ##energy used when relay receives data 
                total_energy = (energy_relay_tx + energy_relay_rx)
                Energy_Relay_Relay[i][j] = total_energy
                
        return Energy_Relay_Relay

    e_r_r = (Energy_r_r(relayList))
    
    ###print('Energy Matrix of Relay x Relay layer', '\n')
    ###for i in e_r_r:
    ###    print(i)
        

    Problem = LpProblem("The Energy Problem", LpMinimize)
    Var_S_R = [pulp.LpVariable(str('SR' + str(i) + "_" + str(j)), 0, 1, pulp.LpInteger) for i in range(len(sensorList)) for j in range(len(relayList))]
    Var_R_R = [pulp.LpVariable(str('RR' + str(i) + "_" + str(j)), 0, 1, pulp.LpInteger) for i in range(len(relayList)) for j in range(len(relayList))]
    Bool_Var = [pulp.LpVariable(str('B' + str(i)), 0, 1, pulp.LpInteger) for i in range(len(relayList))]
    

    k = 0                ##for variable indexing
    objective = []       ##for objective function
    linkflow_column = [] ##for data rate constraint
    conn_SR = []         ##for connection constraint
    conn_RR = []    

    while k < len(Var_S_R):
        for i in range(len(e_s_r)):
            for j in range(len(e_s_r[i])):
                objective.append(e_s_r[i][j] * Var_S_R[k])
                conn_SR.append(n_s_r[i][j] * Var_S_R[k])
                k += 1   
            Problem += lpSum(conn_SR) == 1
            conn_SR = []
   

    B_Max = 3000                    ##by controlling the maximum data rate we control number of sensors per relay
    B_SR = 100
    S_Per_R = B_Max / B_SR          ##contains the number of sensor per relay constraint applied above (a relay can have a 100 data rate with a sensor)
    Var_alias_SR = []

    for i in Var_S_R:
        Var_alias_SR.append(str(i))
    booleansumcolumn=[]                                                 ##holds the variables for each column at a time
    
    for i in range(len(relayList)):
        for j in range(len(sensorList)):
            e = Var_alias_SR.index(str('SR' + str(j) + "_" + str(i)))   ##iterating column wise on the SxR matrix
            booleansumcolumn.append(Var_S_R[e])
            linkflow_column.append(l_s_r[j][i] * Var_S_R[e])
        Problem += lpSum(booleansumcolumn) >= Bool_Var[i]               ##1. OR gate for constraining number of relays 
        Problem += lpSum(linkflow_column) <= (B_Max)                    ##controlling number of sensors per relay by constraining the total data rate received by an individual relay
        for c in booleansumcolumn:
            Problem += c <= Bool_Var[i]                                 ##2. OR gate for constraining number of relays 
        booleansumcolumn=[]                                             ##emptying the columns to fill the variables of the next column
        linkflow_column=[]

    k = 0
    while k < len(Var_R_R):
        for i in range(len(e_r_r)):
            for j in range(len(e_r_r[i])):
                objective.append(e_r_r[i][j] * Var_R_R[k])
                conn_RR.append(n_r_r[i][j] * Var_R_R[k])
                k += 1   
            Problem += lpSum(conn_RR) >= 0
            conn_RR = []

    Var_alias_RR = []
    for i in Var_R_R:
        Var_alias_RR.append(str(i))
    booleansumcolumn=[]                                                 ##holds the variables for each column at a time
    
    for i in range(len(relayList)):
        for j in range(len(relayList)):
            e = Var_alias_RR.index(str('RR' + str(j) + "_" + str(i)))   ##iterating column wise on the RxR matrix
            booleansumcolumn.append(Var_R_R[e])

        Problem += lpSum(booleansumcolumn) >= Bool_Var[i]               ##1. OR gate for constraining number of relays 
       
        for c in booleansumcolumn:
            Problem += c <= Bool_Var[i]                                 ##2. OR gate for constraining number of relays 
        booleansumcolumn=[]                                             ##emptying the columns to fill the variables of the next column

    for i in range(len(relayList)):                                     ##making sure that the relay x relay matrix is symmetric
        for j in range(len(relayList)):
            if i != j:
                e = Var_alias_RR.index(str('RR' + str(j) + "_" + str(i)))   
                f = Var_alias_RR.index(str('RR' + str(i) + "_" + str(j)))
                Problem += Var_R_R[e] == Var_R_R[f]
            elif i == j:                                                 ##controversial condition
                f = Var_alias_RR.index(str('RR' + str(i) + "_" + str(j)))
                Problem += Var_R_R[f] == 0
                
            

    Problem += lpSum(objective)                                         ##adding the objective energy linear combination to minimize
    Problem += lpSum(Bool_Var) <= relayConstraint                       ##number of relays constraint finally

    ###print(Problem)
    t1=time.clock()
    Problem.solve(solvers.PULP_CBC_CMD(fracGap=0.0000000000001))        ##solving the problem
    t2=time.clock()

    ##this part (below) is just to print and understand how the code works
    DeployedRelays_SR = []
    DeployedRelays_RR = []

    for v in Problem.variables():
        ###print(v.name, "=", v.varValue)
        if v.varValue == 1.0 and v.name in Var_alias_SR:
            DeployedRelays_SR.append(v.name)
        elif v.varValue == 1.0 and v.name in Var_alias_RR:
            DeployedRelays_RR.append(v.name)
        

    Fin_Conn_S_R = [[0 for i in range(len(relayList))] for j in range(len(sensorList))] ##the output matrix produced by the LP solver
    Fin_Conn_R_R = [[0 for i in range(len(relayList))] for j in range(len(relayList))]
    
    b = 0                ##used to keep track of the index number of the string
    for i in DeployedRelays_SR:
        for j in i:
            if j !="_":
                b += 1
            else:
               break
            
        s = int(i[2:b])  ##stores the indexes of SxR matrix that are 1(high)
        r = int(i[b+1:])
      
        Fin_Conn_S_R[s][r] = 1
        b = 0
    # print('Final Optimized Connectivity Matrix of Sensor x Relay layer: ', '\n')
    # for i in Fin_Conn_S_R:
    #    print(i)
       

    b = 0                ##used to keep track of the index number of the string
    for i in DeployedRelays_RR:
        for j in i:
            if j !="_":
                b += 1
            else:
               break
            
        s = int(i[2:b])  ##stores the indexes of RxR matrix that are 1(high)
        r = int(i[b+1:])
      
        Fin_Conn_R_R[s][r] = 1
        b = 0
    ##print('Final Optimized Connectivity Matrix of Relay x Relay layer: ', '\n')
    ##for i in Fin_Conn_R_R:
    ##    print(i)
        
    v = []    
    connection={}    
    for i in range(len(Fin_Conn_S_R)):
        for j in range(len(Fin_Conn_S_R[i])):
            if Fin_Conn_S_R[i][j] == 1 and j not in v:
                connection['Relay' + str(j)] = ['Sensor' + str(i)]
                v.append(j)
            elif Fin_Conn_S_R[i][j] == 0:
                pass
            else:
                connection['Relay' + str(j)].append('Sensor' + str(i))

    ##this part (above) is just to print and understand how the code works
                
    print("Optimum Relays Used: ")
    print(sorted(connection),"\n")
    print("The Relay-Sensor dictionary: \n")
    print(connection, "\n")
    print ("Status:", LpStatus[Problem.status])
    print("Number of Candidate Location for Relays: ", len(relayList))
    print("Number of Sensors in the Network: ", len(sensorList))
    print('Number Of Relays Deployed on the Candidate Locations: ', len(connection))
    print("Communication Radius of the Each Node: ", R)
    print("Number of Base stations: ", 1)
    print("Time Taken to Calculate energy: ", t2-t1)
    
    gamma = value(Problem.objective) ##total optimum energy without steiner problem
#    print("Energy of the network before steiner node deployment: ", value(Problem.objective))
    

    ##now checking if the assumption was correct or do we need to add more relay nodes for a single path? (steiner tree problem)
    def relayNumtoInt(string):
        i = 0
        while not string[i].isdigit():
            i += 1
        return int(string[i:])-1
            

    connection_list = sorted(connection)
    terminal_nodes = ([relayNumtoInt(connection_list[i]) for i in range(len(connection_list))])
    diction = {i: [j for j, adjacent in enumerate(row) if adjacent] for i, row in enumerate(n_r_r)}
    G = nx.Graph(diction)
    x = steiner_tree(G, terminal_nodes)  ##returns all the nodes that form a tree

    if len(x.nodes) == len(terminal_nodes):
        print('No stiener nodes required.')
    else:
        ##if additional nodes are deployed we add their energies too
        B_R = 200.0
        ##taking approximately largest distance
        dist = R
        E_radio_R = 100.0 * (10**(-9))
        Transmit_amplifier = 100.0 * (10**(-12))
        energy_relay = (B_R * E_radio_R) + B_R * (E_radio_R + Transmit_amplifier * (dist ** 2))
        for i in range(len((set(x.nodes) - set(terminal_nodes)))):
            gamma += (energy_relay) ##adding relay energy of steiner nodes
        ##print('Additional steiner nodes deployed ', (set(x.nodes) - set(terminal_nodes)))

    a = 0
    for i in connection:   ##calculating maximum sensor per relay
        a = max(a,len(connection[i]))

    ## creating a separate Relay-Relay matrix for additional relays deployed by Steiner node's algorithm
    steiner_R_R = [[0 for j in range(len(Fin_Conn_R_R))] for i in range(len(Fin_Conn_R_R[0]))]
    steiner_dict = nx.to_dict_of_dicts(x)
    for relay_ in steiner_dict.keys():
        neighbors_ = steiner_dict[relay_].keys()
        print(relay_, steiner_dict[relay_].keys())
        
        ##traverse through all neighbors:
        for i in neighbors_:
            ##add these relay connections to steiner_R_R matrix if only its not present in Fin_Conn_R_R
            if(Fin_Conn_R_R[relay_][i]==0 and steiner_R_R[i][relay_]!=1):
                steiner_R_R[relay_][i]=1

    
##    print("Maximum Sensors per Relay in the Network: ", a)            
##    print("Maximum Sensor per Relay constraint of the Network: " , S_Per_R)
##    print("Maximum Relay constraint of the Network: ", relayConstraint)
##    print("Total Energy of the network after steiner nodes: ", gamma)
##    print("Total Number of relays deployed after steiner nodes: ", len(x.nodes))
    def Energy_s(SN,RN): 
        ### creates a matrix for sensor energy only
        bandwidth = 100.0 
        Energy_Sensor = [[[0] for i in range(len(RN))] for j in range(len(SN))]
        E_radio_S = 50.0 * (10 ** (-9))
        # E_radio_R = 100.0 * (10 ** (-9))
        Transmit_amplifier = 100.0 * (10 ** (-12))
        for i in range(len(SN)):
            for j in range(len(RN)):
                dist = distance(abs(SN[i][0] - RN[j][0]), abs(SN[i][1] - RN[j][1]))
                energy_sensor_tx = float(bandwidth * (E_radio_S + (Transmit_amplifier * (dist **2))))  ##energy used when sensor transmits data 
                total_energy = energy_sensor_tx
                Energy_Sensor[i][j] = total_energy                                             ##Only 1/4 sensors will be active per time unit for every relay
        return Energy_Sensor

    e_s = Energy_s(sensorList, relayList)

    #creates a matrix of relay energy used in 1 round
    def Energy_r(RN):
        bandwidth_r = 200.0
        bandwidth_s = 100.0
        Energy_Relay_Relay = [[(0, 0) for i in range(len(RN))] for j in range(len(RN))]

        E_radio_R = 100.0 * (10 ** (-9))
        Transmit_amplifier = 100.0 * (10 ** (-12))
        E_aggregation = 0.00001 ##aggregation energy
        for i in range(len(RN)):
            for j in range(len(RN)):
                dist = distance(abs(RN[i][0] - RN[j][0]), abs(RN[i][1] - RN[j][1]))
                energy_relay_tx = float(bandwidth_r * (E_radio_R + (Transmit_amplifier * (dist**2))))  ##energy used when relay transmits data
                energy_relay_rx = float(bandwidth_r * E_radio_R)                                       ##energy used when relay receives data 
                energy_relay_rx_s = float(bandwidth_s* E_radio_R)
                total_energy = (energy_relay_tx + energy_relay_rx)
                Energy_Relay_Relay[i][j] = (total_energy, energy_relay_rx_s, E_aggregation)
                
        return Energy_Relay_Relay
    
    e_r = Energy_r(relayList)

    def init_Energy_r():
        ### creates a matrix for the simulation to use (stores the energy of relays)
        init_e_r = 0.005;   
        nw_e_r = [[0 for j in range(len(relayList))] for i in range(len(relayList))]
        for i in range(len(relayList)):
            for j in range(len(relayList)):
                if(Fin_Conn_R_R[i][j]==1 or steiner_R_R[i][j]==1):
                    nw_e_r[i][j]=init_e_r                                                  
        
        return nw_e_r

    nw_e_r = init_Energy_r()

    def init_Energy_s():
        init_e_s = 0.0005
        nw_e_s = [[0 for j in range(len(relayList))] for i in range(len(sensorList))]
        for i in range(len(sensorList)):
            for j in range(len(relayList)):
                if(Fin_Conn_S_R[i][j]==1):
                    nw_e_s[i][j]=init_e_s
        
        return nw_e_s

    nw_e_s = init_Energy_s()

    #returns a matrix which contains the states of all the deployed sensors in the network 
    def state_matrix(SN, RN, Fin_Conn_S_R):
    
        state_s = [[0 for j in range(len(RN))] for i in range(len(SN))]

        for i in range(len(SN)):
            for j in range(len(RN)):
                if(Fin_Conn_S_R[i][j]==1):
                    state_s[i][j] = 1
        return state_s

    state_s = state_matrix(sensorList, relayList, Fin_Conn_S_R)

    def init_s_counter(state_s, Fin_conn_S_R):
        #count all active sensors connected to a relay
        #counter variable: contains a list of the format: [counter, starting_index_of1/4th_sensors,index_of_sensor1, index_of_sensor2,...]
        s_counter = [[0, 2] for i in range(len(state_s[0]))]

        for i in range(len(state_s[0])):
            for j in range(len(state_s)):
                if(Fin_conn_S_R[j][i]==1):
                    s_counter[i][0]+=1
                    s_counter[i].append(j) #stores the row index of the sensor
        

        
        # ##divide the counted values into 4 parts
        # for i in range(len(s_counter)):
        #     if((s_counter[i][0]//4 )!=0): ##if s_counter divided by 4 gives 0, then take all sensonrs in one round only
        #         s_counter[i][0] = s_counter[i][0]//4
        return s_counter
    
    s_counter = init_s_counter(state_s, Fin_Conn_S_R)
    # print("s_counter: ")
    # for i in s_counter:
    #     print(i)

    def init_sensor_PH_value(SN):
        ###initializes the PH values for all sensor nodes
        PH_list_s = [7.2 for j in range(len(SN))]
        return PH_list_s
    
    PH_list_sensors = init_sensor_PH_value(sensorList)

    def Master_PH_checker(PH_list_sensors, Fin_Conn_S_R, state_s):
        ###checks the pH of all sensor nodes. 
        Clusters_with_oil_spills = [] ##contains all relay indexes where oil spill was detected
        for i in range(len(Fin_Conn_S_R[0])):
            aggregated_PH = 0
            no_of_sensors = 0
            for j in range(len(Fin_Conn_S_R)):
                pH_value = PH_list_sensors[j] ##ph of the sensor
                # print(pH_value)
                if (Fin_Conn_S_R[j][i]==1 and state_s[j][i] ==1): ##check if the pH is not normal and the relay is connected to the sensor and the sensor is turned on
                    aggregated_PH+= pH_value
                    no_of_sensors+=1
                    
            if(no_of_sensors!=0):
                
                aggregated_PH = float(aggregated_PH)/float(no_of_sensors)
                if(aggregated_PH>8.5):##if aggregated_PH is greater than 8.5 than an oil spill is detected
                    Clusters_with_oil_spills.append(i)
            
        return Clusters_with_oil_spills

    def PH_checker(PH_list_sensors, Fin_Conn_S_R, state_s, cluster_head):
        aggregated_PH = 0
        no_of_sensors = 0
        for i in range(len(PH_list_sensors)):
            if(Fin_Conn_S_R[i][cluster_head]==1 and state_s[i][cluster_head]==1): ##check if sensor and relay are connected and switched on and pH level is exceded
                aggregated_PH+=PH_list_sensors[i] ##if spill detected
        if(no_of_sensors!=0):
            aggregated_PH = aggregated_PH/no_of_sensors
            if(aggregated_PH>8.5):
                return True
            else:
                return False
        else:
            return False

    def add_neighbour(Cluster_head, Fin_Conn_R_R, checked): ##cluster_head contains the index of that relay which cluster detects oil leaks
        ##checks and returns neigbouring clusters to a cluster
        neighbor_relays = []
        for i in range(len(Fin_Conn_R_R)): #checking for the relays connected to the "Cluster_head" relay
            possible_neighbor = Fin_Conn_R_R[i][Cluster_head]
            if(possible_neighbor==1 and (possible_neighbor not in checked)): ##if there is a 1, then the relays are neighbors!
                neighbor_relays.append(i)
        return neighbor_relays
    

    def oil_simulator(Fin_Conn_S_R, sensorList, PH_list_sensors):
        xrange = 50
        yrange = 50 
        oil_spill_PH = 10 ## pH value of oil
        for i in range(len(sensorList)):
            # print(sensorList[i])
            if (sensorList[i][0] <=xrange and sensorList[i][1] <=yrange):
                PH_list_sensors[i] = oil_spill_PH


    def reset_oil(sensorList, PH_list_sensors):
        normal_PH = 7.5
        for i in range(len(sensorList)):
            PH_list_sensors[i] = normal_PH
        

        




    

    def SRP_toggler(state_s,  s_counter, PH_list_sensors, Fin_Conn_S_R, Fin_Conn_R_R, round, srpfile):
        
        
        ### The following code is divided into 2 blocks.
        #Block 1 is for Normal operation of SRP and Block 2 is behaviour after oil detection 
        
        ### Block 1:
        # print("check for oil leaks:")
        oil_affected_relays = Master_PH_checker(PH_list_sensors, Fin_Conn_S_R, state_s)
        # print(oil_affected_relays)
        
        if (oil_affected_relays==[]):##if no oil spill detected then run normal operation
            # print("No leaks detected.\n proceed to normal protocol")
            ##toggle 1/4 th sensors:
            #this loop iterates over a cluster of relay. j gives us the relay index
            for j in range(len(s_counter)):
                counter_temp = s_counter[j][0] ##number of sensors in the cluster
                starting_sensor = s_counter[j][1] ## stores which sensor to start with (does not stores the actual sensors index)
                
                counter_1_4_th = s_counter[j][0]//4 ##divides the cluster into four parts

                ##check if all 4 batches have been activated. If so then reset the starting sensor
                if(starting_sensor >= len(s_counter[j])-1 and counter_temp!=0):
                    srpfile.write("For "+ str(j) +"'th relay. resetting cycle\n")
                    # print("For", j, "'th relay. resetting cycle")
                    s_counter[j][1] = 2  ##reset the starting sensor i.e. start from the 1st sensor (i.e. with index 2 )
                    starting_sensor = s_counter[j][1]
                ##iterating over the list to turn off all sensors and turn on the 1/4th batch 
                counting = 0  
                starting_detected = False
                for k in range(counter_temp+2):
                    ##skip the first two indexes as they contain the count and the starting point of the sensor batch
                    if(k >1):
                        
                        #check if the index is the starting point
                        if(s_counter[j][k]==s_counter[j][starting_sensor]):
                            starting_detected = True
                        #check if the starting sensor is detected and 1/4th of the sensors are not activated. turn on the sensor 
                        if(counter_1_4_th!=0):  ##if counter_1_4_th is zero then the cluster remains on and no switching is required
                            if(starting_detected and counting < counter_1_4_th): 
                                state_s[s_counter[j][k]][j] = 1
                                counting+=1
                                s_counter[j][1]+=1 ##update the starting sensor 
                            #else turn off the sensor
                            else:
                                state_s[s_counter[j][k]][j] = 0
                        else: ##the cluster will have all sensors turned on 
                            state_s[s_counter[j][k]][j] = 1
                            counting+=1
                ##priniting sensors conncted to a relay
                if(counter_temp!=0):
                    srpfile.write("Relay: "+ str(j) + " has "+ str(counting)+ " active sensors out of "+ str(len(s_counter[j]) -2)+ " sensors\n")
                    # print("Relay: ", j , " has ", counting, " active sensors out of ", len(s_counter[j]) -2, " sensors")
        else:
            srpfile.write("Oil Detected in Round: "+ str(round)+" "+str(len(x.nodes)-1) +"\n")
            print("Oil Detected in Round: ", round+len(x.nodes)-1)
            #turn on all sensors in the oil detected relays:
            # print("Oil leak detected. Turning on all sensors in all affected clusters")
            srpfile.write("Oil leak detected. Turning on all sensors in all affected clusters\n")
            for i in oil_affected_relays: 
                for j in range(len(state_s)):
                    if(Fin_Conn_S_R[j][i]==1): #check if the sensor is connected to the relay
                        ##turn it on:
                        state_s[j][i] = 1 
            # print("check neighboring cluster heads PH")
            # #alert its neighbors and add their neighbors to the oil_affected_relays list: 
            # oil_affected_temp = oil_affected_relays[:]
            # while(oil_affected_temp!=[]):
            #     print("in loop")
            #     check_cluster = oil_affected_temp.pop(0) ##cluster to be checked for spill
            #     if(PH_checker(PH_list_sensors, Fin_Conn_S_R, state_s, check_cluster)): ##if true then check neighbors aswell 
            #         new_clusters = add_neighbour(check_cluster, Fin_Conn_R_R, oil_affected_relays)
            #         oil_affected_relays.append(new_clusters) ##add to the master relay list
            #         oil_affected_relays.append(new_clusters) ##add to the temp relay list for stack implimentation
                    


    
    def simu_network(nw_e_s, nw_e_r, state_s):
        #intializing the rounds
        file = open("percentage_network.txt", "w")
        srpfile = open("srp_output.txt", "w")
        file.write("sensors: " +str(sen) +"\n")
        file.write("Relays: " + str(len(connection)) +"\n")
        file.write(str(connection))
        file.write("\n")
        total_energy = 0
        

        round = 0
        dead_s = 0  
        dead_r = 0
        
        #running the simulation for n rounds
        while(True):
            consumed_round_energy = 0
            #initializing dead sensors 
            # print("ROUND: ", round)
            file.write("ROUND: " + str(round)+ "\n")
            srpfile.write("ROUND: " + str(round)+ "\n")
            # if(dead_r>0):
            #     # print("Ah ha!")
            #     break
            #updating all sensors
            ##this loop iterates over all the relays
            # print("sending/receiving data")
            for i in range(len(Fin_Conn_S_R[0])):
                ##this loop iterates over all sensors
                # print("sending data from sensor")
                for j in range(len(Fin_Conn_S_R)):
                    #checking if the sensor is deployed as well as not asleep as well as not dead
                    
                    if(Fin_Conn_S_R[j][i]== 1 and state_s[j][i] == 1): 
                        #check if a node died
                        if(nw_e_s[j][i]- e_s[j][i] <=0):  
                            # print("sensor ", j, " died")
                            # strings = "sensor " + str(j) + " died" +"\n"
                            # file.write(strings)
                            Fin_Conn_S_R[j][i]=0 ##sensor dies
                            dead_s+=1
                        else: 
                            ##transmit data to the relay 
                            # print("transmission energy: ", e_s[j][i], " residual energy of sensor: ", nw_e_s[j][i]-e_s[j][i])
                            # strings = "transmission energy: " + str(e_s[j][i]) +" residual energy of sensor: " +str(nw_e_s[j][i]-e_s[j][i]) +"\n"
                            # file.write(strings)
                            nw_e_s[j][i] -= e_s[j][i] 
                            consumed_round_energy+=e_s[j][i] 
                            
                ##relay connected to steiner relays:
                for w in range(len(steiner_R_R)):
                    if (steiner_R_R[i][w] ==1):
                        if(nw_e_r[i][w]-e_r[i][w][0] >0):
                            nw_e_r[i][w]-=e_r[i][w][0]
                            consumed_round_energy+=e_r[i][w][0]
                        else:
                            dead_r+=1
                            steiner_R_R[i][w]=0

                # print("sending/receiving data on relays")
                ##this loop iterates over all relays connected to i'th relay
                for k in range(len(Fin_Conn_R_R)):
                    ##check if the two relays are connected. If yes then exchange data
                    # print("checking relay connection", Fin_Conn_R_R[k][i])
                    if(Fin_Conn_R_R[k][i] == 1):
                        #check if node is dead:
                        # print("connection relay found")
                        if (nw_e_r[k][i] - e_r[k][i][0] <= 0):
                            Fin_Conn_R_R[k][i]=0
                            for m in range(len(Fin_Conn_S_R)): ##turn off all sensors connected to the relays
                                if(Fin_Conn_S_R[m][i]!=0):
                                    Fin_Conn_S_R[m][i] = 0
                                    dead_s+=1
                            # print("A relay with energy: ", nw_e_r[k][i], "died")
                            # strings = "A relay with energy: "+str(nw_e_r[k][i]) +"died" + "\n"
                            # file.write(strings)
                            dead_r+=1
                        else:
                            
                            # print("transmission energy: ", e_r[k][i], " residual energy of relay: ", nw_e_s[k][i]-e_s[k][i])
                            # strings = "transmission energy: " +str(e_r[k][i])+ " residual energy of relay: "+ str(nw_e_s[k][i]-e_s[k][i]) +"\n"
                            # file.write(strings)
                            nw_e_r[k][i] -= e_r[k][i][0]
                            consumed_round_energy+=e_r[k][i][0]
                            #check for connected sensors:
                            #this loop checks for connected sensors to a relay
                            for l in range(len(Fin_Conn_S_R)): #gives index of sensor
                                if(Fin_Conn_S_R[l][i]==1 and state_s[l][i] ==1):
                                    if((nw_e_r[k][i] - e_r[k][i][1]) > 0):
                                        # print("receival energy: ", e_r[k][i], " residual energy of relay: ", nw_e_r[k][i]-e_r[k][i][1])
                                        # strings = "receival energy: "+ str(e_r[k][i])+ " residual energy of relay: "+ str(nw_e_r[k][i]-e_r[k][i][1]) +"\n"
                                        # file.write(strings)
                                        nw_e_r[k][i]-=  e_r[k][i][1]            ##receive data from sensor
                                        consumed_round_energy+=e_r[k][i][1]
                                    else:
                                        Fin_Conn_R_R[k][i]=0
                                        dead_r+=1
                                        for m in range(len(Fin_Conn_S_R)):##turn off all sensors connected to the relays
                                            if(Fin_Conn_S_R[m][i]!=0):
                                                Fin_Conn_S_R[m][i] = 0
                                                dead_s+=1
                                        

                ##This loop aggregates data
                for m in range(len(Fin_Conn_R_R)):
                    if(Fin_Conn_R_R[m][i]==1):
                        nw_e_r[m][i]-=e_r[m][i][2]
                        consumed_round_energy+=e_r[m][i][2]


            dead_s_pc = (dead_s/sen)*100
            # print("dead relays: ", dead_r)
            strings = "dead relays: " + str(dead_r) +"\n"
            file.write(strings)
            dead_r_pc = (dead_r/len(x.nodes))*100
            dead_nw_pc =( dead_s_pc + dead_r_pc)/2
            # print("Dead Network pc: ", dead_nw_pc, " %")
            strings = "Dead Network pc: " + str(dead_nw_pc) + " % \n"
            file.write(strings)
            # print("Dead Sensor pc: ", dead_s_pc, " %")
            strings = "Dead Sensor pc: "+ str(dead_s_pc) + " % \n"
            file.write(strings)
            # print("Dead relays pc: ", dead_r_pc, " %")
            strings = "Dead relays pc: "+ str(dead_r_pc)+ " % \n"
            file.write(strings)

            ##at round 100 spill oil:
            if(round==30):
                oil_simulator(Fin_Conn_S_R, sensorList, PH_list_sensors)

                print("Spilling oil")
            if(round== 30+len(x.nodes) - 1):
                reset_oil(sensorList, PH_list_sensors)

            if( dead_s_pc > 90 or dead_r_pc >90):
                break 
            # print("Energy matrix Relay:")
            # for x in e_r:
            #     print(x)
            # print("Energy matrix Sensor:")
            # for x in e_s:
            #     print(x)

            ##toggles the state of the sensors (SRP implimented here)
            SRP_toggler(state_s, s_counter, PH_list_sensors, Fin_Conn_S_R, Fin_Conn_R_R, round, srpfile)
            #output energy used in the round:
            total_energy+=consumed_round_energy
            # print("Energy used in round:", consumed_round_energy)
            srpfile.write("Energy used in round: " + str(consumed_round_energy) + "\n")
            ##update round
            round+=1
            
        file.write("total rounds: "+ str(round)+"\n")
        # print("total rounds:", round)

        file.write("total Energy used: "+ str(total_energy) +"\n")
        # print("total Energy used:", total_energy)

        file.close()
        srpfile.close()
        return nw_e_s

    network_energy_s = simu_network(nw_e_s, nw_e_r, state_s)
    file = open("energy matrix.txt", "a")
    file.write("sensor residual matrix"+"\n")
    for i in network_energy_s:
        file.write(str(i))
        file.write("\n")
        # print(i) 
    file.write("Relay residual matrix"+"\n")
    for i in nw_e_r:
        file.write(str(i)+"\n")

    file.write("Total energy used: "+ str(gamma) +"\n")
    file.close()
    print(s_counter)



    return len(x.nodes), gamma



k =100
radius = 30
relay, energy = Optimize(radius, k)
print('Radius =', radius,  ', Sensors = ', k)
print('Relays used:', relay)
print('Energy used: ', energy)

# ##running 100 simulations per configuration (Radius = 15)
# relays_15 = []
# energies_15 = []
# k = 100
# while k <= 1000:
#     for i in range(100):
#         relay, energy  = Optimize(15, k)
#         relays_15.append(relay)
#         energies_15.append(energy)
#     print('Radius = 15, Sensors = ', k)
#     print('Relays used:', sum(relays_15)/len(relays_15))
#     print('Energy used: ', sum(energies_15)/len(energies_15))
#     relays_15 = []
#     energies_15 = []
#     k += 100


# ##running 100 simulations per configuration (Radius = 30)
# k = 100
# relays_30 = []
# energies_30 = []
# while k <= 1000:
#     for i in range(100):
#         relay, energy  = Optimize(30, k)
#         relays_30.append(relay)
#         energies_30.append(energy)
#     print('Radius = 30, Sensors = ', k)
#     print('Relays used:', sum(relays_30)/len(relays_30))
#     print('Energy used: ', sum(energies_30)/len(energies_30))
#     relays_30 = []
#     energies_30 = []
#     k += 100
    


    

