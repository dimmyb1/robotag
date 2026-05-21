    class Cell():
        def __init__(self, xx, yy):
            self.x = xx
            self.y = yy
    
        def __eq__(self, other):
            if isinstance(other, Cell):
                return self.x == other.x and self.y == other.y
            return False

    def getNeighbourEdgesOf(fromE):
        A = ["Pab1", "Pab2", "Paf1", "Pae1"]
        B = ["Pab1", "Pab2", "Pbd1", "Pbc1"]
        C = ["Pcd1", "Pcd2", "Pch1", "Pbc1"]
        D = ["Pcd1", "Pcd2", "Pdf1", "Pbd1"]
        E = ["Peg1", "Peg2", "Pae1", "Pef1"]
        F = ["Pef1", "Paf1", "Pfg1", "Pdf1"]
        G = ["Peg1", "Peg2", "Pfg1", "Pgh1"]
        H = ["Pch1", "Phh1", "Pgh1"]
        #this is not a mistake. H only has 3 unique edge transitions.
            
        toReturn = []

        if fromE in A:
            A.remove(fromE)
            if(toReturn):
                toReturn.extend(A)
                return toReturn
            else:
                toReturn = A

        if fromE in B:
            B.remove(fromE)
            if(toReturn):
                toReturn.extend(B)
                return toReturn
            else:
                toReturn = B
        if fromE in C:
            C.remove(fromE)
            if(toReturn):
                toReturn.extend(C)
                return toReturn
            else:
                toReturn = C
        if fromE in D:
            D.remove(fromE)
            if(toReturn):
                toReturn.extend(D)
                return toReturn
            else:
                toReturn = D
        if fromE in E:
            E.remove(fromE)
            if(toReturn):
                toReturn.extend(E)
                return toReturn
            else:
                toReturn = E
        if fromE in F:
            F.remove(fromE)
            if(toReturn):
                toReturn.extend(F)
                return toReturn
            else:
                toReturn = F
        if fromE in G:
            G.remove(fromE)
            if(toReturn):
                toReturn.extend(G)
                return toReturn
            else:
                toReturn = G
        if fromE in H:
            H.remove(fromE)
            if(toReturn):
                toReturn.extend(H)
                return toReturn
            else:
                toReturn = H

        return toReturn
      
    #Graph Functions
    def calculateProbabilities(Po, x, y, cells):
        #let's say radar stores the closest ultrasonic ping in euclidean metric in ultrasonic_distance
        #we get two readings: first ping entering reading (entry_angle)
        # second ping exiting reading (exit_angle)
        # the order doesnt make a difference, main thing is that we have the angle, wwe'll just take min or max of the two values.
        #and servo was the angle at which we go tthe reading, +- the known margin of error
        #and that BOXMEAS is the l / w of the boxes in the grid in euclidean metric
        BOXMEAS = 0.4
        PERSISTENCE = 0.7
        CONTAMINATION = 0.3
        
        #now we have a pretty small set of cells which the opponent can be in
        #reset probabilities
        P = {
            "Pab1": 0.0,
            "Pab2": 0.0,
            "Paf1": 0.0,
            "Pae1": 0.0,
            "Pef1": 0.0,
            "Peg1": 0.0,
            "Peg2": 0.0,
            "Pgh1": 0.0,
            "Phh1": 0.0,
            "Pfg1": 0.0,
            "Pdf1": 0.0,
            "Pbd1": 0.0,
            "Pbc1": 0.0,
            "Pcd1": 0.0,
            "Pcd2": 0.0,
            "Pch1": 0.0
        }

        
        for c in cells:
            if c.x == 0:
                if c.y == 0:
                    P["Peg2"] += 0.2106
                elif c.y == 1:
                    P["Peg2"] += 0.5466
                    P["Peg1"] += 0.0108
                elif c.y == 2:
                    P["Pae1"] += 0.2005
                    P["Peg2"] += 0.3825
                    P["Peg1"] += 0.0031
                elif c.y == 3:
                    P["Pae1"] += 0.6361
                    P["Paf1"] += 0.0053
                elif c.y == 4:
                    P["Pae1"] += 0.5522
                    P["Pab2"] += 0.0433
                    P["Paf1"] += 0.0099
                elif c.y == 5:
                    P["Pab2"] += 0.0293
            elif c.x == 1:
                if c.y == 0:
                    P["Peg2"] += 0.5323
                elif c.y == 1:
                    P["Peg2"] +=0.0268
                    P["Peg1"] +=0.6065
                elif c.y == 2:
                    P["Paf1"]+=0.0249
                    P["Peg1"]+= 0.0917
                    P["Peg2"]+= 0.0322
                    P["Pef1"]+= 0.3172
                    P["Pae1"]+= 0.2603
                elif c.y == 3:
                    P["Pae1"] += 0.0841
                    P["Paf1"] += 0.5752
                elif c.y == 4:
                    P["Pab1"]+= 0.3221
                    P["Pab2"]+= 0.1905
                    P["Paf1"]+= 0.1632
                    P["Pae1"]+= 0.0297
                elif c.y == 5:
                    P["Pab2"] += 0.5855
            elif c.x == 2:
                if c.y == 0:
                    P["Peg2"] +=0.1621
                elif c.y == 1:
                    P["Pgh1"]+= 0.3086
                    P["Pfg1"]+= 0.2170
                    P["Peg1"]+= 0.0647
                    P["Peg2"]+= 0.1755
                elif c.y == 2:
                    P["Pef1"]+= 0.0791
                    P["Pfg1"]+= 0.0963
                    P["Paf1"]+= 0.2668
                    P["Pdf1"]+= 0.3226
                elif c.y == 3:
                    P["Pbd1"] += 0.3311
                    P["Paf1"] += 0.1247
                    P["Pdf1"] += 0.0016
                elif c.y == 4:
                    P["Pab1"]+= 0.0753
                    P["Pab2"]+= 0.1886
                    P["Pbc1"]+= 0.2815
                    P["Pbd1"]+= 0.1985
                elif c.y == 5:
                    P["Pab2"] += 0.2288
            elif c.x == 3:
                if c.y == 0:
                    P["Phh1"] += 0.3463
                elif c.y == 1:
                    P["Pgh1"]+= 0.1191
                    P["Pch1"]+= 0.2218
                    P["Phh1"]+= 0.4263
                elif c.y == 2:
                    P["Pdf1"] += 0.2206
                    P["Pch1"] += 0.4203
                elif c.y == 3:
                    P["Pdf1"]+= 0.3213
                    P["Pcd1"]+= 0.0416
                    P["Pcd2"]+= 0.2097
                    P["Pbd1"]+= 0.1095
                elif c.y == 4:
                    P["Pbd1"] += 0.0039
                    P["Pcd1"]+= 0.1546
                    P["Pcd2"]+= 0.3530
                    P["Pbc1"]+= 0.1164
                    P["Pch1"]+= 0.1825
                elif c.y == 5:
                    P["Pch1"] += 0.4471
            elif c.x == 4:
                if c.y == 0:
                    P["Phh1"] += 0.2595
                elif c.y == 1:
                    P["Phh1"] += 0.3974
                elif c.y == 2:
                    P["Pch1"] += 0.5882
                elif c.y == 3:
                    P["Pch1"] += 0.4595
                    P["Pcd2"] += 0.1255
                elif c.y == 4:
                    P["Pch1"] +=0.4561
                    P["Pcd2"] += 0.2891
                elif c.y == 5:
                    P["Pch1"] += 0.5771
        


        #BAYES FILTER
        #let's hop on over to Po dict for a moment
        #Po is a dictionary which is storing our historic info
        #we have a temp dict called Px:
        Px = {}
        #we will apply some blurring effect and forgetfulness to Po
        
        for k,v in Po.items():
            #reduce the weight of the old info by 1-PERSISTENCE
            Px[k] = PERSISTENCE * v

            #allow for contamination from neighbouring edges at a rate of CONTAMINATION
            #where PERSISTENCE + CONTAMINATION = 1
            neighbours = getNeighbourEdgesOf(k)

            #Neighbouring Edge Key -> nek
            for nek in neighbours:
                nekNLen = len(getNeighbourEdgesOf(nek))

                Px[k] += CONTAMINATION * ( Po[nek] / nekNLen)

        #now we replace Po with Px
        Po = Px.copy()

        totalProb = 0.0
        #ok! Po has been blurred!
        #now let us update our P values (Bayesian inference)
        for k in P:
            P[k] *= Po[k]
            P[k] = max(P[k], 0.001) #clamp to prevent collapse
            totalProb += P[k]

        #now we normalise
        for k in P:
            P[k] /= totalProb

        #P gets cleared every time this function runs anyway
        #so all we have to do is save Po as P for our next run.
        Po = P.copy()

        return Po


    Po = {
            "Pab1": 0.0625,
            "Pab2": 0.0625,
            "Paf1": 0.0625,
            "Pae1": 0.0625,
            "Pef1": 0.0625,
            "Peg1": 0.0625,
            "Peg2": 0.0625,
            "Pgh1": 0.0625,
            "Phh1": 0.0625,
            "Pfg1": 0.0625,
            "Pdf1": 0.0625,
            "Pbd1": 0.0625,
            "Pbc1": 0.0625,
            "Pcd1": 0.0625,
            "Pcd2": 0.0625,
            "Pch1": 0.0625
        }



    
    x = 3
    y = 1
    Po = calculateProbabilities(Po, x, y, [Cell(3,3), Cell(3,4)])
    print("at D:", sorted(Po.items(), key=lambda x: x[1], reverse=True))
    Po = calculateProbabilities(Po, x, y, [Cell(2,3), Cell(2,4)])
    print("at Pbd1:", sorted(Po.items(), key=lambda x: x[1], reverse=True))
    Po = calculateProbabilities(Po, x, y, [Cell(2,4)])
    print("at B:", sorted(Po.items(), key=lambda x: x[1], reverse=True))
    Po = calculateProbabilities(Po, x, y, [Cell(1,5), Cell(2,5)])
    print("at Pab2:", sorted(Po.items(), key=lambda x: x[1], reverse=True))



