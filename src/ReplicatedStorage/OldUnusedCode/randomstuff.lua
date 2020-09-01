          
          --Old constraint forwards code

          -- The axis of constraint is relative to the initial joint placement
            local yAxis = limbConstraintTable[i][1]
            local centerAxis = limbConstraintTable[i][2]
            local angles = limbConstraintTable[i][3]
            pointTo = ConicalConstraint(pointTo, limbLengthTable[i], yAxis,
                                        centerAxis, angles)
