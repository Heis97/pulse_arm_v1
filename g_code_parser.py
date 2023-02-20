from polygon import Point3D


def parse_g_code(code:str)->"list[Point3D]":
    p3ds = []
    lines = code.split("\n")
    x=0 
    y=0
    z=0

    #r =0.9
    g= 0.9
    b= 0.1
    com_num = 28
    cur_extr= 0
    f = 0

    for line in lines:        
        coords = line.split()
        if len(coords)>0:
            if coords[0][0]=="G":
                com_num = int(coords[0][1:])
                
                
            if coords[0][0]=="T":
                cur_extr = int(coords[0][1])
                if cur_extr == 0:
                    #r =0.9
                    g= 0.5
                    b= 0.1
            if com_num ==0 or com_num == 1:   
                if cur_extr == 1:
                    #r =0.5
                    g= 0.9
                    b= 0.1
                if cur_extr == 2:
                    #r =0.1
                    g= 0.5
                    b= 0.9
                if cur_extr == 3:
                    r =0.1
                    g= 0.5
                    b= 0.1
                if cur_extr == 4:
                    #r =0.5
                    g= 0.1
                    b= 0.1
                for coord in coords:
                    if coord[0]=="X":
                        x = float(coord[1:])
                    if coord[0]=="Y":
                        y = float(coord[1:])
                    if coord[0]=="Z" or coord[0]=="A" or coord[0]=="B":
                        z = float(coord[1:])

                    if coord[0]=="F":
                        f = float(coord[1:])
                        
            
            if coords[0][0]=="G":
                r = f    
                if com_num==0:
                    p3ds.append(Point3D(x,y,z,False,r,g,b))
                if com_num==1:
                    p3ds.append(Point3D(x,y,z,True,r,g,b))
    return p3ds

def parse_g_code_pulse(code:str)->"list[Point3D]":
    p3ds = []
    lines = code.split("\n")
    x=0 
    y=0
    z=0

    r =0.9
    g= 0.9
    b= 0.1
    com_num = 28
    cur_extr= 0

    for line in lines:        
        coords = line.split()
        if len(coords)>0:
            if coords[0][0]=="G":
                com_num = int(coords[0][1:])
                
                
            if coords[0][0]=="T":
                cur_extr = int(coords[0][1])
                if cur_extr == 0:
                    r =0.9
                    g= 0.5
                    b= 0.1
            if com_num ==0 or com_num == 1:   
                if cur_extr == 1:
                    r =0.5
                    g= 0.9
                    b= 0.1
                if cur_extr == 2:
                    r =0.1
                    g= 0.5
                    b= 0.9
                if cur_extr == 3:
                    r =0.1
                    g= 0.5
                    b= 0.1
                if cur_extr == 4:
                    r =0.5
                    g= 0.1
                    b= 0.1
                for coord in coords:
                    if coord[0]=="X":
                        x = float(coord[1:])
                    if coord[0]=="Y":
                        y = float(coord[1:])
                    if coord[0]=="Z":
                        z = float(coord[1:])
                    if coord[0]=="A":
                        r = float(coord[1:])
                    if coord[0]=="B":
                        g = float(coord[1:])
                    if coord[0]=="C":
                        b = float(coord[1:])
            
            if coords[0][0]=="G":    
                if com_num==0:
                    p3ds.append(Point3D(x,y,z,False,r,g,b))
                if com_num==1:
                    p3ds.append(Point3D(x,y,z,True,r,g,b))
    return p3ds




#def parse_val(val:str)->float:


                    

            