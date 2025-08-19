from polygon import Point3D


def parse_g_code(code:str,d_nos = 0.9, d_syr = 12.1, dz = 0.4)->"list[Point3D]":
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
    ext = 0
    ext_prev = 0
    f = 0
    e_f = 0
    e_d = 0

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

                    if coord[0]=="V":
                        e_f = float(coord[1:])

                    if coord[0]=="F":
                        f = float(coord[1:])
                    if coord[0]=="D":
                        e_d = float(coord[1:])
                    if coord[0]=="E":
                        ext_prev = ext
                        ext = float(coord[1:])  
                        if ext_prev>ext:
                            e_d = -1  
                        e_f = vel_div_ard(f,d_nos, d_syr, dz)
            
            if coords[0][0]=="G":
                r = f    
                g = e_f
                b = e_d
                if com_num==0:
                    p3ds.append(Point3D(x,y,z,False,r,g,b))
                if com_num==1:
                    p3ds.append(Point3D(x,y,z,True,r,g,b))
    return p3ds

def vel_div_ard(vel_nos:float,d_nos:float,d_syr:float,dz:float):
    vel = (vel_nos*d_nos*dz)/(d_syr*d_syr)
    nT = float(5000)
    p = float(1)
    rev = float(200 * 16)
    st = int((nT*p)/(vel*rev))
    return st


def parse_g_code_pulse(code:str,units:float = 1,d_nos = 0.9, d_syr = 12.1, dz = 0.4)->"list[Point3D]":
    p3ds = []
    lines = code.split("\n")
    x=0 
    y=0
    z=0

    pitch=0 
    roll=0
    yaw=0

    r =0.9
    g= 0.9
    b= 0.1
    com_num = 28
    cur_extr= 0
    ext = 0
    ext_prev = 0
    e_f =0
    e_d =0
    f =10
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
                        x = units* float(coord[1:])
                    if coord[0]=="Y":
                        y = units*float(coord[1:])
                    if coord[0]=="Z":
                        z = units*float(coord[1:])
                    
                    if coord[0]=="A":
                        roll = float(coord[1:])
                    if coord[0]=="B":
                        pitch = float(coord[1:])
                    if coord[0]=="C":
                        yaw = float(coord[1:])

                    if coord[0]=="V":
                        e_f = float(coord[1:])

                    if coord[0]=="F":
                        f = float(coord[1:])
                    if coord[0]=="D":
                        e_d = float(coord[1:])
                    if coord[0]=="E":
                        ext_prev = ext
                        ext = float(coord[1:])  
                        if ext_prev>ext:
                            e_d = -1  
                        e_f = vel_div_ard(f,d_nos, d_syr, dz) 
            
            if coords[0][0]=="G":
                r = f 
                g = e_f
                b = e_d   
                #pitch = 0
                #roll*=0.5
                if com_num==0:
                    p3ds.append(Point3D(x,y,z,False,0,g,b,_pitch=pitch,_roll=roll,_yaw=yaw))
                if com_num==1:
                    p3ds.append(Point3D(x,y,z,True,0,g,b,_pitch=pitch,_roll=roll,_yaw=yaw))

    return p3ds

def parse_g_code_conv_cnc_to_def(code:str)->str:
    p3ds = []
    lines = code.split("\n")
    x=0 
    y=0
    z=0
    for line in lines:        
        coords = line.split()
        if len(coords)>0 and (('X' in line) or ('Y' in line) or ('Z' in line)):
            for coord in coords:
                if coord[0]=="X":
                    x = float(coord[1:])
                if coord[0]=="Y":
                    y = float(coord[1:])
                if coord[0]=="Z":
                    z = float(coord[1:])
            p3ds.append(Point3D(x,y,z))
            
            #if li

    return gen_xyz_g_code(p3ds)

def gen_xyz_g_code(ps:list[Point3D], scale:float = 1.0):
    code= "" 
    for p in ps:
        code+="G1 X"+str(round(scale*p.x,2))+" Y"+str(round(scale*p.y,2))+" Z"+str(round(scale*p.z,2))+'\n'

    return code

#def parse_val(val:str)->float:


                    

            