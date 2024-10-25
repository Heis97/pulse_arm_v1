import sys

pi = 3.1415926535

def to_rad(a):
    return float(a)*pi/180

def ptp(x,y,z,a,b,c):
    ret = "frame = new Frame("+str(x)+","+str(y)+","+str(z)+","+str(to_rad(a))+","+str(to_rad(b))+","+str(to_rad(c))+");\n"
    ret += "iiwa.move(ptp(frame).setJointVelocityRel(0.1).setJointAccelerationRel(0.1));\n"
    return ret

def lin(x,y,z,a,b,c,vel,acs):
    ret = "frame = new Frame("+str(x)+","+str(y)+","+str(z)+","+str(to_rad(a))+","+str(to_rad(b))+","+str(to_rad(c))+");\n"
    ret += "iiwa.move(lin(frame).setCartVelocity("+str(vel)+").setCartAcceleration("+str(acs)+").setOrientationVelocity("+str(vel)+");\n"
    return ret

def grip_on():
    ret = "openGripper();\n"
    return ret


def grip_off():
    ret = "closeGripper();\n"
    return ret


def delay(t):
    ret = "Thread.sleep("+str(t*1000)+");\n"
    return ret


f = open("cod1.txt")
prog = ""
for line in f:
    line_cur = line.lower()
    line_cur = line_cur.replace("  "," ")
    line_cur = line_cur.replace("  "," ")
    line_cur = line_cur.replace("  "," ")
    line_cur = line_cur.replace("  "," ")
    line_cur = line_cur.replace("\n","")
    if "on" in line_cur:
        prog += grip_on()
    if "off" in line_cur:
        prog += grip_off()
    if "ptp" in line_cur:
        res1 = line_cur.split(")")
        res4 = res1[1].replace("a","")
        res4 = res4.replace("v","")
    

        res2 = res1[0].split("(")
        res3 = res2[1].split(" ")
        x = float(res3[0].strip())
        y = float(res3[1].strip())
        z = float(res3[2].strip())
        a = float(res3[3].strip())
        b = float(res3[4].strip())
        c = float(res3[5].strip())
        prog+=ptp(x,y,z,a,b,c)
    if "lin" in line_cur:
        res1 = line_cur.split(")")
        
        res4 = res1[1].replace("a","")
        res4 = res4.replace("v","")
        res4 = res4.strip()
        res5 = res4.split(" ")
        
        vel = float(res5[0].strip())
        acs = float(res5[1].strip())

        res2 = res1[0].split("(")
        res3 = res2[1].split(" ")
        x = float(res3[0].strip())
        y = float(res3[1].strip())
        z = float(res3[2].strip())
        a = float(res3[3].strip())
        b = float(res3[4].strip())
        c = float(res3[5].strip())
        prog+=lin(x,y,z,a,b,c,vel,acs)

print(prog)

