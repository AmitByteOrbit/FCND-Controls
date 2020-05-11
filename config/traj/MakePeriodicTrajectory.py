import math
import numpy as np

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def fmt(value):
	return "%.3f" % value

period = [4, 2, 4]
radius = 1.5
timestep = 0.02
maxtime = max(period)*3
timemult = [1, 1, 1]
phase=[0,0,0]
amp = [1,0.4,.5]
center = [0, 0, -2]

with open('FigureEight.txt', 'w') as the_file:
	t=0
	px = 0
	py = 0
	pz = 0
	while t <= maxtime:
		x = math.sin(t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0]
		y = math.sin(t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1]
		z = math.sin(t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2]
		the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z))
		vx = 0
		vy = 0
		vz = 0
		######## BEGIN STUDENT CODE

		yaw, pitch, roll = 0.0, 0.0, 0.0
		p, q, r =  0.0, 0.0, 0.0
		
		if (t > 0):	
			vx = (x - px) / timestep
			vy = (y - py) / timestep
			vz = (z - pz) / timestep
		
			if (t < maxtime):

				fx = math.sin((t + timestep) * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0]
				fy = math.sin((t + timestep) * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1]
				fz = math.sin((t + timestep) * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2]

				p1 = np.array([px,py,pz])
				p2 = np.array([x,y,z])
				p3 = np.array([fx,fy,fz])

				v1 = np.subtract(p2,p1)
				v2 = np.subtract(p3,p2)

				R = rotation_matrix_from_vectors(v1,v2)
				sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

				roll = math.atan2(R[2,1] , R[2,2])
				pitch = math.atan2(-R[2,0], sy)
				yaw = math.atan2(R[1,0], R[0,0])

				p = roll / timestep
				q = pitch / timestep
				r = roll / timestep

		px, py, pz = x, y, z

		######## END STUDENT CODE
		the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz))
		the_file.write("," + fmt(yaw) + "," + fmt(pitch) + "," + fmt(roll))
		the_file.write("," + fmt(p) + "," + fmt(q) + "," + fmt(r))

		######## EXAMPLE SOLUTION
		#the_file.write("," + fmt((x-px)/timestep) + "," + fmt((y-py)/timestep) + "," + fmt((z-pz)/timestep));
		#px = x;
		#py = y;
		#pz = z;
		######## END EXAMPLE SOLUTION
		
		the_file.write("\n")
		
		t += timestep
			
