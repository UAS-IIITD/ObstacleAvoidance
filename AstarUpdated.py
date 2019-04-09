import matplotlib.pyplot as plt
import utm
from math import sqrt
import simplekml
from polycircles import polycircles

show_animation = True
obx = [-35.3636919, -35.3635552]
oby = [149.1642845, 149.1641450]
radius = [5, 5]

def draw_kml_circles(obx,oby,radius):
	kml = simplekml.Kml()
	for i in range(len(obx)):
		polycircle = polycircles.Polycircle(latitude=obx[i],
                                    longitude=oby[i],
                                    radius=radius[i],
                                    number_of_vertices=100)
		
		pol = kml.newpolygon(name="obstacle"+str(i),outerboundaryis=polycircle.to_kml())
		pol.style.polystyle.color = simplekml.Color.changealphaint(radius[i], simplekml.Color.green)
	kml.save("circles.kml")

class Node:
	def __init__(self, x, y, cost, pind):
		self.x = x
		self.y = y
		self.cost = cost
		self.pind = pind

	def __str__(self):
		return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_final_path(ngoal, closedset, reso):
	rx, ry = [ngoal.x * reso], [ngoal.y * reso]
	pind = ngoal.pind
	while pind != -1:
		n = closedset[pind]
		rx.append(n.x * reso)
		ry.append(n.y * reso)
		pind = n.pind

	return rx, ry

def a_star_planning(sx, sy, gx, gy, minx, miny, maxx, maxy, reso, rr):
	nstart = Node((sx / reso), (sy / reso), 0.0, -1)
	ngoal = Node((gx / reso), (gy / reso), 0.0, -1)
	motion = get_motion_model()

	openset, closedset = dict(), dict()
	openset[calc_index(nstart, minx, minx, miny)] = nstart

	while 1:
		c_id = min(openset, key=lambda o: openset[o].cost + calc_heuristic(ngoal, openset[o]))
		current = openset[c_id]

		if show_animation: 
			plt.plot(current.x * reso, current.y * reso, "xc")

		delta = 1
		if current.x >= ngoal.x - delta and current.x <= ngoal.x + delta and current.y >= ngoal.y - delta and current.y <= ngoal.y + delta:
			ngoal.pind = current.pind
			ngoal.cost = current.cost
			break

		del openset[c_id]
		closedset[c_id] = current

		for i, _ in enumerate(motion):
			node = Node(current.x + motion[i][0],
						current.y + motion[i][1],
						current.cost + motion[i][2], c_id)
			n_id = calc_index(node, minx, minx, miny)

			if n_id in closedset:
				continue

			if not verify_node(node, minx, miny, maxx, maxy):
				continue

			if n_id not in openset:
				openset[n_id] = node
			else:
				if openset[n_id].cost >= node.cost:
					openset[n_id] = node

	rx, ry = calc_final_path(ngoal, closedset, reso)

	return rx, ry

def calc_heuristic(n1, n2):
	w = 1.0
	d = w * sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
	return d

def is_obstacle(node):
	for cx, cy, r in zip(obx, oby, radius):
		d1 = sqrt((cx - node.x)**2 + (cy - node.y)**2) - 0.5
		if d1 <= r/1.0:
			return True
	return False


def verify_node(node, minx, miny, maxx, maxy):

	if node.x < minx:
		return False
	elif node.y < miny:
		return False
	elif node.x >= maxx:
		return False
	elif node.y >= maxy:
		return False
	elif is_obstacle(node):
		return False

	return True

def calc_index(node, xwidth, xmin, ymin):
	return (node.y - ymin) * xwidth + (node.x - xmin)

def get_motion_model():
	motion = [[1, 0, 1],
			  [0, 1, 1],
			  [-1, 0, 1],
			  [0, -1, 1],
			  [-1, -1, sqrt(2)],
			  [-1, 1, sqrt(2)],
			  [1, -1, sqrt(2)],
			  [1, 1, sqrt(2)]]

	return motion

def create_circle():
	circle= plt.Circle((20, 20), radius= 5)
	return circle

def show_shape(patch):
	ax=plt.gca()
	ax.add_patch(patch)
	plt.axis('scaled')

def main():
	grid_size = 1.0
	drone_size = 1.0
	inputfile = open("input.waypoints", "r")
	outputfile = open("output.waypoints", "w+")
	line = inputfile.readline()
	outputfile.write(line)
	line = inputfile.readline()
	draw_kml_circles(obx,oby,radius)
	
	waypoints = []
	altitude = []
	utmCo = []
	lines = []
	while line:
		if (len(line) > 0):
			lines.append(line)
			line = line.split()
			lx = float(line[8])
			ly = float(line[9])
			alt = float(line[10])
			waypoints.append((lx, ly))
			altitude.append(alt)
		line = inputfile.readline()

	length = len(waypoints)
	outputfile.write(lines[0])
	
	# converting the initial waypoints to utm coordinates
	for i in range(length):
		coordinates = utm.from_latlon(waypoints[i][0], waypoints[i][1])
		utmCo.append(coordinates)

	# No. of waypoints in the final output file
	Count = 0

	# converting the obstacle waypoints to utm coordinates
	for i in range(len(obx)):
		coordinates = utm.from_latlon(obx[i], oby[i]);
		obx[i] = coordinates[0];
		oby[i] = coordinates[1];

	for i in range(length-1):
		j = i+1
		# initial waypoint and final waypoint
		sx = utmCo[i][0]
		sy = utmCo[i][1]
		gx = utmCo[j][0]
		gy = utmCo[j][1]

		prev_x = sx
		prev_y = sy
		prev_wp_is_obstacle = False

		current_x = sx+1
		m = (gy - sy) / (gx - sx)
		c = sy - (m * sx)

		miniwp = []

		if (current_x < gx):
			while current_x < gx:
				current_y = m*current_x + c
				node = Node((current_x/grid_size), (current_y/grid_size), 0, 0)
				cur_wp_is_obstacle = is_obstacle(node)			
				if (not prev_wp_is_obstacle) and cur_wp_is_obstacle:
					miniwp.append((prev_x, prev_y))
				elif prev_wp_is_obstacle and (not cur_wp_is_obstacle):
					miniwp.append((current_x, current_y))
				prev_x = current_x
				prev_y = current_y
				current_x += 1
				prev_wp_is_obstacle = cur_wp_is_obstacle
		else:
			while current_x > gx:
				current_y = m*current_x + c
				node = Node((current_x/grid_size), (current_y/grid_size), 0, 0)
				cur_wp_is_obstacle = is_obstacle(node)			
				if (not prev_wp_is_obstacle) and cur_wp_is_obstacle:
					miniwp.append((prev_x, prev_y))
				elif prev_wp_is_obstacle and (not cur_wp_is_obstacle):
					miniwp.append((current_x, current_y))
				prev_x = current_x
				prev_y = current_y
				current_x -= 1
				prev_wp_is_obstacle = cur_wp_is_obstacle

		for k in range(0, len(miniwp), 2):
			fwpx = []
			fwpy = []
			j = k+1
			sx = miniwp[k][0]
			sy = miniwp[k][1]
			gx = miniwp[j][0]
			gy = miniwp[j][1]
			minx = round(sx) - 100
			miny = round(sy) - 100
			maxx = round(gx) + 100
			maxy = round(gy) + 100

			for c in range(len(obx)):
				circle = plt.Circle((obx[c], oby[c]), radius=radius[c]);
				show_shape(circle);
			if show_animation: 
				plt.plot(sx, sy, "xr")
				plt.plot(gx, gy, "xb")
				plt.grid(True)
				plt.axis("equal")

			rx, ry = a_star_planning(sx, sy, gx, gy, minx, miny, maxx, maxy, grid_size, drone_size)
			rx.reverse()
			ry.reverse()

			for w in range(1, len(rx)-1):
				if (rx[w] == rx[w-1]) and (rx[w] == rx[w+1]):
					continue
				if (ry[w] == ry[w-1]) and (ry[w] == ry[w+1]):
					continue
				if (abs(rx[w-1] - rx[w]) == abs(ry[w-1] - ry[w])) and (abs(rx[w] - rx[w+1]) == abs(ry[w] - ry[w+1])):
					continue

				fwpx.append(rx[w-1])
				fwpy.append(ry[w-1])

			fwpx.append(rx[-1])
			fwpy.append(ry[-1])

			for x, y in zip(fwpx, fwpy):
				line = lines[i]
				line = line.split()
				geoCo = utm.to_latlon(x, y, utmCo[i][2], utmCo[i][3])
				line[1] = '0'
				line[8] = str(geoCo[0])
				line[9] = str(geoCo[1])
				outputfile.write('{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(str(Count) , (line[1]) , (line[2]) , (line[3]) , (line[4]) , (line[5]) , (line[6]) , (line[7]) , (line[8]) , (line[9]) , (line[10]) , (line[11])))
				Count += 1

			if show_animation: 
				plt.plot(fwpx, fwpy, "-r")
				plt.show()

		line = lines[i+1]
		line = line.split();
		line[0] = Count;
		outputfile.write('{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(str(Count) , (line[1]) , (line[2]) , (line[3]) , (line[4]) , (line[5]) , (line[6]) , (line[7]) , (line[8]) , (line[9]) , (line[10]) , (line[11])))

if __name__ == '__main__':
	main()