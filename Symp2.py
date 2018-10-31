import matplotlib.pyplot as plt
import math
import numpy as np
plt.ion()

G = 6.673e-11  # gravity constant
gridArea = [-10000, 10000, -10000, 50000]  # margins of the coordinate grid
gridScale = 100000000.  # 1 unit of grid equals 100000000m or 100000km

plt.clf()  # clear plot area
plt.axis(gridArea)  # create new coordinate grid
plt.grid(b="on")  # place grid



class Object:
    _instances = []
    def __init__(self, name, position, radius, mass, xvelocity, yvelocity):
        self.name = name
        self.position = position
        self.radius = radius  # in grid values
        self.mass = mass
        self.placeObject()
        self.xvelocity = xvelocity
        self.yvelocity = yvelocity
        self.velocity = math.sqrt(self.xvelocity**2 + self.yvelocity**2)

        #self.xvelocity2 = 0
        #self.yvelocity2 = 0
        #self.velocity2 = math.sqrt(self.xvelocity1**2 + self.yvelocity1**2)

        Object._instances.append(self)

    def placeObject(self):
        drawObject = plt.Circle(self.position, radius=self.radius, fill=False, color="black")
        plt.gca().add_patch(drawObject)
        plt.show()

    def giveMotion(self, time):
        if self.velocity != 0:
            x1_comp = self.xvelocity
            y1_comp = self.yvelocity
            self.velocity = math.sqrt((x1_comp**2)+(y1_comp**2))

        #else:
            #self.velocity = self.velocity + deltaV  # in m/s
        self.time = time  # in seconds
        self.vectorUpdate()

    def vectorUpdate(self):
        self.placeObject()
        data = []
        data2 = []
        distance1 = []
        distance2= [10000000]
        i = np.sqrt(4*G*1.9885e30/700000000)
        print(i)

        for t in range(self.time):
            for x in [y for y in Object._instances if y is not self]:
                #distance b/t
                r_vec = [self.position[0] - x.position[0], self.position[1] - x.position[1]]
                distance = np.linalg.norm(r_vec)
                distance1.append(distance)
                if t > 5:
                    if distance1[t-4] > distance1[t-3] and distance1[t-3] < distance1[t-2]:
                        #ratio = np.tan(np.pi/2 -self.position[0]/self.position[1])
                        distance2.append(distance1[t-3])
                        #ratio = np.arctan(x.yvelocity/x.xvelocity)
                        #ratio2 = np.arctan(self.position[1]/self.position[0])

                        x.xvelocity -= np.cos(ratio2)*.005*x.xvelocity #-  35*np.cos(ratio)
                        x.yvelocity -= np.sin(ratio2)*.005*x.yvelocity #- 35*np.sin(ratio)
                        self.xvelocity -= np.cos(ratio2)*.005*self.xvelocity #+  35*np.cos(ratio2)
                        self.yvelocity -= np.sin(ratio2)*.005*self.yvelocity #+ 35*np.sin(ratio2)
                        
              

                ratio2 = np.arctan(self.position[1]/self.position[0])
                F_g = G*x.mass/((distance*gridScale)**2)
                F_gx = np.cos(ratio2)*F_g
                ax_1 = G * self.mass*gridScale*(self.position[0] - x.position[0])/((distance*gridScale)**3)
                ax_2 = G * x.mass*gridScale*(-self.position[0] + x.position[0])/((distance*gridScale)**3)
                ay_1 = G * self.mass*gridScale*(self.position[1] - x.position[1])/((distance*gridScale)**3)
                ay_2 = G * x.mass*gridScale*(-self.position[1] + x.position[1])/((distance*gridScale)**3)

                '''if t > 10:
                    if distance1[t-1] < distance1[t-2]:
                        print("V_Esc not achieved")'''


                '''print("ax_2:")
                print(ax_2)
                print("ay_1")
                print(ay_1)'''

                self.xvelocity += 2*ax_2 - 2*ax_1 # update velocity
                self.yvelocity += 2*ay_2 - 2*ay_1
                #x.xvelocity += ax_1
                #.yvelocity += ay_1
                traveledx1 = self.xvelocity/gridScale  # x grid distance traveled per 1 sec
                traveledy1 = self.yvelocity/gridScale  # y grid distance

                #x.xvelocity += 5*ax_1
                #x.yvelocity += 5*ay_1
                #traveledx2 = x.xvelocity/gridScale
                #traveledy2 = x.yvelocity/gridScale
                traveledx2 = 0
                traveledy2 = 0

                self.position = (self.position[0] + traveledx1, self.position[1] + traveledy1)  # update pos
                x.position = (x.position[0] + traveledx2, x.position[1] + traveledy2)

                data.append([self.position[0], self.position[1]])
                data2.append([x.position[0], x.position[1]])

                collision = 0
            for x in [y for y in Object._instances if y is not self]:
                if (self.position[0] - x.position[0])**2 + (self.position[1] - x.position[1])**2 <= x.radius**2:
                    collision = 1
                    break
            if collision != 0:
                print("Collision!")
                break
        plt.plot([x[0] for x in data], [x[1] for x in data], [x[0] for x in data2], [x[1] for x in data2])

        #plt.plot([x[0][0] for x in data], [x[0][1] for x in data])



Earth = Object(name="Earth", position=(0.0, 0.0), radius=6.95, mass=20.5*1.9885e30, xvelocity = 0., yvelocity = 0.)
Moon = Object(name="Moon", position=(-500.,12*250*6.95), radius=1, mass = 1.4*1.9885e30, xvelocity = 30000., yvelocity = 0.)

#Earth.giveMotion(time=60000)
Moon.giveMotion(time=60000000)

plt.show(block=True)