import numpy as np
import cv2 as cv
from tb3_driving.param import ROBOT_MARGIN, Colour, MIN_THRESHOLD, THRESHOLD, THRESHOLD_STEP
PI = np.pi

colour = Colour()

background_colour = colour.grey(10)
font = cv.FONT_HERSHEY_SIMPLEX
font_scale = 0.5
ltype = cv.LINE_AA
#ROBOT_MARGIN = Source.ROBOT_MARGIN

class Info_Formatting():
    def __init__(self, info_height:int=200, height:int=1000, width:int=1000):
        vertical_space = int(info_height/4)
        left_margin = 40
        data_left_margin = left_margin + 180
        left_margin_2 = int(width/2) + 40
        data_left_margin_2 = left_margin_2 + 240
        self.info_background = colour.grey(5)

        self.item1 = (left_margin, height - vertical_space)
        self.item2 = (data_left_margin_2, height - vertical_space)
        self.item3 = (left_margin, height + vertical_space*3)

        self.item4 = (left_margin, height - int(vertical_space/2))
        self.item5 = (data_left_margin_2, height - int(vertical_space/2))
        self.item6 = (data_left_margin, height + vertical_space*3)

        

        self.item7 = (left_margin_2, height + vertical_space)
        self.item8 = (left_margin_2, height + vertical_space*2)
        self.item9 = (left_margin_2, height + vertical_space*3)

        self.item10 = (data_left_margin_2, height + vertical_space)
        self.item11 = (data_left_margin_2, height + vertical_space*2)
        self.item12 = (data_left_margin_2, height + vertical_space*3)


class Graph:
    """Initialise Cartesian coordinate plane
    """
    def __init__(self, max_x=1000, max_y=1000, width: int = 1000, height: int = 1000,\
        info_height: int = 200):

        self.width = width
        self.height = height
        self.origin = (int(width/2), int(height/2))
        self.max_range = min(max_x, max_y)
        self.scale = min(width, height) / max(max_x*2, max_y*2)

        self.info_height = info_height
        #self.canvas_height = height + self.info_height
        self.canvas_height = height

        self.grid = np.full((self.canvas_height, width, 3), background_colour, np.uint8)
        self.plot = np.zeros((self.height, width, 3), np.uint8)
        self.canvas = np.zeros((self.canvas_height, width, 3), np.uint8)

        self.pose = [0, 0, 0]

        #self.is_autonomous = True

        self.init_grid()

    def init_grid(self):

        # TurtleBot3 margin
        cv.circle(self.grid, self.origin, int(ROBOT_MARGIN*self.scale), colour.grey(2), -1)

        # x-axis
        x_axis_startpoint = (self.origin[1], self.height+1)
        x_axis_endpoint = (self.origin[1], 1)
        cv.arrowedLine(self.grid, x_axis_startpoint, x_axis_endpoint, colour.grey(25),\
            tipLength=0.015, line_type=ltype)

        # y-axis
        y_axis_startpoint = (self.width+1, self.origin[0])
        y_axis_endpoint = (1, self.origin[0])
        cv.arrowedLine(self.grid, y_axis_startpoint, y_axis_endpoint, colour.grey(25), \
            tipLength=0.015, line_type=ltype)

        # Origin
        cv.line(self.grid, self.origin, self.origin, colour.black, 3, ltype)

        # Information area
        self.info_format = Info_Formatting(self.info_height, self.height, self.width)
        cv.rectangle(self.grid, (0, self.height), (self.width, self.canvas_height), \
            self.info_format.info_background, -1)
        cv.putText(self.grid, "Robot coordinates:", \
            self.info_format.item1, font, font_scale, colour.white, 1, ltype)
        cv.putText(self.grid, "Robot orientation:", \
            self.info_format.item2, font, font_scale, colour.white, 1, ltype)
        cv.putText(self.grid, "Target coordinates:", \
            self.info_format.item3, font, font_scale, colour.white, 1, ltype)

        cv.putText(self.grid, "Linear speed:", \
            self.info_format.item7, font, font_scale, colour.white, 1, ltype)
        cv.putText(self.grid, "Angular speed:", \
            self.info_format.item8, font, font_scale, colour.white, 1, ltype)
        cv.putText(self.grid, "Operation mode:", \
            self.info_format.item9, font, font_scale, colour.white, 1, ltype)

        #print("Finished initialising grid")

    def get_coord(self, x, y):
        return (int(self.origin[0] - y*self.scale), int(self.origin[1] - x*self.scale))

    def get_coord_from_tuple(self, point: tuple):
        return (int(self.origin[0] - point[1]*self.scale), \
            int(self.origin[1] - point[0]*self.scale))

    def cartesian_from_polar(self, r, theta, theta_in_deg=True):
        if theta_in_deg:
            theta = theta / 180*np.pi
        r *= 1000
        xr = r * np.cos(theta)
        yr = r * np.sin(theta)
        return self.get_coord(xr, yr)

    def generate_range_circles(self, no_circle=4):
        for i in range(no_circle):
            radius = int(self.max_range / (no_circle) * (i+1))
            #print(radius)
            cv.circle(self.grid, self.origin, int(radius*self.scale), colour.grey(25), \
                lineType=ltype)
            #cv.circle(self.grid, self.origin, int(radius*self.scale), colour.green, lineType=ltype)
            text_x, text_y = self.get_coord(-radius/np.sqrt(2), -radius/np.sqrt(2))
            #print(type(text_location[0]))
            cv.putText(self.grid, str(radius), (text_x+8, text_y+8), font, 0.5,\
                colour.grey(25), 1, ltype)
            #cv.putText(self.grid, str(radius), (text_x, text_y), font, 0.5, colour.green, 1, ltype)

    
    def scatter(self, points, col=colour.grey(100), inp_radius=2, excp=np.inf):
        i = 0
        scale = 360/len(points)
        for point in points:
            if point != excp:
                point_to_plot = self.cartesian_from_polar(point, i*scale)
                cv.circle(self.plot, point_to_plot, inp_radius, col, -1, ltype)
            i += 1

    def point_plot(self, point, col=colour.red, inp_radius=8, excp=np.inf):
        xt = point[0];         yt = point[1]
        if xt != excp and yt != excp:
            xc = self.pose[0];      yc = self.pose[1]
            r = np.sqrt((xt-xc)**2 + (yt-yc)**2)
            theta = np.arctan2(yt-yc, xt-xc) - self.pose[2]
            point_to_plot = self.cartesian_from_polar(r, theta, False)
            cv.circle(self.plot, point_to_plot, inp_radius, col, -1, ltype)

    def cross_plot(self, point, col=colour.yellow, inp_radius=15, inp_thickness=1, excp=np.inf):
        xt = point[0];         yt = point[1]
        if xt != excp and yt != excp:
            xc = self.pose[0];      yc = self.pose[1]
            r = np.sqrt((xt-xc)**2 + (yt-yc)**2)
            theta = np.arctan2(yt-yc, xt-xc) - self.pose[2]
            point_to_plot = self.cartesian_from_polar(r, theta, False)

            x_start = (point_to_plot[0] - inp_radius, point_to_plot[1])
            x_end = (point_to_plot[0] + inp_radius, point_to_plot[1])
            y_start = (point_to_plot[0], point_to_plot[1] - inp_radius)
            y_end = (point_to_plot[0], point_to_plot[1] + inp_radius)
            cv.line(self.plot, x_start, x_end, col, inp_thickness, ltype)
            cv.line(self.plot, y_start, y_end, col, inp_thickness, ltype)

    def update_info(self, pose, linear_speed=0, angular_speed=0, is_autonomous=False,\
        target=(0, 0)):
        """Update information panel
        ==============================================
        :param pose: Tuple (x, y, theta)"""

        cv.putText(self.plot, "({:.3f}, {:.3f}) m".format(pose[0], pose[1]), \
            self.info_format.item4, font, font_scale, colour.white, 1, ltype)
        
        cv.putText(self.plot, "{:.3f} deg".format(pose[2]/np.pi*180), \
            self.info_format.item5, font, font_scale, colour.white, 1, ltype)
        
        cv.putText(self.plot, "({:.3f}, {:.3f}) m".format(target[0], target[1]), \
            self.info_format.item6, font, font_scale, colour.white, 1, ltype)

        cv.putText(self.plot, "{:.3f} m/s".format(linear_speed), \
            self.info_format.item10, font, font_scale, colour.white, 1, ltype)
        
        cv.putText(self.plot, "{:.3f} rad/s".format(angular_speed), \
            self.info_format.item11, font, font_scale, colour.white, 1, ltype)

        if is_autonomous:
            cv.putText(self.plot, "AUTONOMOUS", \
                self.info_format.item12, font, font_scale, colour.white, 1, ltype)
        else:
            cv.putText(self.plot, "MANUAL", \
                self.info_format.item12, font, font_scale, colour.white, 1, ltype)

    def standardise_angle_deg(self, angle_in_deg, output_less_than_180=False):
        while angle_in_deg >= 360: angle_in_deg -= 360
        while angle_in_deg < 0: angle_in_deg += 360
        if angle_in_deg <= 180 or not output_less_than_180:
            return angle_in_deg
        else:
            return 360 - angle_in_deg

    def get_available_directions(self, scan_ranges, threshold):
        r = np.inf
        dead_angle = -np.inf
        result = np.zeros(360, float)
        robot_margin = ROBOT_MARGIN * 1.0 / 1000
        for i in range(540):
            index = self.standardise_angle_deg(i)
            ri = scan_ranges[index]
            #if ri == 0 or np.abs(robot_margin / (2*ri)) > 1:
            #    result[index] = 0
            #    continue
            if ri < threshold:
                r = ri
                dead_angle = i + round(2 * np.arcsin(robot_margin / (2*r)) / PI*180)
                result[index] = 0
            else:
                if i > dead_angle:
                    r = np.inf
                    dead_angle = -np.inf
                    result[index] = threshold
                else:
                    result[index] = 0

        r = np.inf
        dead_angle = np.inf
        for i in range(359, -180, -1):
            index = self.standardise_angle_deg(i)
            ri = scan_ranges[index]
            #if ri == 0 or np.abs(robot_margin / (2*ri)) > 1: 
            #    result[index] = 0
            #    continue
            if ri < threshold:
                r = ri
                dead_angle = i - round(2 * np.arcsin(robot_margin / (2*r)) / PI*180)
                result[index] = 0
            else:
                if i < dead_angle:
                    r = np.inf
                    dead_angle = np.inf
                    #result[i] = 1
                else:
                    result[index] = 0

        return result

    def vision(self, points):
        t = MIN_THRESHOLD
        while t <= THRESHOLD:
            A = self.get_available_directions(points, t)
            self.scatter(A, colour.light_green, 1, excp=0)
            t += THRESHOLD_STEP

    def update_map(self, points, points_2, target, sub_target, waypoint, pose,
                    linear_speed=0, angular_speed=0, is_autonomous=False):

        self.plot = np.zeros((self.canvas_height, self.width, 3), np.uint8)
        self.pose = pose

        self.scatter(points)
        self.scatter(points_2, colour.light_green, 1, excp=0)
        #self.vision(points)

        if is_autonomous:
            self.point_plot(target)
            self.point_plot(waypoint, colour.green, 5)
            self.cross_plot(sub_target)

        self.update_info(pose, linear_speed, angular_speed, is_autonomous, target)
        self.canvas = cv.addWeighted(self.grid, 1, self.plot, 1, 0)
        cv.imshow("Map", self.canvas)
        cv.waitKey(1)
