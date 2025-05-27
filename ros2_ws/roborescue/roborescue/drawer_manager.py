#!/usr/bin/env python3
import math
DIRECTION = ["LEFT", "RIGHT"]
DEFINED_LETTERS = ["R","O","B"]

class DrawerManager:
    """ Main class to manage the trajectory for drawing a letter or an object. \n
    The turtle should start from the bottom right or bottom left, depending on the desired direction. """


    def __init__(self, length, width):
        self.length = length
        self.width = width

    def manager(self, letter_, origin_x, origin_y, direction):
        """ Wrapper function to return a list of points to draw all the defined letters"""

        if direction not in DIRECTION:
            return None
        
        letter = letter_.upper()
        
        if letter == "R":
            return self.draw_R(origin_x, origin_y, direction)
        elif letter == "O":
            return self.draw_O(origin_x, origin_y, direction)
        elif letter == "B":
            return self.draw_B(origin_x, origin_y, direction)
        elif letter == "E":
            return self.draw_E(origin_x, origin_y, direction)
        elif letter == "S":
            return self.draw_S(origin_x, origin_y, direction)
        elif letter == "C":
            return self.draw_C(origin_x, origin_y, direction)
        elif letter == "U":
            return self.draw_U(origin_x, origin_y, direction)
        elif letter == "I":
            return self.draw_I(origin_x, origin_y, direction)
        else:
            return None
        
    def robot_manager(self, part, origin_x, origin_y):
        """ Wrapper function to return a list of points to draw the robot and the antenna circle"""
        letter = part.upper()
        
        if letter == "MOUTH":
            return self.draw_MOUTH(origin_x, origin_y)
        elif letter == "EYE":
            return self.draw_EYE(origin_x, origin_y)
        elif letter == "ROBOT":
            return self.draw_ROBOT(origin_x, origin_y)
        else:
            return None
        
    def draw_R(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """ Returns a list of points (x,y) to draw the letter R"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right = [
            (x0 - w/2, y0),
            (x0 - (2*w)/3, y0 + l / 2), # diagonal to halfway up the letter height
            (x0 - (2*w)/3, y0),         # vertical line down to the point at y=0 and x=width/3
            (x0 - w, y0),
            (x0 - w, y0 + l),
            (x0, y0 + l),           
            (x0, y0 + l / 2),
            (x0 - w/2, y0 + l / 2),
            (x0, y0)
        ]

        points_bottom_left = [
            (x0, y0 + l),           # vertical line upward by letter height
            (x0 + w, y0 + l),       # horizontal line right by letter width
            (x0 + w, y0 + l / 2),   # downward line by half the letter height
            (x0 + w/2, y0 + l / 2), # horizontal line left to half the letter width
            (x0 + w, y0),           # diagonal to the point at y=0 and x=width
            (x0 + w / 2, y0),       # horizontal line to half the width
            (x0 + w/3, y0 + l / 2), # diagonal to halfway up the letter height
            (x0 + w/3, y0),         # vertical line down to the point at y=0 and x=width/3
            (x0, y0)                # horizontal line left back to the start point
        ]

        return self.choise_direction(points_bottom_right, points_bottom_left, direction)


    def draw_O(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """ Returns a list of points (x,y) to draw the letter O"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right = [
            (x0-w, y0),             # horizontal line left by letter width
            (x0-w, y0 + l),         # vertical line upward by letter height
            (x0, y0 + l),           # horizontal line right by letter width
            (x0, y0),               # vertical line downward by letter height
        ]

        points_bottom_left = [
            (x0, y0 + l),           # vertical line upward by letter height
            (x0+w, y0 + l),         # horizontal line right by letter width
            (x0+w, y0),             # vertical line downward by letter height
            (x0, y0),               # horizontal line left by letter width
        ]

        return self.choise_direction(points_bottom_right, points_bottom_left, direction)

    def draw_B(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """ Returns a list of points (x,y) to draw the letter B"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right = [
            (x0-w, y0),             # horizontal line left by letter width
            (x0-w, y0 + l),         # vertical line upward by letter height
            (x0, y0 + l),           # horizontal line right by letter width
            (x0, y0 + l - l/3),     # vertical line down
            (x0-w/2, y0 + l / 2),   # diagonal line downward by half the letter height
            (x0, y0 + l/3),         # diagonal line downward 
            (x0, y0),               # diagonal line right back to the start point
        ]

        points_bottom_left = [
            (x0, y0 + l),           # vertical line upward by letter height
            (x0 + w, y0 + l),       # horizontal line right by letter width
            (x0 + w, y0 + l -l/3),  # vertical line down
            (x0 + w/2, y0 + l/2),   # diagonal to the center point
            (x0 + w, y0 + l/3),           
            (x0 + w, y0),           # vertical line down
            (x0, y0)                # horizontal line left back to the start point
        ]

        return self.choise_direction(points_bottom_right, points_bottom_left, direction)

    def draw_S(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """Returns a list of points (x, y) to draw the letter S"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right = [
            (x0 - w, y0),             # bottom left
            (x0 - w, y0 + l/3),                 
            (x0 - w + w/2, y0 + l/3),                  
            (x0 - w + w/4, y0 + l/3),                 
            (x0 - w + w/4, y0 + l),                 
            (x0, y0 + l),                 
            (x0, y0 + l - l/3),                 
            (x0 - w + w/2, y0 + l - l/3),                 
            (x0 - w + w/2 + w/4, y0 + l - l/3),                 
            (x0 , y0),                # bottom right (start point)                    
        ]
        
        points_bottom_left = [
            (x0, y0 + l/3),
            (x0 + w/2, y0 + l/3),
            (x0 + w/4, y0 + l/3),
            (x0 + w/4, y0 + l),
            (x0 + w, y0 + l),
            (x0 + w, y0 + l - l/3),
            (x0 + w - w/2, y0 + l - l/3),
            (x0 + w - w/4, y0 + l - l/3),           
            (x0 + w , y0),           # Diagonal down to bottom right
            (x0 , y0)           
        ]

        return self.choise_direction(points_bottom_right, points_bottom_left, direction)
        
    def draw_C(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """Returns a list of points (x, y) to draw the letter C"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right = [
            (x0 - w, y0),             # bottom left -> bottom right
            (x0 - w, y0 + l),         # top left
            (x0 , y0 + l),            # top right
            (x0 , y0 + l - l/3),            
            (x0 - w + w/3, y0 + l - l/3),            
            (x0 - w + w/3, y0 + l/3),            
            (x0, y0 + l/3),        
            (x0, y0)                 # bottom left
        ]
        
        points_bottom_left = [
            (x0, y0 + l),                     # top right
            (x0 + w, y0 + l),                 # top left
            (x0 + w, y0 + l - l/3),
            (x0 + w/3, y0 + l - l/3),
            (x0 + w/3, y0 + l/3),
            (x0 + w, y0 + l/3),
            (x0 + w, y0),                     # bottom right 
            (x0, y0),                         # bottom left 
        ]

        return self.choise_direction(points_bottom_right, points_bottom_left, direction)
        
    def draw_U(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """Returns a list of points (x, y) to draw the letter U"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right =[
            (x0 - w, y0),             # down left
            (x0 - w, y0 + l),         # top left    
            (x0 - w + w/3, y0 + l),    
            (x0 - w + w/3, y0 + l/3),    
            (x0 - w + w/3 + w/3, y0 + l/3),    
            (x0 - w + w/3 + w/3, y0 + l),    
            (x0 , y0 + l),            # top right
            (x0 , y0)                 # bottom right (start point)    
        ]
        
        points_bottom_left = [
            (x0, y0 + l),            # top left
            (x0 + w/3, y0 + l),            
            (x0 + w/3, y0 + l/3),            
            (x0 + w/3 + w/3, y0 + l/3),            
            (x0 + w/3 + w/3, y0 + l),            
            (x0 + w, y0 + l),       # top right     
            (x0 + w, y0),           # bottom right            
            (x0, y0),               # bottom right            
        ]
        
        return self.choise_direction(points_bottom_right, points_bottom_left, direction)

    def draw_E(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """Returns a list of points (x, y) to draw the letter E"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right =[
                (x0 - w, y0),             # bottom left
                (x0 - w, y0 + l),         # top left    
                (x0, y0 + l),             # top right
                (x0, y0 + l - l/3),             
                (x0 - w + w/3, y0 + l - l/3),             
                (x0 - w + 2*(w/3), y0 + l - l/3),             
                (x0 - w + 2*(w/3), y0 + l - 2*(l/3)),             
                (x0 - w + w/3 , y0 + l - 2*(l/3)),             
                (x0 , y0 + l - 2*(l/3)),             
                (x0 , y0)                 # bottom right (initial point)
            ]
        
        points_bottom_left = [
            (x0, y0 + l),                           # top left
            (x0 + w, y0 + l),                       # top right
            (x0 + w, y0 + l - l/3),                
            (x0 + w/3, y0 + l - l/3),          
            (x0 + 2*(w/3), y0 + l - l/3),          
            (x0 + 2*(w/3), y0 + l/3),          
            (x0 + w/3, y0 + l/3),          
            (x0 + w, y0 + l/3),              
            (x0 + w, y0),                           # bottom right
            (x0 , y0)                               # bottom left (initial point)
        ]
        
        return self.choise_direction(points_bottom_right, points_bottom_left, direction)
    
    def draw_I(self, origin_x=5.54, origin_y=5.54, direction="RIGHT"):
        """ Returns a list of points (x,y) to draw the letter I """
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points_bottom_right = [
            (x0 - w, y0),             # base horizontal line (left)
            (x0, y0),                 # base horizontal line (right)
            (x0 - w/2, y0),           # center bottom
            (x0 - w/2, y0 + l),       # vertical line up
            (x0 - w, y0 + l),         # top horizontal line (left)
            (x0, y0 + l),             # top horizontal line (right)
        ]

        points_bottom_left = [
            (x0, y0),                 # base horizontal line (left)
            (x0 + w, y0),             # base horizontal line (right)
            (x0 + w/2, y0),           # center bottom
            (x0 + w/2, y0 + l),       # vertical line up
            (x0, y0 + l),             # top horizontal line (left)
            (x0 + w, y0 + l),         # top horizontal line (right)
        ]

        return self.choise_direction(points_bottom_right, points_bottom_left, direction)
    
    def draw_MOUTH(self, origin_x=5.54, origin_y=5.54):
        """Returns a list of points (x,y) to draw an eye (circle starting from bottom, CCW)"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y
        
        points = [
            (x0, y0),               # start point
            (x0 - w, y0),        
            (x0 - w, y0 - l/6),  
            (x0, y0 - l/6),   
            (x0, y0),
        ]

        return points

    def draw_EYE(self, origin_x=5.54, origin_y=5.54):
        """Returns a list of points (x,y) to draw an eye (circle starting from bottom, CCW)"""
        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points = [
            (x0, y0),               # start point
            (x0 + w/2, y0),        
            (x0 + w/2, y0 + l/6),  
            (x0, y0 + l/6),   
            (x0, y0),
        ]

        return points

    def draw_ROBOT(self, origin_x=5.54, origin_y=5.54):
        """ Returns a list of points (x,y) to draw the robot and the antenna circle """

        l = self.length
        w = self.width
        x0, y0 = origin_x, origin_y

        points = [
            # LEFT EAR
            (x0, y0),               # start point
            (x0, y0 + l / 6),
            (x0, y0 - l / 6),
            (x0, y0),
            (x0 + w/3, y0),
            # HEAD
            (x0 + w/3, y0 + l/2),
            (x0 + (w), y0 + l/2),
            (x0 + (w), y0 + l/1.25),
        ]
        # ANTENNA
        cx = x0 + w
        cy = y0 + l/1.25
        r = w / 6
        segments = 12  # more segments = better circle

        start_angle = 3 * math.pi / 2
        end_angle = start_angle + 2 * math.pi

        circle = []
        for i in range(segments):
            angle = start_angle + (2 * math.pi * i / segments)
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)
            circle.append((x, y))
        circle.append(circle[0])  # close circle

        points.extend(circle)
        # Continue drawing
        points += [
            (x0 + (w), y0 + l/2),
            (x0 + (w*(5/3)), y0 + l/2),
            (x0 + (w*(5/3)), y0),
            # RIGHT EAR
            (x0 + (w*(6/3)), y0),
            (x0 + (w*(6/3)), y0 + l / 6),
            (x0 + (w*(6/3)), y0 - l / 6),
            (x0 + (w*(6/3)), y0),
            (x0 + (w*(5/3)), y0),
            # Head part 2
            (x0 + (w*(5/3)), y0 - l/2),
            (x0 + w/3, y0 - l/2),
            (x0 + w/3, y0),
        ]

        return points   
        
    def choise_direction(self, points_bottom_right, points_bottom_left, direction):
        if direction == "RIGHT":
            return points_bottom_right
        elif direction == "LEFT" :
            return points_bottom_left
