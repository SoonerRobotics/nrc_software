# this file copied from the igvc_software repo and will be used as a template to make nrc_pure_pursuit.py

import math

class PurePursuit:


# Stuff from igvc_nav that uses pure_pursuit:

# def timer_callback(event):
#     if pos is None or heading is None:
#         return

#     lookahead = None
#     radius = 0.4 # Start with a radius of 0.1 meters

#     while lookahead is None and radius <= 2: # Look until we hit 2 meters max
#         lookahead = pp.get_lookahead_point(pos[0], pos[1], radius)
#         radius *= 1.25
    
#     if lookahead is not None:
#         heading_to_la = 90 - math.atan2(lookahead[1] - pos[1], lookahead[0] - pos[0]) * 180 / (math.pi)
#         if heading_to_la < 0:
#             heading_to_la += 360

#         # print('my x: ' + str(pos[0]))
#         # print('my y: ' + str(pos[1]))

#         # print('loc x: ' + str(lookahead[0]))
#         # print('loc y: ' + str(lookahead[1]))

#         # print('want to move x:' + str(lookahead[0] - pos[0]))
#         # print('want to move y:' + str(lookahead[1] - pos[1]))

#         # print('i think my heading is ' + str(heading))
#         # print('i want heading ' + str(heading_to_la))
#         delta = heading_to_la - heading
#         delta = (delta + 180) % 360 - 180

#         motor_pkt = motors()
#         motor_pkt.left = 2 + 1 * (delta / 180)
#         motor_pkt.right = 2 - 1 * (delta / 180)
        
#         publy.publish(motor_pkt)


    def __init__(self):
        self.path = []
    
    def add_point(self, x, y):
        self.path.append((x,y))

    def set_points(self, pts):
        self.path = pts

    def get_lookahead_point(self, x, y, r):
        lookahead = None

        for i in range(len(self.path)-1):
            segStart = self.path[i]
            segEnd = self.path[i+1]

            p1 = (segStart[0] - x, segStart[1] - y)
            p2 = (segEnd[0] - x, segEnd[1] - y)

            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]

            d = math.sqrt(dx * dx + dy * dy)
            D = p1[0] * p2[1] - p2[0] * p1[1]

            discriminant = r * r * d * d - D * D
            if discriminant < 0 or p1 == p2:
                continue

            sign = lambda x: (1, -1)[x < 0]

            x1 = (D * dy + sign(dy) * dx * math.sqrt(discriminant)) / (d * d)
            x2 = (D * dy - sign(dy) * dx * math.sqrt(discriminant)) / (d * d)

            y1 = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (d * d)
            y2 = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (d * d)

            validIntersection1 = min(p1[0], p2[0]) < x1 and x1 < max(p1[0], p2[0]) or min(p1[1], p2[1]) < y1 and y1 < max(p1[1], p2[1])
            validIntersection2 = min(p1[0], p2[0]) < x2 and x2 < max(p1[0], p2[0]) or min(p1[1], p2[1]) < y2 and y2 < max(p1[1], p2[1])

            if validIntersection1 or validIntersection2:
                lookahead = None

            if validIntersection1:
                lookahead = (x1 + x, y1 + y)

            if validIntersection2:
                if lookahead == None or abs(x1 - p2[0]) > abs(x2 - p2[0]) or abs(y1 - p2[1]) > abs(y2 - p2[1]):
                    lookahead = (x2 + x, y2 + y)

        if len(self.path) > 0:
            lastPoint = self.path[len(self.path) - 1]

            endX = lastPoint[0]
            endY = lastPoint[1]

            if math.sqrt((endX - x) * (endX - x) + (endY - y) * (endY - y)) <= r:
                return (endX, endY)

        return lookahead