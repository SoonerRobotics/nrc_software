
import json
import matplotlib.pyplot as plt

from math import cos, sin, radians

from segment import Segment
from trajectory_point import TrajectoryPoint

course_x = [0]
course_y = [0]
course_segs = []
traj_x = []
traj_y = []
traj_pts = []
wpt_x = []
wpt_y = []
wpt_u = []
wpt_v = []

def plot_course(course_info):
    xt = 0
    yt = 0
    course_x = [0]
    course_y = [0]

    # Run through all the segments
    for i in range(len(course_info['segments'])):
        # Get settings
        hdg = course_info['segments'][i]['heading']
        length = course_info['segments'][i]['length']

        # Set the segment start point and other info
        seg = Segment()
        seg.set_start(xt, yt)
        seg.set_hdg(hdg)

        # Update the current endpoint based on the settings
        xt = xt + length * cos(radians(hdg))
        yt = yt + length * sin(radians(hdg))

        # Set the segment endpoint
        seg.set_end(xt, yt)

        # Add segment to the course
        course_segs.append(seg)

        # Append endpoint to the list
        course_x.append(xt)
        course_y.append(yt)

    # Plot the course
    plt.plot(course_x, course_y, 'r')


def plot_waypoints(waypoints):
    # Reset the waypoints
    wpt_x = []
    wpt_y = []
    wpt_u = []
    wpt_v = []

    # Loop through the waypoints
    for i in range(len(waypoints)):
        # Get waypoint info
        hdg = waypoints[i]['heading']
        x = waypoints[i]['x']
        y = waypoints[i]['y']

        # Update display lists
        wpt_x.append(x)
        wpt_y.append(y)
        wpt_u.append(cos(radians(hdg)))
        wpt_v.append(sin(radians(hdg)))

    # Show waypoints as vector field
    plt.quiver(wpt_x, wpt_y, wpt_u, wpt_v)



# Main function
if __name__ == "__main__":
    # Open the configuration parameters file
    config_file = open("config.json", 'r')

    # Parse the configuration as JSON
    params = json.load(config_file)

    # Prepare the course plot
    plot_course(params['course'])

    # Prepare the trajectory
    plot_waypoints(params['waypoints'])

    # TODO: export data to CSV

    # Display plot of trajectory and course
    plt.show()
