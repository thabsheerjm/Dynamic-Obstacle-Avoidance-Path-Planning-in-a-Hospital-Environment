import rospy
from nav_msgs.msg import OccupancyGrid
from d_star import DStar

def setup():
    rospy.init_node('dStar', anonymous=True)
    rospy.Subscriber("getMap", OccupancyGrid, load_map) 

# Load map, start and goal point.
def load_map(data):
    # TODO: Change this into a service call and adjust start and goal
    grid = data.data

if __name__ == "__main__":
    rospy.logwarn("reading new script")
    # Load the map
    setup()
    rospy.logwarn("made it here")
    # TODO: adjust dynamic grid to be sensor values
    # Search
    d_star = DStar()

    # Run D*
    # d_star.run()
    print("test")
