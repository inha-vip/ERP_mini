import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import rospy

def fuckLPP(msg):
    xlist, ylist = [], []
    LPP_Array = MarkerArray()
    LPP_Array.markers = []
    
    for LPP_count in range (len(msg.points)):
        LPP1 = Marker()
        LPP1.header.frame_id = "world" #+ str(ob_count) # publish path in map frame
        LPP1.ns = "LPP_" + str(LPP_count)
        LPP1.type = LPP1.CUBE
        LPP1.action = LPP1.ADD
        LPP1.lifetime = rospy.Duration(0.5)
        LPP1.id = LPP_count
        LPP1.pose.position.x = msg.points[LPP_count].x
        LPP1.pose.position.y = msg.points[LPP_count].y
        LPP1.pose.position.z = 0
        LPP1.pose.orientation.x = 0.0
        LPP1.pose.orientation.y = 0.0
        LPP1.pose.orientation.z = 0.0
        LPP1.pose.orientation.w = 1.0
        LPP1.scale.x = 0.1
        LPP1.scale.y = 0.1
        LPP1.scale.z = 0.1
        LPP1.color.a = 1.0
        LPP1.color.r = 1.0
        LPP1.color.g = 1.0
        LPP1.color.b = 0.0
    
        LPP_Array.markers.append(LPP1)
    print(msg)

    pub.publish(LPP_Array)
        

    for LPP_count in range (len(LPP_Array.markers)):
        LPP_Array.markers.pop()
        

def fuckGPP(msg):
    GPP_Array = MarkerArray()
    GPP_Array.markers = []

    for GPP_count in range (len(msg.points)):
        
        GPP = Marker()
        GPP.header.frame_id = "world" #+ str(ob_count) # publish path in map frame
        GPP.ns = "GPP_" + str(GPP_count)
        GPP.type = GPP.CUBE
        GPP.action = GPP.ADD
        GPP.lifetime = rospy.Duration(0)
        GPP.id = GPP_count
        GPP.pose.position.x = msg.points[GPP_count].x
        GPP.pose.position.y = msg.points[GPP_count].y
        GPP.pose.position.z = 0
        GPP.pose.orientation.x = 0.0
        GPP.pose.orientation.y = 0.0
        GPP.pose.orientation.z = 0.0
        GPP.pose.orientation.w = 1.0
        GPP.scale.x = 0.1
        GPP.scale.y = 0.1
        GPP.scale.z = 0.1
        GPP.color.a = 1.0
        GPP.color.r = 0.0
        GPP.color.g = 1.0
        GPP.color.b = 0.0
    
        GPP_Array.markers.append(GPP)

    pub_G.publish(GPP_Array)
    #print (msg.points)
    for GPP_count in range (len(GPP_Array.markers)):
        GPP_Array.markers.pop()


def fuckob(msg):
    ob_Array = MarkerArray()
    ob_Array.markers = []
    
    for ob_count in range (len(msg.points)):
        ob = Marker()
        ob.header.frame_id = "world" #+ str(ob_count) # publish path in map frame
        ob.ns = "LPP_" + str(ob_count)
        ob.type = ob.CUBE
        ob.action = ob.ADD
        ob.lifetime = rospy.Duration(0.5)
        ob.id = ob_count
        ob.pose.position.x = msg.points[ob_count].x
        ob.pose.position.y = msg.points[ob_count].y
        ob.pose.position.z = 0
        ob.pose.orientation.x = 0.0
        ob.pose.orientation.y = 0.0
        ob.pose.orientation.z = 0.0
        ob.pose.orientation.w = 1.0
        ob.scale.x = 0.1
        ob.scale.y = 0.1
        ob.scale.z = 0.1
        ob.color.a = 1.0
        ob.color.r = 0.0
        ob.color.g = 0.0
        ob.color.b = 1.0

        ob_Array.markers.append(ob)

    pub_ob.publish(ob_Array)
        

    for ob_count in range (len(ob_Array.markers)):
        ob_Array.markers.pop()

    
    #print (msg.points)



if __name__ == "__main__":
    #plt.plot([6.78591132663 , -6.06284527727],[15.8453863844, -11.4849896419])
    pub_G = rospy.Publisher('/fuck_g', MarkerArray, queue_size = 1, latch = True)
    pub = rospy.Publisher('/fuck', MarkerArray, queue_size = 1)
    pub_ob = rospy.Publisher('/fuck_ob', MarkerArray, queue_size = 1)
    rospy.init_node('fuckingLPP', anonymous=False)
    rospy.Subscriber('/LPP', PointCloud, fuckLPP)
    rospy.Subscriber('/GPP', PointCloud, fuckGPP)
    rospy.Subscriber('/OBB', PointCloud, fuckob)
    rospy.spin()