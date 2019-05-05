# spawn models, change position of objects and also delete models in Gazebo
import rospy
from gazebo_msgs.srv import GetModelState,SetModelState
from gazebo_msgs.msg import ModelState
class GazeboObj:
    def __init__(self):
        print "initialized obj"
        
    def get_obj_state(self,obj_name):
        rospy.wait_for_service('gazebo/get_model_state')
        model_srv=rospy.ServiceProxy('gazebo/get_model_state',GetModelState,)
        resp=model_srv(model_name=obj_name)
        #print resp.pose
        return resp.pose

    def move_obj(self,obj_name,pose):
        obj_model=ModelState()
        obj_model.model_name=obj_name
        obj_model.pose=pose
        obj_model.reference_frame='world'
        rospy.wait_for_service('gazebo/set_model_state')
        model_srv=rospy.ServiceProxy('gazebo/set_model_state',SetModelState,)
        resp=model_srv(model_state=obj_model)
        #print resp

if __name__=='__main__':
    gz_obj=GazeboObj()
    obj_init_pose=gz_obj.get_obj_state('unit_box')
    raw_input("move_obj?")
    gz_obj.move_obj('unit_box',obj_init_pose)
