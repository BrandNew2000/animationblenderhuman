import bpy
import cv2
import time
import numpy

import mediapipe as mp


class OpenCVAnimOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.opencv_operator"
    bl_label = "OpenCV Animation Operator"
    
    # Initialize mediapipe
    mp_hands = mp.solutions.hands
    mp_pose=mp.solutions.pose
    hands = mp_hands.Hands()
    pose = mp_pose.Pose()
    
    _timer = None
    _cap  = None
    stop = False
    human=None
  
    
    # Webcam resolution:
    width = 640
    height = 480
    normalisationpos=(0,0,0)
    
    
    def getpose(self, pos, results_pose):
        landmarks=results_pose.pose_landmarks
        point=landmarks.landmark[pos]
        x, y, z = (point.x), (point.y), (point.z)
        return x,y,z


    def rotation(self, x,y, angle):
        angle=angle*numpy.pi/180
        
        return x*numpy.cos(angle)+y*numpy.sin(angle), -x*numpy.sin(angle)+y*numpy.cos(angle)
    
    
    def resetbone(self, bones, name):
        bones[name].location=(0,0,0)
        bones[name].keyframe_insert(data_path="location", index=-1)
    
    
    
    def reset(self, bones):
        
        bone_list=["hand_ik_ctrl_L","hand_ik_ctrl_R","sole_ctrl_L","sole_ctrl_R","shoulder_L"]
        
        for bone_item in bone_list:
            self.resetbone(bones, bone_item)
    
    # The main "loop"
    def modal(self, context, event):
        bones = bpy.data.objects["RIG-Vincent"].pose.bones

        if (event.type in {'RIGHTMOUSE', 'ESC'}) or self.stop == True:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            self.init_camera()
            _, image = self._cap.read()
            #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            #gray = cv2.equalizeHist(gray)
            
            
            rgb_frame = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Process the frame with mediapipe hands    
                
            #results_hands = self.hands.process(rgb_frame)
            results_pose = self.pose.process(rgb_frame)
            
            




            try:

                
                a1,b1,c1=self.getpose(11, results_pose)
                a2,b2,c2=self.getpose(12, results_pose)
                a,b,c=(a1+a2)/2,(b1+b2)/2,(c1+c2)/2
                #normalisationpos=(a-0.2,-b,1-c)
                normalisationpos=(a,b,c)
                
                
                
                # ARMS
                
                a,b,c = self.getpose(15, results_pose)
                #location=(a,-b,1-c)
                location=(a,b,c)
                
                
                location_relative=(location[0]-normalisationpos[0],location[1]-normalisationpos[1],location[2]-normalisationpos[2])
                a,b = self.rotation(location_relative[0], location_relative[1],38)
                c= location_relative[2]
                location_relative=(a,b,c)
                #location_converted=(5*(location[0]-normalisationpos[0]),5,-5*(location[1]-normalisationpos[1]))
                location_converted=(0.2*(location_relative[2]),1*(location_relative[0])-0.55,1*(location_relative[1]))
                bones["hand_ik_ctrl_L"].location=location_converted
                bones["hand_ik_ctrl_L"].keyframe_insert(data_path="location", index=-1)
                
                
                
                
                a,b,c = self.getpose(16, results_pose)
                location=(a,b,c)
                
                '''
                location_relative=(location[0]-normalisationpos[0],location[1]-normalisationpos[1],location[2]-normalisationpos[2])
                a,b = self.rotation(location_relative[0], location_relative[1],-218)
                c= location_relative[2]
                location_relative=(a,b,c)
                
                #location_converted=(5*(location[0]-normalisationpos[0]),5,-5*(location[1]-normalisationpos[1]))
                location_converted=(-0.2*(location_relative[2]),-1*(location_relative[0])+0.45,-2*(location_relative[1]))
                '''
                location_relative=(location[0]-normalisationpos[0],location[1]-normalisationpos[1],location[2]-normalisationpos[2])
                a,b = self.rotation(location_relative[0], -location_relative[1],38)
                c= location_relative[2]
                location_relative=(a,b,c)
                location_converted=(-0.2*(location_relative[2]),-1*(location_relative[0])-0.53,-1*(location_relative[1]-0.07))
                
                
                bones["hand_ik_ctrl_R"].location=location_converted
                bones["hand_ik_ctrl_R"].keyframe_insert(data_path="location", index=-1)
                #location_converted=(-0.2*(location[2]-normalisationpos[2]),-1*(location[0]-normalisationpos[0])-0.55,1*(location[1]-normalisationpos[1])-0.25)
                #bones["hand_ik_ctrl_R"].location=location_converted
                #bones["hand_ik_ctrl_R"].keyframe_insert(data_path="location", index=-1)
            
                
                
                
                # LEGS
                
                a,b,c = self.getpose(29, results_pose)
                #location=(a,-b,1-c)
                location=(a,b,c)
                location_converted=(-1*(location[0]-normalisationpos[0]),-1*(location[2]-normalisationpos[2]),-1*(location[1]-normalisationpos[1]))
                bones["sole_ctrl_L"].location=location_converted
                bones["sole_ctrl_L"].keyframe_insert(data_path="location", index=-1)
                
                
                a,b,c = self.getpose(30, results_pose)
                #location=(a,-b,1-c)
                location=(a,b,c)
                location_converted=(-1*(location[0]-normalisationpos[0]),-1*(location[2]-normalisationpos[2]),-1*(location[1]-normalisationpos[1]))
                bones["sole_ctrl_R"].location=location_converted
                bones["sole_ctrl_R"].keyframe_insert(data_path="location", index=-1)
                
                
                
                #SHOULDER
                
                a,b,c = self.getpose(11, results_pose)
                location=(a,b,c)
                location_relative=(location[0]-normalisationpos[0],location[1]-normalisationpos[1],location[2]-normalisationpos[2])
                a,b = self.rotation(location_relative[0], -location_relative[1],-38)
                c= location_relative[2]
                location_relative=(a,b,c)
                location_converted=(-0.5*(location_relative[1])+0.05,0.5*(location_relative[0])-0.10,-0.2*(location_relative[2]))
                bones["shoulder_L"].location=location_converted
                bones["shoulder_L"].keyframe_insert(data_path="location", index=-1)
                
                
                a,b,c = self.getpose(12, results_pose)
                location=(a,b,c)
                location_relative=(location[0]-normalisationpos[0],location[1]-normalisationpos[1],location[2]-normalisationpos[2])
                a,b = self.rotation(location_relative[0], -location_relative[1],38)
                c= location_relative[2]
                location_relative=(a,b,c)
                location_converted=(0.5*(location_relative[1])-0.05,-0.5*(location_relative[0])-0.10,-0.2*(location_relative[2]))
                bones["shoulder_R"].location=location_converted
                bones["shoulder_R"].keyframe_insert(data_path="location", index=-1)
                
                
                

                
                
                
                
        
                if self.human==False:
                    print("Human found")
                
                self.human=True
                
                
                #movement = tuple(map(lambda i, j: i - j, location_converted, currentpos))
                #0*5*(location[2]-normalisationpos[2])

        
                #bpy.ops.transform.translate(value=movement)
            except:
                
                if self.human==True:
                    print("No humans found.")
                    self.reset(bones)
                self.human=False


            # Show camera image in a window                     
            cv2.imshow("Output", image)
            cv2.waitKey(1)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                
                self.reset(bones)
                self.cancel(context)
                return {'CANCELLED'}
                
            

        return {'PASS_THROUGH'}
    
    def init_camera(self):
        if self._cap == None:
            self._cap = cv2.VideoCapture(0)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            time.sleep(1.0)
    
    def stop_playback(self, scene):
        print(format(scene.frame_current) + " / " + format(scene.frame_end))
        if scene.frame_current == scene.frame_end:
            bpy.ops.screen.animation_cancel(restore_frame=False)
        
    def execute(self, context):
        bpy.app.handlers.frame_change_pre.append(self.stop_playback)

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.01, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)
        cv2.destroyAllWindows()
        self._cap.release()
        self._cap = None

def register():
    bpy.utils.register_class(OpenCVAnimOperator)

def unregister():
    bpy.utils.unregister_class(OpenCVAnimOperator)

if __name__ == "__main__":
    register()

    # test call
    #bpy.ops.wm.opencv_operator()



