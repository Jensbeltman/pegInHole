import open3d.open3d as o3d
from intelRealsenseCamera import CameraRGBD
class Visualizer(o3d.visualization.VisualizerWithKeyCallback): 
    def __init__(self,camera):
        super().__init__()
        self.vis_frame_count = 0
        # key_action_callback will be triggered when there's a keyboard press, release or repeat event
        self.register_key_callback(32,self.key_action_callback)# space
        self.create_window()
        self.run()
        
    @classmethod    
    def key_action_callback(vis, action, mods):
        print(action)
        if action == 1:  # key down
            print('key down')
        elif action == 0:  # key up
            print('key up')
        elif action == 2:  # key repeat
            exit()
        return True

    def visPCD(self,pcd):
        if self.vis_frame_count == 0:
            self.add_geometry(pcd)

        self.update_geometry(pcd)
        self.poll_events()
        self.update_renderer()

        self.vis_frame_count += 1