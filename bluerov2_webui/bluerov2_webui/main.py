import rclpy
import threading
from nicegui import ui

# --- UPDATED IMPORTS ---
from bluerov2_webui.ros_interface import ROVNode
from bluerov2_webui.ui_layout import build_interface

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    rov_node = ROVNode()

    # Build the UI
    build_interface(rov_node)

    # ROS Spin Thread
    def ros_spin_thread():
        try:
            rclpy.spin(rov_node)
        except Exception as e:
            pass
    
    t = threading.Thread(target=ros_spin_thread, daemon=True)
    t.start()

    try:
        ui.run(
            title='ROV Cockpit', 
            port=8080, 
            dark=True, 
            reload=False, 
            show=False
        )
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                rov_node.destroy_node()
                rclpy.shutdown()
            except:
                pass

if __name__ == '__main__':
    main()