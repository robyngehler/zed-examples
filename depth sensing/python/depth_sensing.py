########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    This sample demonstrates how to capture a live 3D point cloud   
    with the ZED SDK and display the result in an OpenGL window.    
"""

import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl

if __name__ == "__main__":
    print("Running Depth Sensing sample ... Press 'Esc' to quit")
# perfomance parameters
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.VGA,
                                 camera_fps=100,
                                 depth_mode=sl.DEPTH_MODE.PERFORMANCE,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
                                         
    init.depth_minimum_distance = 0.1
    init.depth_maximum_distance = 20
    init.svo_real_time_mode = True
    
    zed = sl.Camera()
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    
    res = sl.Resolution()
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    camera_model = zed.get_camera_information().camera_model


    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            current_fps = zed.get_current_fps()
            print("current fps: ", current_fps)
            l_image_timestamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
            print("timestamp: ", l_image_timestamp)
            zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH) # Get the depth Matrix
            
            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = image.get_width() / 2
            y = image.get_height() / 2
            point_cloud_value = point_cloud.get_value(x, y)
            distance = math.sqrt(point_cloud_value[0]*point_cloud_value[0] + point_cloud_value[1]*point_cloud_value[1] + point_cloud_value[2]*point_cloud_value[2])
            printf("Distance to Camera at (", x, y, "): ", distance, "mm")
            
            print("duration: ", zed.get_nanoseconds - l_image_timestamp)
            
          

    zed.close()
