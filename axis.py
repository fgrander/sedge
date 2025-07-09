#!/usr/bin/python
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#

import sys
print(sys.executable)

import threading
#import urllib.request, urllib.error, urllib.parse
import urllib
import requests #, requests.auth
import datetime
import time

import cv2
import numpy as np
from PIL import Image
from io import BytesIO

import subprocess as sp

#import rospy
#from sensor_msgs.msg import CompressedImage, CameraInfo
#from std_msgs.msg import Bool
#from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
#import camera_info_manager

class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5
        self.analyse = EggerAnalyse()

        self.ffmpeg_sizeStr = str(int(self.axis.width)) + 'x' + str(int(self.axis.height))
        self.ffmpeg_fps = int(self.axis.fps)
        self.ffmpeg_fps = 15
        self.rtsp_server = 'rtsp://192.168.0.10:8554/mystream'
        #command = ['C://ffmpeg-7.1.1-full//bin//ffmpeg.exe',

        command = ['/usr/bin/ffmpeg',
                   '-re',
                   '-s', self.ffmpeg_sizeStr,
                   '-r', str(self.ffmpeg_fps),  # rtsp fps (from input server)
                   '-i', '-',

                   # You can change ffmpeg parameter after this item.
                   '-pix_fmt', 'yuv420p',
                   '-r', '30',  # output fps
                   '-g', '50',
                   '-c:v', 'libx264',
                   '-b:v', '2M',
                   '-bufsize', '64M',
                   '-maxrate', "4M",
                   '-preset', 'veryfast',
                   '-rtsp_transport', 'tcp',
                   '-segment_times', '5',
                   '-f', 'rtsp',
                   self.rtsp_server]

        self.process = sp.Popen(command, stdin=sp.PIPE)

    def run(self):
        while(True):
            self.stream()

    def stream(self):
        while(True):
            self.formURL()
            self.authenticate()
            if self.openURL():
                self.publishFramesContinuously()

            #rospy.sleep(2) # if stream stays intact we shouldn't get to this

    def formURL(self):
        self.url = 'http://%s/mjpg/video.mjpg' % self.axis.hostname
        self.url += "?fps=%d&resolution=%dx%d" % (self.axis.fps, self.axis.width,
                                                            self.axis.height)

        # support for Axis F34 multicamera switch
        if (self.axis.camera != 0):
            self.url += "&camera=%s" % str(self.axis.camera)

        print('opening ' + str(self.axis))
        #rospy.logdebug('opening ' + str(self.axis))

    def authenticate(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            top_level_url = "http://" + self.axis.hostname
            password_mgr.add_password(None, top_level_url, self.axis.username,
                                                            self.axis.password)
            if self.axis.use_encrypted_password:
                handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib.request.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib.request.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib.request.install_opener(opener)

    def openURL(self):
        '''Open connection to Axis camera using http'''
        try:
            self.fp = urllib.request.urlopen(self.url, timeout=self.timeoutSeconds)
            return(True)
        except urllib.error.URLError as e:
            #rospy.logwarn('Error opening URL %s' % (self.url) +
            #                'Possible timeout.  Looping until camera appears')
            print('Error opening URL %s' % (self.url) +
                          'Possible timeout.  Looping until camera appears')

            return(False)

    def publishFramesContinuously(self):
        '''Continuous loop to publish images'''
        while(True):
            try:
                self.findBoundary()
                self.getImage()
                #self.publishMsg()
                #self.publishCameraInfoMsg()
                #print('image received')
                self.img_pil = Image.open(BytesIO(self.img))
                self.img_mat = np.array(self.img_pil)
                self.img_mat = self.img_mat[:, ::-1, ::-1].copy()
                #cv2.imshow('axis', self.img_mat)
                #cv2.imwrite('axis.jpg', self.img)
                frame = self.analyse.run(self.img_mat)

                #cv2.waitKey(1)

                self.process.stdin.write(frame.tobytes())

            except:
                #rospy.logwarn('Timed out while trying to get message.')
                print('Timed out while trying to get message.')
                break

    def findBoundary(self):
        '''The string "--myboundary" is used to denote the start of an image in
        Axis cameras'''
        while(True):
            boundary = self.fp.readline()
            if boundary == b'--myboundary\r\n':
                break

    def getImage(self):
        '''Get the image header and image itself'''
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = {}
        while(True):
            line = self.fp.readline()
            if line == b'\r\n':
                break
            line = line.strip()
            parts = line.split(b": ", 1)
            try:
                self.header[parts[0]] = parts[1]
            except:
                #rospy.logwarn('Problem encountered with image header.  Setting '
                #                                    'content_length to zero')
                print('Problem encountered with image header.  Setting '
                              'content_length to zero')

                self.header[b'Content-Length'] = 0 # set content_length to zero if
                                            # there is a problem reading header
        self.content_length = int(self.header[b'Content-Length'])

    def getImageData(self):
        '''Get the binary image data itself (ie. without header)'''
        if self.content_length>0:
            self.img = self.fp.read(self.content_length)
            self.fp.readline() # Read terminating \r\n and do nothing with it

    #def publishMsg(self):
    #    '''Publish jpeg image as a ROS message'''
    #    self.msg = CompressedImage()
    #    self.msg.header.stamp = rospy.Time.now()
    #    self.msg.header.frame_id = self.axis.frame_id
    #    self.msg.format = "jpeg"
    #    self.msg.data = self.img
    #    self.axis.pub.publish(self.msg)

    #def publishCameraInfoMsg(self):
    #    '''Publish camera info manager message'''
    #    cimsg = self.axis.cinfo.getCameraInfo()
    #    cimsg.header.stamp = self.msg.header.stamp
    #    cimsg.header.frame_id = self.axis.frame_id
    #    cimsg.width = self.axis.width
    #    cimsg.height = self.axis.height
    #    self.axis.caminfo_pub.publish(cimsg)

class EggerAnalyse:
    def __init__(self):
        self.position = 0

    def run(self, frame):
        debug = False

        img_in = frame
        width = img_in.shape[1] // 1
        height = img_in.shape[0] // 1
        dim = (width, height)
        img_in = cv2.resize(img_in, dim)
        img_sw = cv2.cvtColor(img_in, cv2.COLOR_BGR2GRAY)
        if debug:
            cv2.imshow('input_sw', img_sw)

        mask = np.zeros(img_sw.shape[:2], dtype="uint8")
        center = (img_sw.shape[1] // 2, img_sw.shape[0] // 2)
        radius = height // 2 + height // 10
        cv2.circle(img=mask, center=center, radius=radius, color=255, thickness=-1)
        masked = cv2.bitwise_and(img_sw, img_sw, mask=mask)

        if debug:
            cv2.imshow("circle Mask", mask)
            cv2.imshow("Mask Applied to Image", masked)

        mask_sum = np.sum(mask / 255, axis=1)  # number of white pixels in each line
        line_sum = np.sum(masked, axis=1)  # total brightness value of each line for the masked fire image
        line_sum_norm = line_sum / mask_sum  # normalize total brightness by the number of pixels
        line_sum_norm_max = np.max(line_sum_norm)
        line_sum_norm_min = np.min(line_sum_norm)
        line_sum_norm_delta = line_sum_norm_max - line_sum_norm_min  # no used atm
        fire_line_brightness_th = int(
            line_sum_norm_min + line_sum_norm_delta / 2)  # threshold to detect the start of the fire

        # start at the bottom of the image and check, if the brightness value is higher as the threshold
        line_sum_norm_min_idx = np.argmin(line_sum_norm)
        for fire_line_idx in reversed(range(0, line_sum_norm_min_idx)):
            if line_sum_norm[fire_line_idx] > fire_line_brightness_th:
                break

        # draw horizontal fire line
        img_line = np.copy(img_in)
        img_line = cv2.line(img_line, (100, fire_line_idx), (100 + width // 4, fire_line_idx), (0, 255, 0), 2)

        # draw brightness chart on the left
        brightness_chart_width = 100  # width of the chart in pixel

        for pix_y in range(height):
            pix_x = int(brightness_chart_width / line_sum_norm_max * line_sum_norm[pix_y])
            img_line[pix_y, pix_x, :] = 255  # white chart
            img_line[pix_y, pix_x] = [255, 255, 0]  # color chart

        # draw vertical line for threshold
        fire_line_brightness_th_pt1 = (int(fire_line_brightness_th * brightness_chart_width / line_sum_norm_max), 0)
        fire_line_brightness_th_pt2 = (int(fire_line_brightness_th * brightness_chart_width / line_sum_norm_max),
                                       height)

        img_line = cv2.line(img_line, fire_line_brightness_th_pt1, fire_line_brightness_th_pt2, (0, 0, 255), 1)
        # write image
        #cv2.imshow('img', img_line)
        # cv2.imwrite(+file_name, img_in)

        if False:
            img_out = np.zeros((img_in.shape[0] * 2, img_in.shape[1] * 2, img_in.shape[2]), dtype=np.uint8)
            img_out[0:img_in.shape[0], 0:img_in.shape[1], :] = img_in
            img_out[0:img_in.shape[0], img_in.shape[1]::, :] = cv2.cvtColor(img_sw, cv2.COLOR_GRAY2BGR)
            img_out[img_in.shape[0]::, 0:img_in.shape[1], :] = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
            img_out[img_in.shape[0]::, img_in.shape[1]::, :] = img_line
            cv2.imshow('out', img_out)

        self.position = fire_line_idx
        ret2, frame2 = cv2.imencode('.png', img_line)
        return frame2

class Axis:
    def __init__(self, hostname, username, password, width, height, fps, frame_id,
                 camera_info_url, use_encrypted_password, camera, ir, defog, wiper):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        self.use_encrypted_password = use_encrypted_password
        self.camera = camera

        self.http_headers = {'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36','From': 'http://'+self.hostname}
        if self.use_encrypted_password:
            self.http_auth = requests.auth.HTTPDigestAuth(self.username, self.password)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(self.username, self.password)
        self.http_timeout = (3, 5)

        # generate a valid camera name based on the hostname
        #self.cname = camera_info_manager.genCameraName(self.hostname)
        #self.cinfo = camera_info_manager.CameraInfoManager(cname = self.cname,
        #                                           url = self.camera_info_url)
        #self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.st = None
        #self.pub = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=1)
        #self.caminfo_pub = rospy.Publisher("camera_info", CameraInfo, self, queue_size=1)

        # The Axis Q62 series supports a night-vision mode with an active IR illuminator
        # If this option is enabled, add the necessary services and topics
        if ir:
            self.ir_on = False
            #self.ir_on_off_srv = rospy.Service('set_ir_on', SetBool, self.handle_toggle_ir)
            #self.ir_on_pub = rospy.Publisher('ir_on', Bool, queue_size=1)
            self.ir_on_pub_thread = threading.Thread(target=self.ir_on_pub_thread_fn)
            self.ir_on_pub_thread.start()

            #self.handle_toggle_ir(SetBoolRequest(False))

        # The Axis Q62 series is equipped with a wiper on the camera lens
        # If this option is enabled, add the necessary services and topics
        if wiper:
            self.wiper_on_time = datetime.datetime.utcnow()
            self.wiper_on = False
            #self.wiper_on_off_srv = rospy.Service('set_wiper_on', SetBool, self.handle_toggle_wiper)
            #self.wiper_on_pub = rospy.Publisher('wiper_on', Bool, queue_size=1)
            self.wiper_on_pub_thread = threading.Thread(target=self.wiper_on_pub_thread_fn)
            self.wiper_on_pub_thread.start()

            #self.handle_toggle_wiper(SetBoolRequest(False))

        # The Axis Q62 series is equipped with a defogger
        # If this option is enabled, add the necessary services and topics
        if defog:
            self.defog_on = False
            #self.defog_on_off_srv = rospy.Service('set_defog_on', SetBool, self.handle_toggle_defog)
            #self.defog_on_pub = rospy.Publisher('defog_on', Bool, queue_size=1)
            self.defog_on_pub_thread = threading.Thread(target=self.defog_on_pub_thread_fn)
            self.defog_on_pub_thread.start()

            #self.handle_toggle_defog(SetBoolRequest(False))

        self.peer_subscribe()

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def peer_subscribe(self):
        '''Lazy-start the image-publisher.'''
        if self.st is None:
            self.st = StreamThread(self)
            self.st.start()


def main():
    #rospy.init_node("axis_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',       # default IP address
        'username': 'fgrander',               # default login name
        'password': 'fgrander',
        'width': 640,
        'height': 480,
        'fps': 0,                         # frames per second (0 = camera default)
        'frame_id': 'axis_camera',
        'camera_info_url': '',
        'use_encrypted_password' : True,
        'camera' : 0,
        'ir': False,
        'defog': False,
        'wiper': False }
    args = arg_defaults
    Axis(**args)

    while(True):
        cv2.waitKey(1)



if __name__ == "__main__":
    main()