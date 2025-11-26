from flask import Flask, render_template, jsonify, url_for
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from flask import request
import geometry_msgs.msg
from clover import srv
import numpy as np
import threading
import tf2_ros
import logging
import signal
import rospy
import math
import sys
import cv2
import tf

rospy.init_node('pipe_flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
rospy.wait_for_service("/mavros/cmd/command")
cmd_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
bridge = CvBridge()

app = Flask(__name__)


@app.route('/')
def main():
    return render_template('main.html')


@app.route('/update_map')
def update_map():
    return jsonify({"img": url_for('static', filename='map.png')})


@app.route('/update_coordinates')
def coordinates():
    return jsonify(coords + (['None'] * (5 - len(coords))))


@app.route('/status', methods=['POST'])
def stat():
    global status
    status = request.data.decode('utf-8')
    return jsonify({"result": "ok"})


@app.route('/mission_state')
def mission_status():
    return jsonify({"mission": status})


@app.route('/start', methods=['POST'])
def start_mission():
    global mission_accomplishment, coords, status
    status = 'execution'
    coords = []
    mission_accomplishment = True
    return jsonify({"mission_accomplishment": True})


@app.route("/kill_switch", methods=["POST"])
def kill_switch():
    global mission_accomplishment, flag_kill_switch, start_flight, status
    status = 'accident'
    flag_kill_switch = True
    start_flight = False


@app.route("/stop", methods=["POST"])
def stop_mission():
    global mission_accomplishment, start_flight, status
    status = 'stop'
    mission_accomplishment = False
    start_flight = True
    return jsonify({"mission_accomplishment": False})


@app.route("/break", methods=["POST"])
def breack():
    global fl_break, status
    status = 'completed'
    fl_break = True
    return jsonify({"mission_accomplishment": False})


def kill_motors():
    try:
        res = cmd_srv(command=185, param1=1)
        if res.success:
            rospy.loginfo("Motors killed (FT activated)")
        else:
            rospy.logwarn("Kill command failed")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


'''
def restore_motors():
    try:
        res = cmd_srv(command=185, param1=0)
        if res.success:
            rospy.loginfo("Motors restored (FT deactivated)")
        else:
            rospy.logwarn("Restore command failed")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
'''


def mission_running():
    video_writer = None
    is_recording = False

    frame_global = None
    center_x = None
    last_center = None
    frame_width = None
    center_buffer = []
    BUFFER_SIZE = 5

    lost_frames = 0
    max_lost_frames = 15

    current_branch_side = None
    branch_detected = False
    last_branch_side = None
    branch_missing_frames = 0

    count = 0
    branch_coords_printed = False

    lower = np.array([0, 100, 100])
    upper = np.array([10, 255, 255])

    Kp = 2.7
    Ki = 0.09
    Kd = 0.48

    integral_error = 0.0
    prev_error = 0.0
    integral_error_z = 0.0
    prev_error_z = 0.0

    TARGET_HEIGHT = 1.5

    def start_video_recording():
        nonlocal video_writer, is_recording
        try:
            image_msg = rospy.wait_for_message('main_camera/image_raw', Image, timeout=5)
            cv_image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            height, width = cv_image.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(
                'pipe_flight_recording.mp4', fourcc, 30.0, (width, height), isColor=True
            )
            is_recording = True
            print("Started video recording: pipe_flight_recording.mp4")
        except Exception as e:
            print(f"Error starting video recording: {e}")

    def stop_video_recording():
        nonlocal video_writer, is_recording
        if video_writer is not None:
            video_writer.release()
            is_recording = False
            print("Stopped video recording")

    def image_callback(msg):
        nonlocal frame_global, center_x, last_center, frame_width, current_branch_side

        try:
            frame_global = bridge.imgmsg_to_cv2(msg, 'bgr8')

            if is_recording and video_writer is not None:
                video_writer.write(frame_global)

            if frame_width is None:
                frame_width = frame_global.shape[1]

            hsv = cv2.cvtColor(frame_global, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)

            mask_roi = mask[frame_global.shape[0] // 2 - 20: frame_global.shape[0] // 2 + 20, :]
            mask_left = mask_roi[:, 55: mask_roi.shape[1] // 2]
            mask_right = mask_roi[:, mask_roi.shape[1] // 2: mask_roi.shape[1] - 55]

            current_branch_side = None

            left_contour_area = 0
            right_contour_area = 0

            contours_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_left:
                x, y, w, h = cv2.boundingRect(contour)
                fill_ratio = cv2.countNonZero(mask_left) / (mask_left.shape[0] * mask_left.shape[1])
                width_ratio = w / mask_left.shape[1]
                area = cv2.contourArea(contour)

                if fill_ratio >= 0.12 and width_ratio > 0.14 and area > left_contour_area:
                    left_contour_area = area

            contours_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours_right:
                x, y, w, h = cv2.boundingRect(contour)
                fill_ratio = cv2.countNonZero(mask_right) / (mask_right.shape[0] * mask_right.shape[1])
                width_ratio = w / mask_right.shape[1]
                area = cv2.contourArea(contour)

                if fill_ratio >= 0.12 and width_ratio > 0.14 and area > right_contour_area:
                    right_contour_area = area

            if left_contour_area > 0 and right_contour_area > 0:
                if left_contour_area > right_contour_area * 1.5:
                    current_branch_side = 'left'
                elif right_contour_area > left_contour_area * 1.5:
                    current_branch_side = 'right'
            elif left_contour_area > 0:
                current_branch_side = 'left'
            elif right_contour_area > 0:
                current_branch_side = 'right'

            vertical_projection = np.sum(mask_roi, axis=0)

            if np.max(vertical_projection) > 0:
                max_indices = np.where(vertical_projection == np.max(vertical_projection))[0]
                cx = int(np.mean(max_indices))
                last_center = cx
            else:
                if last_center is None:
                    last_center = frame_width // 2
                cx = last_center

            center_buffer.append(cx)
            if len(center_buffer) > BUFFER_SIZE:
                center_buffer.pop(0)
            center_x = int(np.mean(center_buffer))

        except Exception as e:
            print(f"Error in image callback: {e}")

    def update_pid(error, integral, prev, kp, ki, kd):
        integral += error
        integral = max(min(integral, 1.0), -1.0)
        derivative = error - prev
        output = kp * error + ki * integral + kd * derivative
        output = max(min(output, 1.5), -1.5)
        return output, integral, derivative

    rospy.Subscriber('main_camera/image_raw', Image, image_callback)

    try:
        rospy.wait_for_message('main_camera/image_raw', Image, timeout=5)
        start_video_recording()
    except rospy.ROSException:
        print("Camera not available")

    navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
    rospy.sleep(4)

    navigate(x=1, y=1, z=1.5, yaw=math.radians(45), frame_id='aruco_map')
    rospy.sleep(7)

    rate = rospy.Rate(20)

    global mission_accomplishment, start_flight, flag_kill_switch, coords

    x_old, y_old = 1.0, 1.0

    try:
        while not rospy.is_shutdown():

            if not mission_accomplishment:
                navigate(x=0, y=0, z=1.5, speed=2.0, yaw=math.radians(45), frame_id='aruco_map')
                rospy.sleep(15)
                land()
                start_flight = True
                break

            if fl_break:
                mission_accomplishment = False
                break

            if flag_kill_switch:
                kill_motors()
                mission_accomplishment = False
                break

            if frame_global is None or center_x is None or frame_width is None:
                rate.sleep()
                continue

            try:
                telemetry = get_telemetry(frame_id='aruco_map')

                error_z = TARGET_HEIGHT - telemetry.z
                if abs(error_z) < 0.01:
                    error_z = 0

                vz_cmd, integral_error_z, _ = update_pid(error_z, integral_error_z, prev_error_z, Kp,
                                                         Ki, Kd)

                prev_error_z = error_z

            except:
                vz_cmd = 0

            x, y = telemetry.x, telemetry.y
            img = cv2.imread('static/map.png')

            pts = np.array([
                [
                    50 + int((x - 0.1 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80),
                    770 - int((y + 0.1 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80)
                ],
                [
                    50 + int((x + 0.1 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80),
                    770 - int((y - 0.1 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80)
                ],
                [
                    50 + int((x_old + 0.1 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80),
                    770 - int((y_old - 0.1 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80)
                ],
                [
                    50 + int((x_old - 0.1 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80),
                    770 - int((y_old + 0.1 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80)
                ]
            ], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.fillPoly(img, [pts], (0, 0, 255))

            if current_branch_side and not branch_detected:
                branch_detected = True
                last_branch_side = current_branch_side
                branch_missing_frames = 0
                branch_coords_printed = False

            elif current_branch_side and branch_detected and not branch_coords_printed:
                branch_missing_frames = 0

                hsv = cv2.cvtColor(frame_global, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                mask_roi = mask[frame_global.shape[0] // 2 - 20: frame_global.shape[0] // 2 + 20,
                           50: frame_global.shape[1] - 50]

                horizontal_projection = np.sum(mask_roi, axis=1)
                if np.max(horizontal_projection) > 0:
                    branch_center_y = np.argmax(horizontal_projection)
                    roi_height = mask_roi.shape[0]
                    if abs(branch_center_y - roi_height // 2) < roi_height * 0.05:
                        coords.append({'x': float(f'{x:.3f}'), 'y': float(f'{y:.3f}')})
                        if current_branch_side == 'left':
                            x_1 = x + 0.7 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)
                            y_1 = y - 0.7 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)
                        else:
                            x_1 = x - 0.7 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)
                            y_1 = y + 0.7 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)

                        pts_1 = np.array([
                            [
                                50 + (x_1 - 0.05 * ((y - y_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80,
                                770 - (y_1 + 0.05 * ((x - x_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80
                            ],
                            [
                                50 + (x_1 + 0.05 * ((y - y_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80,
                                770 - (y_1 - 0.05 * ((x - x_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80
                            ],
                            [
                                50 + (x + 0.05 * ((y - y_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80,
                                770 - (y - 0.05 * ((x - x_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80
                            ],
                            [
                                50 + (x - 0.05 * ((y - y_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80,
                                770 - (y + 0.05 * ((x - x_1) / ((x - x_1) ** 2 + (y - y_1) ** 2) ** 0.5)) * 80
                            ]
                        ], np.int32)
                        pts_1 = pts_1.reshape((-1, 1, 2))
                        cv2.fillPoly(img, [pts_1], (0, 0, 255))

                        x_2 = 50 + int((x - 0.1 * ((y_old - y) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80)
                        y_2 = 770 - int((y + 0.1 * ((x_old - x) / ((x_old - x) ** 2 + (y_old - y) ** 2) ** 0.5)) * 80)

                        cv2.circle(img, (x_2, y_2), 4, -1)

                        print(
                            f"New subpipe: {current_branch_side}, coords_start:=({telemetry.x:.2f}, {telemetry.y:.2f}, {telemetry.z:.2f})")
                        count += 1
                        branch_coords_printed = True

            elif not current_branch_side and branch_detected:
                branch_missing_frames += 1
                if branch_missing_frames >= 12:
                    branch_detected = False
                    last_branch_side = None
                    branch_missing_frames = 0

            cv2.imwrite('static/map.png', img)
            x_old, y_old = x, y

            error_y = (center_x - frame_width // 2) / frame_width
            if abs(error_y) < 0.01:
                error_y = 0

            mask_val = np.max(
                cv2.inRange(cv2.cvtColor(frame_global, cv2.COLOR_BGR2HSV), lower, upper)
            )
            lost_frames = lost_frames + 1 if mask_val == 0 else 0

            if lost_frames > max_lost_frames or count == 5:
                print("End pipe")
                navigate(x=0, y=0, z=1.5, speed=2.0, yaw=math.radians(45), frame_id='aruco_map')
                rospy.sleep(15)
                land()
                break

            vy_cmd, integral_error, _ = update_pid(-error_y, integral_error, prev_error, Kp, Ki, Kd)
            prev_error = error_y

            set_velocity(vx=0.07, vy=vy_cmd, vz=vz_cmd, frame_id='body')
            rate.sleep()

    finally:
        mission_accomplishment = False
        start_flight = True
        stop_video_recording()
        if flag_kill_switch or fl_break:
            sys.exit()
        global status
        status = 'expectation'


def run_flask():
    app.run(host='0.0.0.0', port=5070)


def signal_handler(sig, frame):
    print("Exiting...")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

log = logging.getLogger('werkzeug')
log.disabled = True

mission_accomplishment = False
flag_kill_switch = False
fl_break = False
status = 'expectation'
start_flight = True
coords = []

flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

img = cv2.imread('static/map_base.png')
cv2.imwrite('static/map.png', img)

print("Waiting for start signal from server...")
while True:
    if not mission_accomplishment:
        continue
    elif start_flight:
        start_flight = False
        print('START')
        img = cv2.imread('static/map_base.png')
        cv2.imwrite('static/map.png', img)
        mission_running()
