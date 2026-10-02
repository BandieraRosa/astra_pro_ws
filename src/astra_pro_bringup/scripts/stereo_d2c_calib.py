#!/usr/bin/env python3
"""D2C stereo calibration: IR <-> color using a checkerboard.

Depth and IR share the same sensor, so the IR<->color extrinsic
doubles as the depth<->color (D2C) extrinsic.

Usage:
  1. enable IR stream (enable_ir=true) and start bringup + ir_converter,
     so /camera/ir/image_mono8 and /camera/color/image_raw are flowing.
  2. capture pairs (SPACE saves when corners found in BOTH, Q quits):
       stereo_d2c_calib.py capture --out /tmp/d2c_pairs
  3. solve with fixed intrinsics from the two camera_info yamls:
       stereo_d2c_calib.py solve --pairs /tmp/d2c_pairs \\
           --ir-yaml <astra_pro_ir.yaml> --color-yaml <astra_pro_color.yaml>
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml


def load_cam_info(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    k = np.array(d['camera_matrix']['data'], dtype=float).reshape(3, 3)
    dist = np.array(d['distortion_coefficients']['data'], dtype=float).reshape(-1)
    size = (int(d['image_width']), int(d['image_height']))
    return k, dist, size


def make_object_points(cols, rows, square):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return objp


def find_board(gray, cols, rows):
    ok, corners = cv2.findChessboardCorners(
        gray, (cols, rows),
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        cv2.cornerSubPix(corners, gray, (11, 11), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return ok, corners


def decode_image(msg):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'mono8':
        return arr.reshape(msg.height, msg.width)
    if msg.encoding == 'rgb8':
        img = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    raise ValueError('unsupported encoding: %s' % msg.encoding)


def cmd_capture(args):
    import time

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    import message_filters

    rclpy.init()
    node = Node('d2c_capture')
    debug_pub = node.create_publisher(Image, '~/debug', 10)
    os.makedirs(args.out, exist_ok=True)
    objp = make_object_points(args.cols, args.rows, args.square)
    saved = [n for n in os.listdir(args.out) if n.startswith('pair_')]
    idx = len(saved)
    last = {'ir': None, 'color': None}
    last_save_t = 0.0

    def diverse(corners, prev):
        if prev is None:
            return True
        return float(np.mean(np.abs(corners.reshape(-1, 2) - prev.reshape(-1, 2)))) > 30.0

    def publish_debug(vis, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'd2c_capture'
        msg.height, msg.width = vis.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = vis.tobytes()
        debug_pub.publish(msg)

    def try_save(c_ir, c_c):
        nonlocal idx, last_save_t
        if not (diverse(c_ir, last['ir']) and diverse(c_c, last['color'])):
            return False
        np.savez(os.path.join(args.out, 'pair_%03d.npz' % idx),
                 ir=c_ir.reshape(-1, 2), color=c_c.reshape(-1, 2))
        last['ir'], last['color'] = c_ir, c_c
        last_save_t = time.monotonic()
        idx += 1
        return True

    def cb(ir_msg, color_msg):
        try:
            ir = decode_image(ir_msg)
            color = decode_image(color_msg)
        except ValueError as e:
            print(e, flush=True)
            return
        ir_g = ir if ir.ndim == 2 else cv2.cvtColor(ir, cv2.COLOR_BGR2GRAY)
        color_g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        ok_ir, c_ir = find_board(ir_g, args.cols, args.rows)
        ok_c, c_c = find_board(color_g, args.cols, args.rows)
        vis_ir = cv2.drawChessboardCorners(cv2.cvtColor(ir_g, cv2.COLOR_GRAY2BGR),
                                           (args.cols, args.rows), c_ir, ok_ir)
        vis_c = cv2.drawChessboardCorners(color.copy(), (args.cols, args.rows), c_c, ok_c)
        status = 'ir:%s color:%s saved:%d%s' % (
            'OK' if ok_ir else '--', 'OK' if ok_c else '--', idx,
            ' AUTO/%ss' % args.auto if args.auto > 0 else ' MANUAL')
        vis = np.hstack([cv2.resize(vis_ir, (640, 480)), cv2.resize(vis_c, (640, 480))])
        cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        publish_debug(vis, ir_msg.header.stamp)
        cv2.imshow('ir | color  (SPACE=save, Q=quit)', vis)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            print('captured %d pairs in %s' % (idx, args.out), flush=True)
            rclpy.shutdown()
            sys.exit(0)
        auto_due = (args.auto > 0 and ok_ir and ok_c
                    and time.monotonic() - last_save_t >= args.auto)
        if (key == ord(' ') and ok_ir and ok_c) or auto_due:
            if try_save(c_ir, c_c):
                print('saved pair %d' % (idx - 1), flush=True)
            elif key == ord(' '):
                print('pose too similar, move board', flush=True)

    sub_ir = message_filters.Subscriber(node, Image, args.ir_topic)
    sub_c = message_filters.Subscriber(node, Image, args.color_topic)
    sync = message_filters.ApproximateTimeSynchronizer([sub_ir, sub_c], 10, 0.15)
    sync.registerCallback(cb)
    print('waiting for synchronized pairs... (need >=15, aim 20-30)', flush=True)
    rclpy.spin(node)


def cmd_solve(args):
    files = sorted(f for f in os.listdir(args.pairs) if f.endswith('.npz'))
    assert len(files) >= 10, 'need >=10 pairs, got %d' % len(files)
    k_ir, d_ir, size_ir = load_cam_info(args.ir_yaml)
    k_c, d_c, size_c = load_cam_info(args.color_yaml)
    assert size_ir == size_c, 'resolution mismatch %s vs %s' % (size_ir, size_c)
    objp = make_object_points(args.cols, args.rows, args.square)
    objpoints, img_ir, img_c = [], [], []
    for f in files:
        z = np.load(os.path.join(args.pairs, f))
        objpoints.append(objp)
        img_ir.append(z['ir'].reshape(-1, 1, 2))
        img_c.append(z['color'].reshape(-1, 1, 2))
    print('solving from %d pairs %s ...' % (len(files), size_ir), flush=True)
    ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objpoints, img_ir, img_c, k_ir, d_ir, k_c, d_c, size_ir,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6))
    print('reprojection RMSE: %.4f px' % ret)
    print('baseline |T|: %.4f m (expect ~0.02-0.04)' % float(np.linalg.norm(T)))
    print('R:\n', R)
    print('T (m):\n', T.reshape(-1))
    # rotation matrix -> quaternion (x, y, z, w) for static_transform_publisher
    tr = np.trace(R)
    qw = np.sqrt(max(0.0, 1.0 + tr)) / 2.0
    qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
    t = T.reshape(-1)
    print('static_transform_publisher args:')
    print('  %.6f %.6f %.6f %.6f %.6f %.6f %.6f camera_depth_optical_frame '
          'camera_color_optical_frame' % (t[0], t[1], t[2], qx, qy, qz, qw))
    np.savez(os.path.join(args.pairs, 'd2c_result.npz'), R=R, T=T.reshape(-1), rmse=ret)


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--out', required=True)
    c.add_argument('--ir-topic', default='/camera/ir/image_mono8')
    c.add_argument('--color-topic', default='/camera/color/image_raw')
    c.add_argument('--cols', type=int, default=10, help='inner corners per row (11 squares -> 10)')
    c.add_argument('--rows', type=int, default=7, help='inner corners per col (8 squares -> 7)')
    c.add_argument('--square', type=float, default=0.02)
    c.add_argument('--auto', type=float, default=0.0,
                   help='auto-save interval seconds when both detected (0=manual SPACE)')
    s = sub.add_parser('solve')
    s.add_argument('--pairs', required=True)
    s.add_argument('--ir-yaml', required=True)
    s.add_argument('--color-yaml', required=True)
    s.add_argument('--cols', type=int, default=10)
    s.add_argument('--rows', type=int, default=7)
    s.add_argument('--square', type=float, default=0.02)
    args = p.parse_args()
    if args.cmd == 'capture':
        cmd_capture(args)
    else:
        cmd_solve(args)


if __name__ == '__main__':
    main()
