import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import yaml


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def capture(args):
    folder = Path(args.output)
    folder.mkdir(parents=True, exist_ok=True)
    camera = cv2.VideoCapture(args.device)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    if not camera.isOpened():
        raise RuntimeError("camera unavailable; close other camera applications and check OS permissions")
    writer = None
    saved = len(list(folder.glob("calibration_*.png")))
    start = time.monotonic()
    print("S: save unmodified calibration image. Q: finish. Recording starts immediately with --record.")
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("camera read failed")
            h, w = frame.shape[:2]
            if args.record and writer is None:
                writer = cv2.VideoWriter(str(folder / "indoor.avi"), cv2.VideoWriter_fourcc(*"MJPG"), args.fps, (w,h))
                if not writer.isOpened():
                    raise RuntimeError("video writer failed")
            if writer is not None:
                writer.write(frame)
            cv2.imshow("DS685 camera - S save calibration; Q finish", frame)
            key = cv2.waitKey(1) & 255
            if key == ord("s"):
                cv2.imwrite(str(folder / f"calibration_{saved:03}.png"), frame)
                saved += 1
                print(f"Saved {saved} calibration images")
            if key == ord("q") or (args.seconds and time.monotonic() - start >= args.seconds):
                break
    finally:
        actual_fps = camera.get(cv2.CAP_PROP_FPS)
        camera.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        write_json(folder / "capture.json", dict(device=args.device, width=w if 'w' in locals() else None,
                   height=h if 'h' in locals() else None, requested_fps=args.fps, reported_fps=actual_fps,
                   elapsed_seconds=time.monotonic()-start, saved_calibration_images=saved,
                   note="AVI uses requested constant fps; check actual motion and frame timing before SLAM"))


def calibrate(folder, board, square_m, output, min_views=12):
    if min(board) < 3 or square_m <= 0:
        raise ValueError("board dimensions and measured square size must be positive")
    objects = np.zeros((board[0]*board[1],3), np.float32)
    objects[:,:2] = np.mgrid[:board[0],:board[1]].T.reshape(-1,2) * square_m
    object_points, image_points, used, rejected = [], [], [], []
    image_size = None
    for path in sorted(Path(folder).glob("*.png")):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            rejected.append(str(path)); continue
        size = image.shape[::-1]
        if image_size is not None and size != image_size:
            raise ValueError("calibration images must all have the same resolution")
        image_size = size
        ok, corners = cv2.findChessboardCornersSB(image, board, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            rejected.append(str(path)); continue
        object_points.append(objects.copy())
        image_points.append(corners.astype(np.float32))
        used.append(str(path))
    if len(used) < min_views:
        raise ValueError(f"only {len(used)} usable chessboards; need {min_views} varied views; board is INNER corners")
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(object_points,image_points,image_size,None,None)
    errors = []
    for obj, img, rv, tv in zip(object_points,image_points,rvecs,tvecs):
        projected, _ = cv2.projectPoints(obj,rv,tv,K,D)
        residuals = projected.reshape(-1, 2) - img.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(residuals**2, axis=1)))))
    all_points = np.concatenate(image_points).reshape(-1,2)
    centers = np.array([p.mean(axis=0).ravel() for p in image_points])
    warnings = []
    if rms > 1.0:
        warnings.append("RMS exceeds 1 pixel; inspect blur, pattern flatness and outlier views")
    span = np.ptp(centers,axis=0) / image_size
    if np.any(span < .25):
        warnings.append("board centers cover less than 25% of an image dimension; capture more edge views")
    report = dict(kind="measured perspective calibration", image_size=list(image_size), K=K.tolist(),
                  distortion=D.ravel().tolist(), rms_px=float(rms), per_view_rms_px=errors,
                  used_images=used, rejected_images=rejected, board_inner_corners=list(board), square_size_m=square_m,
                  board_center_span_fraction=span.tolist(), corner_span_fraction=(np.ptp(all_points,axis=0)/image_size).tolist(),
                  warnings=warnings, model="OpenCV perspective Brown-Conrady, five distortion coefficients")
    write_json(output, report)
    return report


def stella_config(calibration, fps, setup="monocular", depth_factor=1.0):
    if fps <= 0:
        raise ValueError("fps must be measured and positive")
    K, D = np.asarray(calibration["K"],float), np.asarray(calibration["distortion"],float)
    w,h = calibration["image_size"]
    if K.shape != (3,3) or D.size != 5 or not np.isfinite(K).all() or not np.isfinite(D).all() or min(K[0,0],K[1,1]) <= 0:
        raise ValueError("requires valid perspective calibration with five coefficients")
    camera = dict(name="DS685 calibrated perspective camera", setup=setup, model="perspective", cols=int(w), rows=int(h),
                  fps=float(fps), color_order="BGR", fx=float(K[0,0]), fy=float(K[1,1]), cx=float(K[0,2]), cy=float(K[1,2]))
    camera.update(zip(["k1","k2","p1","p2","k3"],map(float,D)))
    if setup == "RGBD":
        # This affects close/far feature selection, not the metric depth scale.
        camera.update(focal_x_baseline=float(K[0,0])*.05, depth_threshold=40.)
    return dict(Camera=camera, Preprocessing=dict(min_size=400, depthmap_factor=float(depth_factor)),
                Feature=dict(name="ORB",scale_factor=1.2,num_levels=8,ini_fast_threshold=20,min_fast_threshold=7),
                Mapping=dict(backend="g2o",baseline_dist_thr_ratio=.02), Tracking=dict(backend="g2o"),
                System=dict(map_format="msgpack"))


def inspect_video(path, max_frames=None, calibration=None, output_video=None, scale=1.0):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise ValueError("cannot open video")
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not np.isfinite(fps) or fps <= 0 or scale <= 0:
        cap.release(); raise ValueError("invalid fps or scale")
    count, blurred, first_size, writer = 0, 0, None, None
    try:
        while max_frames is None or count < max_frames:
            ok, frame = cap.read()
            if not ok:
                break
            size = frame.shape[1::-1]
            if first_size is None:
                first_size = size
                if calibration and list(size) != calibration["image_size"]:
                    raise ValueError("video resolution differs from calibration; do not reuse wrong intrinsics")
            elif size != first_size:
                raise ValueError("video resolution changed")
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            blurred += int(cv2.Laplacian(gray,cv2.CV_64F).var() < 50)
            if output_video:
                if calibration:
                    frame = cv2.undistort(frame,np.array(calibration["K"]),np.array(calibration["distortion"]))
                if scale != 1:
                    frame = cv2.resize(frame, (round(size[0]*scale),round(size[1]*scale)))
                if writer is None:
                    writer = cv2.VideoWriter(str(output_video),cv2.VideoWriter_fourcc(*"MJPG"),fps,frame.shape[1::-1])
                    if not writer.isOpened():
                        raise ValueError("failed to create output video")
                writer.write(frame)
            count += 1
    finally:
        cap.release()
        if writer is not None:
            writer.release()
    if count == 0:
        raise ValueError("video contains no decoded frames")
    report = dict(path=str(path), decoded_frames=count, fps=float(fps), image_size=list(first_size),
                  duration_s=count/fps, blur_fraction=blurred/count, inspected_prefix=max_frames is not None,
                  blur_note="Laplacian variance <50 is a diagnostic heuristic, not a calibration test")
    if output_video and calibration:
        adjusted = dict(calibration)
        K = np.array(calibration["K"])
        new_size = [round(v*scale) for v in first_size]
        # Respect rounding of output dimensions when scaling K.
        K[0] *= new_size[0]/first_size[0]
        K[1] *= new_size[1]/first_size[1]
        adjusted.update(K=K.tolist(), distortion=[0.]*5, image_size=new_size,
                        kind="derived calibration for undistorted and resized video")
        write_json(str(output_video)+".calibration.json",adjusted)
        Path(str(output_video)+".yaml").write_text(yaml.safe_dump(stella_config(adjusted,fps),sort_keys=False))
    return report


def main():
    p = argparse.ArgumentParser(description="Capture, calibrate and validate perspective webcam footage for StellaVSLAM")
    sub = p.add_subparsers(dest="command",required=True)
    a=sub.add_parser("capture"); a.add_argument("--device",type=int,default=0); a.add_argument("--output",required=True)
    a.add_argument("--width",type=int,default=640); a.add_argument("--height",type=int,default=480)
    a.add_argument("--fps",type=float,default=30); a.add_argument("--record",action="store_true"); a.add_argument("--seconds",type=float)
    a=sub.add_parser("calibrate"); a.add_argument("--images",required=True); a.add_argument("--board",nargs=2,type=int,default=[9,6])
    a.add_argument("--square-m",type=float,required=True); a.add_argument("--output",required=True)
    a=sub.add_parser("yaml"); a.add_argument("--calibration",required=True); a.add_argument("--fps",type=float,required=True); a.add_argument("--output",required=True)
    a=sub.add_parser("video"); a.add_argument("--input",required=True); a.add_argument("--report",required=True)
    a.add_argument("--calibration"); a.add_argument("--output-video"); a.add_argument("--scale",type=float,default=1); a.add_argument("--max-frames",type=int)
    args=p.parse_args()
    if args.command == "capture": capture(args)
    elif args.command == "calibrate": print(json.dumps(calibrate(args.images,tuple(args.board),args.square_m,args.output),indent=2))
    elif args.command == "yaml":
        Path(args.output).write_text(yaml.safe_dump(stella_config(json.loads(Path(args.calibration).read_text()),args.fps),sort_keys=False))
    else:
        calibration=json.loads(Path(args.calibration).read_text()) if args.calibration else None
        write_json(args.report,inspect_video(args.input,args.max_frames,calibration,args.output_video,args.scale))


if __name__ == "__main__": main()
