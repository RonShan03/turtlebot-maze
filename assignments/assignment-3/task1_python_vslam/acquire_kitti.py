"""Extract a prefix from an official local ZIP, or use HTTP ranges for that prefix.

No unofficial mirrors; never fetch the entire 22 GB image archive implicitly.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
from urllib.request import urlopen
from zipfile import ZipFile

OFFICIAL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/"


def extract_prefix(archive, output, sequence, frames):
    root = Path(output).resolve()
    written = []
    for name in archive.namelist():
        p = PurePosixPath(name)
        parts = p.parts
        if len(parts) >= 3 and parts[-3:-1] == ("poses", sequence):
            continue
        wanted = (name == f"dataset/poses/{sequence}.txt" or name == f"poses/{sequence}.txt"
                  or name.endswith(f"sequences/{sequence}/calib.txt")
                  or name.endswith(f"sequences/{sequence}/times.txt")
                  or (name.endswith(".png") and f"sequences/{sequence}/image_0/" in name and p.stem.isdigit() and int(p.stem) < frames))
        if not wanted:
            continue
        if ".." in parts or p.is_absolute():
            raise ValueError("unsafe archive path")
        relative = PurePosixPath(*parts[parts.index("sequences"):]) if "sequences" in parts else PurePosixPath("poses", p.name)
        destination = root.joinpath(*relative.parts)
        if not destination.resolve().is_relative_to(root):
            raise ValueError("archive path escaped output")
        payload = archive.read(name)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
        written.append({"path": str(relative), "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()})
    return written


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--sequence", default="00", choices=[f"{i:02}" for i in range(11)])
    parser.add_argument("--frames", type=int, default=150)
    parser.add_argument("--images-zip", help="official downloaded grayscale archive; no network if supplied")
    parser.add_argument("--poses-zip")
    parser.add_argument("--remote", action="store_true", help="HTTP range reads from official S3 archive (requires remotezip)")
    args = parser.parse_args()
    if args.frames < 2 or not (args.images_zip or args.remote):
        parser.error("specify --images-zip or --remote, and at least two frames")
    output = Path(args.output)
    if (output / "sequences" / args.sequence / "image_0").exists():
        parser.error("use a new output directory to avoid mixing sequences")
    manifest = dict(source="official KITTI odometry", sequence=args.sequence, frames_requested=args.frames, files=[])
    if args.images_zip:
        with ZipFile(args.images_zip) as z:
            manifest["files"] += extract_prefix(z, output, args.sequence, args.frames)
    else:
        from remotezip import RemoteZip
        url = OFFICIAL + "data_odometry_gray.zip"
        with RemoteZip(url, timeout=45) as z:
            manifest["files"] += extract_prefix(z, output, args.sequence, args.frames)
        manifest["image_url"] = url
    if args.poses_zip:
        with ZipFile(args.poses_zip) as z:
            manifest["files"] += extract_prefix(z, output, args.sequence, args.frames)
    elif args.remote:
        import io
        with urlopen(OFFICIAL + "data_odometry_poses.zip", timeout=45) as response:
            payload = response.read(10 * 1024 * 1024)
        with ZipFile(io.BytesIO(payload)) as z:
            manifest["files"] += extract_prefix(z, output, args.sequence, args.frames)
    output.mkdir(parents=True, exist_ok=True)
    (output / "acquisition.json").write_text(json.dumps(manifest, indent=2))
    from .dataset import KittiSequence
    seq = KittiSequence(output, args.sequence, args.frames)
    if len(seq.images) != args.frames or seq.ground_truth is None:
        raise RuntimeError("incomplete acquisition; supply the official pose archive")
    print(f"Acquired {len(seq.images)} official frames, calibration, timestamps and poses into {output}")


if __name__ == "__main__":
    main()
