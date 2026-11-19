"""Add genuine frame-publisher snapshots to the pinned official video example.

This instrumentation does not change tracking/mapping or synthesized measurements.
"""
from pathlib import Path
import sys
path=Path(sys.argv[1]); text=path.read_text()
text=text.replace('#include "stella_vslam/system.h"','#include "stella_vslam/system.h"\n#include "stella_vslam/publish/frame_publisher.h"')
target='slam->feed_monocular_frame(frame, timestamp, mask);'
if text.count(target)!=1: raise RuntimeError('official example source changed; inspect instrumentation')
text=text.replace(target,target+'''
                if (!eval_log_dir.empty() && num_frame % 300 == 0) {
                    cv::imwrite(eval_log_dir + "/tracking_" + std::to_string(num_frame) + ".jpg",
                                slam->get_frame_publisher()->draw_frame());
                }
''')
path.write_text(text)
