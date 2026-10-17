"""Human readable local filenames; remote recording paths remain protocol keys."""
import math
import pathlib
import re
from datetime import datetime, timezone, timedelta

BEIJING = timezone(timedelta(hours=8))


def video_kind(path):
    path = pathlib.Path(path)
    if path.parent.name == 'annotated':return 'annotated'
    return 'depth' if path.name.startswith('depth_') or '_深度视频_' in path.name else 'rgb'


def annotation_metadata(filename, metadata=None):
    data = dict(metadata or {})
    tag = str(data.get('model_tag') or filename).lower()
    if data.get('model_family') not in ('custom', 'general'):
        data['model_family'] = 'general' if 'glad' in tag else 'custom' if 'exp_' in tag or 'custom' in tag else 'unknown'
    shape = re.search(r'glad-(\d+)x(\d+)', filename)
    if not data.get('input_nchw') and shape:
        data['input_nchw'] = [1, 3, int(shape[1]), int(shape[2])]
    clock = re.search(r'_(\d{8}_\d{6})(?:_|\.)', filename)
    if not data.get('generated_at_unix_s') and clock:
        try:data['generated_at_unix_s'] = datetime.strptime(clock[1], '%Y%m%d_%H%M%S').replace(tzinfo=BEIJING).timestamp()
        except ValueError:pass
    return data


def video_filename(stamp, kind, segment, *, annotation=None, token='', original=False):
    date = datetime.fromtimestamp(stamp, BEIJING).strftime('%Y%m%d-%H%M%S')
    data = annotation or {}
    label = ('自训检测' if data.get('model_family') == 'custom' else '通用检测' if data.get('model_family') == 'general' else '检测视频') if kind == 'annotated' else '深度视频' if kind == 'depth' else '原视频RGB'
    parts = [date, label]
    segment = data.get('source_segment', segment) if kind == 'annotated' else segment
    if segment == 'session':parts.append('整场')
    elif re.fullmatch(r'segment_\d+', segment):parts.append('第' + segment[8:] + '段')
    if kind == 'annotated':
        shape = data.get('input_nchw')
        if isinstance(shape, list) and len(shape) == 4:parts.append(str(shape[3]) + 'x' + str(shape[2]))
        confidence = data.get('confidence', data.get('conf'))
        if isinstance(confidence, (int, float)) and not isinstance(confidence, bool) and math.isfinite(confidence):parts.append('c' + format(confidence, '.2f'))
        generated=data.get('generated_at_unix_s')
        if isinstance(generated,(int,float)) and not isinstance(generated,bool) and math.isfinite(generated) and generated>0:
            parts.append('g'+datetime.fromtimestamp(generated,BEIJING).strftime('%m%d-%H%M%S'))
        if token:parts.append(token)
    if original:parts.append('下载原件')
    return '_'.join(parts) + ('.mp4' if original else '.view.mp4')


def local_download_relative(registry, sid, relative):
    path = pathlib.PurePosixPath(relative)
    if path.name not in ('rgb_video.mp4', 'depth_video.mp4') or len(path.parts) != 3 or path.parts[0] != 'exports':return path
    stamp = registry.scenes[sid]['created_at_unix_s']
    return path.with_name(video_filename(stamp, video_kind(path), path.parent.name, original=True))
