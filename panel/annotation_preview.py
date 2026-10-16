"""Generate a board annotation without downloading the source recording."""
import math
import pathlib
import re
import time
import uuid
import urllib.parse
import shutil
import json


class AnnotationPreview:
    def __init__(self, board, media):
        self.board = board
        self.media = media

    def run(self, params, progress=lambda value: None):
        sid, model_id = params.get('session_id'), params.get('model_id')
        confidence = params.get('confidence', .55)
        if not isinstance(sid, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,80}', sid):
            raise ValueError('场次无效')
        if not isinstance(model_id, str) or not re.fullmatch(r'[0-9a-f]{64}', model_id):
            raise ValueError('模型无效')
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or not .25 <= confidence <= 1:
            raise ValueError('置信度阈值需在 0.25–1 之间')
        started = time.monotonic()
        def report(value):
            progress(dict(value, elapsed_s=time.monotonic() - started))
        report({'phase_code': 'checking', 'stage': '检查场次与模型'})
        models = self.board.call('list_models')
        if not any(m.get('model_id') == model_id for m in models):
            raise ValueError('所选模型不存在，请刷新模型列表')
        rows = self.board.call('list_sessions')
        row = next((r for r in rows if r['session_id'] == sid), None)
        if row is None:
            raise ValueError('板上场次不存在，请刷新场次列表')
        # Use the same protected transfer budget as other media downloads.
        fresh = self.board.call('status')
        if fresh.get('pid'):
            raise ValueError('请先停止相机，再生成检测预览；已有标注视频仍可播放')
        self.media.scene_names([row])
        with self.media.registry.operation(sid):
            folder = self.media.registry.folder(sid, create=True)
            target = folder / 'exports' / 'annotated'
            target.mkdir(parents=True, exist_ok=True)
            if shutil.disk_usage(target).free < 1024 ** 3:
                raise ValueError('电脑磁盘需保留 1 GiB，请清理空间后重试')
            probe = target / (uuid.uuid4().hex + '.writecheck')
            try: probe.write_bytes(b'')
            finally: probe.unlink(missing_ok=True)
            ready = self.board.call('ready_media', {'session_id': sid, 'mode': 'rgb_video'})
            if not ready.get('ready'):
                report({'phase_code': 'board_convert', 'stage': '板上准备 RGB 视频'})
                request = dict(request_id=uuid.uuid4().hex, **{'expected_' + k: fresh[k] for k in ('boot_id', 'run_id', 'revision')})
                converted = self.board.call('export_session', {'session_id': sid, 'mode': 'rgb_video', 'fps': 15}, request, timeout=3700)
                if converted['result']['state'] != 'complete':
                    raise ValueError('场次不完整：' + '；'.join(converted['result'].get('integrity_errors', [])))
            report({'phase_code': 'model_load', 'stage': '加载检测模型'})
            result = self.board.stream_json(
                'python3 -u /home/uavlab/Jetson_ROS2_V0/perception_tools/annotate_session.py --session %s --model-id %s --confidence %.6f' % (sid, model_id, confidence),
                report, timeout=3600)
            if not result.get('ok'):
                raise ValueError('检测预览生成失败：' + str(result.get('error')))
            remote = pathlib.PurePosixPath(result['output'])
            expected = pathlib.PurePosixPath('/home/uavlab/Jetson_ROS2_V0/perception_records') / sid
            if remote.parent != expected or remote.suffix != '.mp4' or not re.fullmatch(r'annotated_[a-zA-Z0-9_.-]+\.mp4', remote.name):
                raise ValueError('板上标注结果路径无效')
            source = target / remote.name
            part = source.with_suffix('.mp4.part')
            report({'phase_code': 'transfer', 'stage': '取回标注视频'})
            try:
                self.board.fetch_file(str(remote), str(part), progress=lambda done, total: report({
                    'phase_code': 'transfer', 'stage': '取回标注视频', 'downloaded_bytes': done, 'total_bytes': total}))
                if not part.stat().st_size:
                    raise ValueError('取回的标注视频为空，请重试')
                part.replace(source)
            finally:
                part.unlink(missing_ok=True)
            output = source.with_name(source.stem + '.view.mp4')
            report({'phase_code': 'local_encode', 'stage': '转换为网页播放格式'})
            self.media.prepare_annotation(source, output)
            output.with_suffix('.json').write_text(json.dumps(dict(result, confidence=confidence), ensure_ascii=False), encoding='utf-8')
            relative = output.relative_to(self.media.root).as_posix()
            report({'phase_code': 'complete', 'stage': '检测标注视频已就绪'})
            return dict(result, relative=relative, url='/api/media/' + urllib.parse.quote(relative), confidence=confidence)
