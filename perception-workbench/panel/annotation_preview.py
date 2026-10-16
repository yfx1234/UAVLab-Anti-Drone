"""Annotate the exact selected local MP4 using the deployed board models."""
import hashlib
import json
import math
import pathlib
import re
import shutil
import time
import urllib.parse
import uuid
from annotation_transport import AnnotationTransport,JOBS_ROOT
from media_names import video_filename


class AnnotationPreview:
    def __init__(self,board,media,transport=None):
        self.board=board;self.media=media
        self.transport=transport or AnnotationTransport(board)

    def run(self,params,progress=lambda value:None):
        model_id=params.get('model_id');confidence=params.get('confidence',.55)
        if not isinstance(model_id,str) or not re.fullmatch('[0-9a-f]{64}',model_id):raise ValueError('模型无效')
        if isinstance(confidence,bool) or not isinstance(confidence,(int,float)) or not math.isfinite(confidence) or not .25<=confidence<=1:
            raise ValueError('置信度阈值需在 0.25–1 之间')
        row=self.media.selected(params.get('relative'))
        sid=row.get('source_session_id') or row['session_id']
        started=time.monotonic()
        def report(value):progress(dict(value,elapsed_s=time.monotonic()-started))
        report({'phase_code':'checking','stage':'检查所选视频与模型'})
        model=next((m for m in self.board.call('list_models') if m.get('model_id')==model_id),None)
        if model is None:raise ValueError('所选模型不存在，请刷新模型列表')
        if self.board.call('status').get('pid'):raise ValueError('请先停止相机，再生成检测预览；已有标注视频仍可播放')
        family='general' if 'glad' in model.get('name','').lower() else 'custom'
        job_id=uuid.uuid4().hex
        if sid not in self.media.registry.scenes:
            self.media.scene_names([{'session_id':sid,'display_name':row['display_name'],
                'created_at_unix_s':row['scene_created_at_unix_s'],
                'created_time_uncertain':row['created_time_uncertain'],'metadata':row['metadata']}])
        with self.media.registry.operation(sid):
            # Revalidate after acquiring the source-scene lock, including a queued rename.
            row=self.media.selected(params.get('relative'))
            folder=self.media.registry.folder(sid,create=True)
            target=folder/'exports/annotated';target.mkdir(parents=True,exist_ok=True)
            source=self.media.path(row['relative']);size=source.stat().st_size
            if not size:raise ValueError('所选视频为空')
            if shutil.disk_usage(target).free<max(1024**3,size*2):raise ValueError('电脑磁盘空间不足，请清理后重试')
            part=target/(job_id+'.mp4.part')
            cleanup_warning=None
            try:
                with self.media.open_file(row['relative']) as stream:
                    digest=hashlib.file_digest(stream,'sha256').hexdigest();stream.seek(0)
                    report({'phase_code':'upload','stage':'发送所选视频','downloaded_bytes':0,'total_bytes':size})
                    self.transport.upload(job_id,stream,size,lambda done,total:report({'phase_code':'upload','stage':'发送所选视频','downloaded_bytes':done,'total_bytes':total}))
                result=self.board.stream_json(
                    'python3 -u /home/uavlab/Jetson_ROS2_V0/perception_tools/annotate_session.py --input-job %s --input-sha256 %s --model-id %s --confidence %.6f' % (job_id,digest,model_id,confidence),report,timeout=3600)
                if not result.get('ok'):raise ValueError('检测预览生成失败：'+str(result.get('error')))
                remote=pathlib.PurePosixPath(JOBS_ROOT)/job_id/'output.mp4'
                if result.get('output')!=str(remote) or result.get('source_sha256')!=digest:raise ValueError('检测结果与所选视频不匹配')
                report({'phase_code':'transfer','stage':'取回标注视频'})
                self.board.fetch_file(str(remote),str(part),progress=lambda done,total:report({'phase_code':'transfer','stage':'取回标注视频','downloaded_bytes':done,'total_bytes':total}))
                if not part.stat().st_size:raise ValueError('取回的标注视频为空，请重试')
                report({'phase_code':'local_encode','stage':'转换为网页播放格式'})
                metadata=dict(result,model_id=model_id,model_family=family,confidence=confidence,
                    source_relative=row['relative'],source_name=row['name'],source_segment=row['segment'],
                    source_session_id=sid,source_sha256=digest,scene_created_at_unix_s=row['scene_created_at_unix_s'],generated_at_unix_s=time.time())
                output=target/video_filename(row['scene_created_at_unix_s'],'annotated',row['segment'],annotation=metadata,token=job_id[:12])
                if output.exists():raise ValueError('标注文件名已存在，请重试')
                try:
                    self.media.prepare_annotation(part,output)
                    output.with_suffix('.json').write_text(json.dumps(metadata,ensure_ascii=False),encoding='utf-8')
                except Exception:
                    output.unlink(missing_ok=True);output.with_suffix('.json').unlink(missing_ok=True);raise
                relative=output.relative_to(self.media.root).as_posix()
            finally:
                part.unlink(missing_ok=True)
                try:self.transport.cleanup(job_id)
                except Exception as error:
                    cleanup_warning='临时视频清理失败：'+str(error)
                    report({'phase_code':'cleanup_warning','stage':cleanup_warning})
            if cleanup_warning:metadata['cleanup_warning']=cleanup_warning
            report({'phase_code':'complete','stage':'检测标注视频已就绪'+('；'+cleanup_warning if cleanup_warning else '')})
            return dict(metadata,relative=relative,url='/api/media/'+urllib.parse.quote(relative))
