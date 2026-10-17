"""Bounded receiver recording; view mode fixed for the whole local session."""
from collections import deque
import io
import json
import pathlib
import shutil
import subprocess
import threading
import time
import uuid
from PIL import Image,ImageDraw
from scene_registry import SceneRegistry

class PreviewRecorder:
    def __init__(self,root,registry=None):
        self.root=pathlib.Path(root);self.lock=threading.RLock();self.cv=threading.Condition(self.lock)
        self.registry=registry or SceneRegistry.shared(self.root.parent/'board_sessions',self.root)
        self.queue=deque(maxlen=2);self.active=False;self.finishing=False;self.frames=0;self.dropped=0
        self.files=[];self.error=None;self.view_mode='rgb';self.thread=None;self.folder=None
        self.preview_id=None;self.created_at_unix_s=None;self.source_session_id=None
    @staticmethod
    def ffmpeg():
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    def start(self,view_mode,source_session_id=None):
        if view_mode not in ('rgb','yolo'):raise ValueError('录像样式无效')
        ffmpeg=self.ffmpeg()
        with self.lock:
            if self.active or self.finishing:raise ValueError('本机预览录像已经运行')
            self.root.mkdir(parents=True,exist_ok=True)
            import shutil
            if shutil.disk_usage(self.root).free<1024**3:raise ValueError('电脑录像磁盘不足 1 GiB')
            self.preview_id='preview_'+uuid.uuid4().hex;self.created_at_unix_s=time.time();self.source_session_id=source_session_id
            self.folder=self.registry.create_preview(self.preview_id,view_mode,self.created_at_unix_s,source_session_id)
            self.active=True;self.finishing=False;self.view_mode=view_mode;self.frames=0;self.dropped=0;self.error=None;self.files=[];self.queue.clear()
            self.thread=threading.Thread(target=self._run,args=(ffmpeg,),daemon=True);self.thread.start()
            return self.snapshot()
    def consume(self,packet):
        with self.cv:
            if not self.active:return
            if len(self.queue)>=2:self.dropped+=1
            self.queue.append(packet);self.cv.notify()
    def _run(self,ffmpeg):
        proc=None;stderr=None;sidecar=None;segment=0;previous=None
        def finish():
            nonlocal proc,stderr
            if proc:
                proc.stdin.close()
                if proc.wait(timeout=30)!=0:raise ValueError('FFmpeg 录像未正常结束，请查看场次编码日志')
                proc=None
            if stderr:stderr.close();stderr=None
        try:
            sidecar=(self.folder/'frames.jsonl').open('w',encoding='utf-8')
            while True:
                with self.cv:
                    self.cv.wait_for(lambda:self.queue or not self.active,timeout=.5)
                    if not self.queue:
                        if not self.active:break
                        continue
                    packet=self.queue.popleft()
                image=Image.open(io.BytesIO(packet.jpeg)).convert('RGB');meta=packet.metadata
                if shutil.disk_usage(self.folder).free<1024**3:
                    raise ValueError('电脑录像磁盘低于 1 GiB，已结束录像')
                if previous and (packet.received_at-previous.received_at>.75 or meta.get('run_id')!=previous.metadata.get('run_id')):finish()
                if proc is None:
                    segment+=1;path=self.folder/f'segment_{segment:04d}.mp4';self.files.append(str(path))
                    stderr=(self.folder/f'ffmpeg_{segment:04d}.log').open('wb')
                    cmd=[ffmpeg,'-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{image.width}x{image.height}',
                         '-r','8','-i','pipe:0','-an','-c:v','libx264','-preset','ultrafast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart','-y',str(path)]
                    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=stderr,
                        creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                if self.view_mode=='yolo':
                    draw=ImageDraw.Draw(image);sx=image.width/meta.get('image_width',image.width);sy=image.height/meta.get('image_height',image.height)
                    for t in meta.get('targets',[]):
                        x1,y1,x2,y2=t['bbox_xyxy'];draw.rectangle((x1*sx,y1*sy,x2*sx,y2*sy),outline=(0,255,0),width=2)
                        label=f"#{t['track_id']} {t['mode']} {t['confidence']:.2f}"
                        if t.get('position_camera'):label+=' XYZ('+','.join(f'{v:.2f}' for v in t['position_camera'])+')m'
                        draw.text((x1*sx,max(0,y1*sy-12)),label,fill=(0,255,0))
                proc.stdin.write(image.tobytes());self.frames+=1
                sidecar.write(json.dumps({'segment':segment,'frame_key':meta['frame_key'],'stamp_ns':meta.get('stamp_ns'),
                    'received_monotonic_s':packet.received_at,'view_mode':self.view_mode,'frame':self.frames})+'\n')
                previous=packet
            finish()
        except Exception as e:
            self.error=str(e)
            try:finish()
            except Exception:pass
        finally:
            if sidecar:sidecar.close()
            with self.lock:
                self.active=False;self.finishing=False
                (self.folder/'summary.json').write_text(json.dumps(self.snapshot(),ensure_ascii=False),encoding='utf-8')
                self.registry.finish_preview(self.preview_id)
    def stop(self):
        with self.cv:self.active=False;self.finishing=True;self.cv.notify_all();thread=self.thread
        if thread:thread.join(timeout=40)
        if thread and thread.is_alive():raise TimeoutError('本机视频仍在结束，请等待')
        return self.snapshot()
    def snapshot(self):
        with self.lock:return {'active':self.active,'finalizing':self.finishing,'frames':self.frames,'dropped':self.dropped,'files':list(self.files),
            'view_mode':self.view_mode,'error':self.error,'folder':str(self.folder) if self.folder else None,
            'preview_id':self.preview_id,'created_at_unix_s':self.created_at_unix_s,'source_session_id':self.source_session_id,
            'note':'预览固定 8 fps；真实采集时间与断链在 frames.jsonl，不能替代原始素材'}
