#!/usr/bin/env python3
"""Idle-only SVO export, timestamp mappings, session-grouped dataset splits."""
import argparse
import hashlib
import json
import math
import pathlib
import subprocess
from collections import Counter

# RGB pixels/timing stay compatible. Old depth assets used the SDK's default
# millimeters with meter-sized limits, so they must be recomputed independently.
MEDIA_CONTRACTS={'rgb_video':'native_stream_v1','depth_video':'native_depth_meter_view_v2'}

def valid_depth_report(report):
    fraction=report.get('depth_valid_sample_fraction')
    return (report.get('depth_unit')=='METER' and report.get('depth_view')=='SDK_VIEW_DEPTH'
            and report.get('depth_minimum_m')==.3 and report.get('depth_maximum_m')==10.5
            and isinstance(fraction,(float,int)) and not isinstance(fraction,bool)
            and math.isfinite(fraction) and 0<fraction<=1)

def timestamp_frame_plan(stamps,fps):
    if not stamps:return []
    plan=[0]
    for i,stamp in enumerate(stamps[1:],1):
        slot=round((stamp-stamps[0])*fps/1e9)
        while len(plan)<slot:plan.append(i-1)
        if len(plan)==slot:plan.append(i)
    return plan

def validate_session_integrity(summary,reports,frames):
    errors=[]
    expected={s['segment_id'] for s in summary.get('segments',[])}
    actual={s['segment_id']:s['frames'] for s in reports}
    if not expected or expected!=set(actual):errors.append('missing/unexpected SVO segment')
    counts=Counter();digests={}
    for f in frames:
        if f.get('submitted') is not True and f.get('recorded') is not True:continue
        sid=f.get('segment_id');counts[sid]+=1
        digests.setdefault(sid,hashlib.sha256()).update((str(f['stamp_ns'])+'\n').encode())
    if sum(counts.values())!=summary.get('submitted_frames',summary.get('recorded_frames')) or sum(counts.values())!=sum(actual.values()):
        errors.append('recorded frame log count mismatch')
    for sid,count in actual.items():
        if counts[sid]!=count:errors.append(f'segment {sid} metadata count mismatch')
        report=next(r for r in reports if r['segment_id']==sid)
        if sid not in digests or digests[sid].hexdigest()!=report.get('timestamp_sha256'):
            errors.append(f'segment {sid} capture/playback timestamp mismatch')
    if summary.get('metadata_complete') is not True or summary.get('tail_metadata_may_arrive') is not False:
        errors.append('metadata finalization unverified')
    if summary.get('dropped_metadata',0):errors.append('metadata dropped')
    return errors

def grouped_split(groups):
    names=sorted(groups,key=lambda name:hashlib.sha256(name.encode()).hexdigest())
    n=len(names)
    if n<3:return {'train':names,'validation':[],'test':[]}
    hold=max(1,round(n*.1))
    return {'train':names[:-2*hold],'validation':names[-2*hold:-hold],'test':names[-hold:]}

def prepare_dataset_index(root):
    root=pathlib.Path(root).resolve();groups={};images=[]
    for session in sorted(root.iterdir()):
        summary=session/'session_summary.json'
        if not session.is_dir() or not summary.is_file() or json.loads(summary.read_text()).get('state')!='complete':continue
        rows=[]
        for mapping in sorted((session/'exports').glob('*/mapping.jsonl')):
            for line in mapping.read_text().splitlines():
                row=json.loads(line);image=pathlib.Path(row['file']).resolve()
                if not image.is_relative_to(session) or not image.is_file():raise ValueError('dataset image missing/outside its session')
                rows.append({'session_id':session.name,'image_relative_path':str(image.relative_to(root)),
                             'stamp_ns':row['stamp_ns'],'svo_position':row['svo_position'],'annotation_status':'unlabeled','target_present':None})
        if rows:groups[session.name]=rows;images.extend(rows)
    result={'schema_version':1,'images':images,'splits':grouped_split(groups),'split_unit':'session',
            'note':'Unlabeled frames are not negative labels. Review every target; do not split adjacent video frames randomly.'}
    (root/'dataset_index.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return result

def ready_media(session_id,mode):
    """Read-only cache check; never opens the camera or rewrites recordings."""
    from perception_supervisor import RECORDS,session_path,read_json
    if mode not in ('rgb_video','depth_video'):raise ValueError('invalid media mode')
    folder=session_path(RECORDS,session_id);summary=read_json(folder/'session_summary.json')
    result={'ready':False,'session_id':session_id,'mode':mode}
    cached=read_json(folder/'media_assets.json').get(mode,{})
    sources=[{'name':p.name,'bytes':p.stat().st_size,'mtime_ns':p.stat().st_mtime_ns} for p in sorted((folder/'segments').glob('*.svo*'))]
    if not sources or summary.get('state')!='complete' or summary.get('metadata_complete') is not True or summary.get('tail_metadata_may_arrive') is not False:return result
    if cached.get('sources')!=sources or cached.get('contract')!=MEDIA_CONTRACTS[mode] or not cached.get('reports'):return result
    try:
        for r in cached['reports']:
            if mode=='depth_video' and not valid_depth_report(r):return result
            path=pathlib.Path(r['video'])
            if not path.resolve().is_relative_to((folder/'exports').resolve()) or path.name!=mode+'.mp4' or not path.is_file() or path.stat().st_size!=r['video_bytes']:return result
    except (OSError,KeyError,TypeError):return result
    return dict(result,ready=True,segments=cached['reports'],verified_frames=summary['verified_frames'],state='complete',integrity_errors=[],cached=True)

def export(session_id,fps=2,mode='frames'):
    from perception_supervisor import RECORDS,WS,session_path,read_json,atomic_json
    folder=session_path(RECORDS,session_id)
    if not folder.is_dir():raise ValueError('session missing')
    if not .2<=fps<=30 or mode not in ('frames','rgb_video','depth_video','yolo_video','verify'):raise ValueError('invalid export options')
    summary=read_json(folder/'session_summary.json')
    if summary.get('state') in ('recording','finalizing'):raise ValueError('session still active')
    exe=WS/'install_perception/zed_tracking/lib/zed_tracking/export_svo_rgb'
    out=folder/'exports';out.mkdir(exist_ok=True)
    direct=mode in ('rgb_video','depth_video')
    sources=[{'name':p.name,'bytes':p.stat().st_size,'mtime_ns':p.stat().st_mtime_ns} for p in sorted((folder/'segments').glob('*.svo*'))]
    assets=read_json(folder/'media_assets.json');cached=assets.get(mode,{})
    if direct:
        ready=ready_media(session_id,mode)
        if ready['ready']:return ready
    reports=[]
    for path in sorted((folder/'segments').glob('*.svo*')):
        cmd=[str(exe),str(path),str(out/path.stem),'0' if direct else str(fps),mode if direct or mode=='verify' else 'frames']
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=3600)
        if p.returncode:raise ValueError('SVO playback failed: '+p.stderr[-1000:])
        report=json.loads(p.stdout.strip().splitlines()[-1]);report['segment_id']=int(path.stem.removeprefix('segment_'))
        if mode=='depth_video' and not valid_depth_report(report):raise ValueError('Depth export has no valid metric depth or uses an outdated rendering contract')
        reports.append(report)
    if not reports:raise ValueError('no SVO segments')
    if mode=='yolo_video':
        try:import cv2
        except ImportError:raise ValueError('OpenCV Python unavailable for offline MP4 export')
        import sqlite3,tempfile
        database=tempfile.TemporaryDirectory(prefix='observations_',dir=out)
        observations=sqlite3.connect(str(pathlib.Path(database.name)/'lookup.sqlite'))
        observations.execute('CREATE TABLE observations(stamp INTEGER PRIMARY KEY, targets TEXT)')
        if mode=='yolo_video' and (folder/'observations.jsonl').exists():
            with (folder/'observations.jsonl').open() as stream:
                for line in stream:
                    frame=json.loads(line);observations.execute('INSERT OR REPLACE INTO observations VALUES(?,?)',
                        (frame['stamp_ns'],json.dumps(frame.get('targets',[]))))
            observations.commit()
        for report in reports:
            mapping=pathlib.Path(report['mapping']);writer=None
            try:
                rows=[json.loads(line) for line in mapping.read_text().splitlines()]
                plan=timestamp_frame_plan([r['stamp_ns'] for r in rows],fps);previous_index=None;image=None
                for index in plan:
                    if index==previous_index:writer.write(image);continue
                    previous_index=index;row=rows[index];image=cv2.imread(row['file'])
                    if image is None:raise ValueError('exported frame missing')
                    found=observations.execute('SELECT targets FROM observations WHERE stamp=?',(row['stamp_ns'],)).fetchone()
                    for target in json.loads(found[0]) if found else []:
                        x1,y1,x2,y2=map(int,target['bbox_xyxy']);cv2.rectangle(image,(x1,y1),(x2,y2),(0,220,0),2)
                        cv2.putText(image,target['mode'],(x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,0),1)
                    if writer is None:
                        output=mapping.parent/(mode+'.mp4');writer=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*'mp4v'),fps,(image.shape[1],image.shape[0]))
                        if not writer.isOpened():raise ValueError('MP4 encoder unavailable')
                    writer.write(image)
            finally:
                if writer is not None:writer.release()
            if writer is None:raise ValueError('no frames for MP4 export')
            reader=cv2.VideoCapture(str(output));decoded=0
            while True:
                ok,_=reader.read()
                if not ok:break
                decoded+=1
            reader.release()
            if decoded!=len(plan):raise ValueError('MP4 playback count mismatch')
            report.update(video=str(output),video_frames=decoded,video_fps=fps,
                          video_duration_s=decoded/fps,video_timebase='capture_timestamp_with_repeated_frames')
        observations.close();database.cleanup()
    recorded=sum(r['frames'] for r in reports)
    issues=['recorded frame log missing']
    if (folder/'frames.jsonl').is_file():
        with (folder/'frames.jsonl').open() as stream:
            issues=validate_session_integrity(summary,reports,(json.loads(line) for line in stream))
    summary.update(state='incomplete' if issues else 'complete',recorded_frames=recorded,verified_frames=recorded,verification=reports,integrity_errors=issues)
    atomic_json(folder/'session_summary.json',summary)
    if direct and not issues:
        for report in reports:report['video_bytes']=pathlib.Path(report['video']).stat().st_size
        assets[mode]={'contract':MEDIA_CONTRACTS[mode],'sources':sources,'reports':reports};atomic_json(folder/'media_assets.json',assets)
    return {'session_id':session_id,'mode':mode,'segments':reports,'verified_frames':recorded,'state':summary['state'],'integrity_errors':issues}

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--session',required=True);a.add_argument('--fps',type=float,default=2);a.add_argument('--mode',default='frames')
    args=a.parse_args();print(json.dumps(export(args.session,args.fps,args.mode)))
