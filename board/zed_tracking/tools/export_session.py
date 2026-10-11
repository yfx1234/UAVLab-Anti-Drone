#!/usr/bin/env python3
"""Idle-only SVO export, timestamp mappings, session-grouped dataset splits."""
import argparse
import hashlib
import json
import pathlib
import subprocess

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

def export(session_id,fps=2,mode='frames'):
    from perception_supervisor import RECORDS,WS,session_path,read_json,atomic_json
    folder=session_path(RECORDS,session_id)
    if not folder.is_dir():raise ValueError('session missing')
    if not .2<=fps<=30 or mode not in ('frames','rgb_video','yolo_video','verify'):raise ValueError('invalid export options')
    summary=read_json(folder/'session_summary.json')
    if summary.get('state') in ('recording','finalizing'):raise ValueError('session still active')
    exe=WS/'install_perception/zed_tracking/lib/zed_tracking/export_svo_rgb'
    out=folder/'exports';out.mkdir(exist_ok=True)
    reports=[]
    for path in sorted((folder/'segments').glob('*.svo*')):
        cmd=[str(exe),str(path),str(out/path.stem),str(fps),'verify' if mode=='verify' else 'frames']
        p=subprocess.run(cmd,capture_output=True,text=True,timeout=3600)
        if p.returncode:raise ValueError('SVO playback failed: '+p.stderr[-1000:])
        reports.append(json.loads(p.stdout.strip().splitlines()[-1]))
    if not reports:raise ValueError('no SVO segments')
    if mode in ('rgb_video','yolo_video'):
        try:import cv2
        except ImportError:raise ValueError('OpenCV Python unavailable for offline MP4 export')
        observations={}
        if mode=='yolo_video' and (folder/'observations.jsonl').exists():
            for line in (folder/'observations.jsonl').read_text().splitlines():
                frame=json.loads(line);observations[frame['stamp_ns']]=frame.get('targets',[])
        for report in reports:
            mapping=pathlib.Path(report['mapping']);writer=None
            try:
                for line in mapping.read_text().splitlines():
                    row=json.loads(line);image=cv2.imread(row['file'])
                    if image is None:raise ValueError('exported frame missing')
                    for target in observations.get(row['stamp_ns'],[]):
                        x1,y1,x2,y2=map(int,target['bbox_xyxy']);cv2.rectangle(image,(x1,y1),(x2,y2),(0,220,0),2)
                        cv2.putText(image,target['mode'],(x1,max(20,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,220,0),1)
                    if writer is None:
                        output=mapping.parent/(mode+'.mp4');writer=cv2.VideoWriter(str(output),cv2.VideoWriter_fourcc(*'mp4v'),fps,(image.shape[1],image.shape[0]))
                        if not writer.isOpened():raise ValueError('MP4 encoder unavailable')
                    writer.write(image)
            finally:
                if writer is not None:writer.release()
    recorded=sum(r['frames'] for r in reports)
    summary.update(state='complete',verified_frames=recorded,verification=reports)
    atomic_json(folder/'session_summary.json',summary)
    return {'session_id':session_id,'mode':mode,'segments':reports,'verified_frames':recorded}

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--session',required=True);a.add_argument('--fps',type=float,default=2);a.add_argument('--mode',default='frames')
    args=a.parse_args();print(json.dumps(export(args.session,args.fps,args.mode)))
