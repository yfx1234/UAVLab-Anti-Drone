"""Scene display metadata and guarded deletion; UUID storage paths never change."""
import datetime
import json
import math
import os
import pathlib
import re
import shutil
import uuid

def read(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return {}

def folder_for(root,session_id):
    root=pathlib.Path(root).resolve()
    if not isinstance(session_id,str) or not re.fullmatch(r'[a-zA-Z0-9_-]{1,80}',session_id):raise ValueError('invalid session ID')
    folder=root/session_id
    if folder.is_symlink() or not folder.is_dir() or folder.resolve().parent!=root:raise ValueError('scene missing or unsafe path')
    if not (folder/'manifest.json').is_file():raise ValueError('scene is not registered')
    return folder

def atomic(path,value):
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False),encoding='utf-8');os.replace(temp,path)

def scene_info(folder):
    manifest=read(folder/'manifest.json');summary=read(folder/'session_summary.json');display=read(folder/'display.json')
    stamp=manifest.get('created_at_unix_s');source=manifest.get('created_time_source','board_clock');uncertain=source=='board_clock'
    if type(stamp) not in (float,int) or not math.isfinite(stamp) or not 0<stamp<4102444800:
        stamp=(folder/'manifest.json').stat().st_mtime;source='legacy_manifest_time';uncertain=True
    name=display.get('name') or datetime.datetime.fromtimestamp(stamp,datetime.timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')
    files=[{'name':p.relative_to(folder).as_posix(),'bytes':p.stat().st_size} for p in folder.rglob('*')
           if not p.is_symlink() and p.is_file() and not p.name.endswith('.tmp') and '.operation' not in p.name]
    return {'session_id':folder.name,'display_name':name,'created_at_unix_s':stamp,'created_time_source':source,
            'created_time_uncertain':uncertain,'manifest':manifest,'summary':summary,'files':files,'bytes':sum(f['bytes'] for f in files)}

def list_sessions(root):
    root=pathlib.Path(root)
    if not root.exists():return []
    rows=[]
    for p in root.iterdir():
        if not p.is_dir() or p.is_symlink() or p.name.startswith('.'):continue
        try:rows.append(scene_info(folder_for(root,p.name)))
        except (ValueError,OSError):continue
    return sorted(rows,key=lambda r:(r['created_at_unix_s'],r['session_id']),reverse=True)

def rename(root,session_id,name):
    folder=folder_for(root,session_id)
    if not isinstance(name,str) or not 1<=len(name.strip())<=100 or any(ord(c)<32 for c in name):raise ValueError('名称需为1–100字，不含换行或控制字符')
    atomic(folder/'display.json',{'name':name.strip()})
    return {'session_id':session_id,'display_name':name.strip()}

def delete(root,session_id,confirmation,active_session):
    folder=folder_for(root,session_id);summary=read(folder/'session_summary.json')
    if confirmation!=session_id:raise ValueError('删除确认与场次不一致')
    if active_session==session_id or summary.get('state') in ('recording','finalizing') or summary.get('tail_metadata_may_arrive') is True:raise ValueError('录制或收尾中的场次不能删除')
    # The resolved absolute target is checked before every recursive delete.
    root=pathlib.Path(root).resolve()
    if folder.resolve().parent!=root:raise ValueError('删除路径越界')
    shutil.rmtree(folder)
    return {'session_id':session_id,'deleted':True,'scope':'board_scene','local_copies_retained':True}
