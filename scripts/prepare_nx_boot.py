#!/usr/bin/env python3
"""Prepare a carrier-preserving NX16 boot candidate; no flashing or automatic reboot."""
import argparse,datetime,hashlib,json,os,pathlib,re,shutil,subprocess,tempfile

LABEL='UAVLAB_NX16'
def entries(text):
    matches=list(re.finditer(r'^\s*LABEL\s+(\S+)\s*$',text,re.M))
    return {m.group(1):text[m.end():matches[i+1].start() if i+1<len(matches) else len(text)] for i,m in enumerate(matches)}

def field(body,key):
    values=re.findall(r'^\s*'+key+r'\s+([^\n]+)$',body,re.M)
    if len(values)!=1:raise ValueError('boot entry needs exactly one '+key)
    return values[0].strip()

def render_extlinux(text,dtb,overlay):
    for path in (dtb,overlay):
        if not re.fullmatch(r'/boot/[a-zA-Z0-9_+./-]+',path) or '..' in pathlib.PurePosixPath(path).parts:
            raise ValueError('invalid boot path')
    default=re.findall(r'^\s*DEFAULT\s+(\S+)\s*$',text,re.M)
    if len(default)!=1:raise ValueError('one explicit DEFAULT required')
    blocks=entries(text)
    if LABEL in blocks:
        if default[0]==LABEL and field(blocks[LABEL],'FDT')==dtb and field(blocks[LABEL],'OVERLAYS')==overlay:
            for key in ('LINUX','INITRD','APPEND'):field(blocks[LABEL],key)
            return text
        raise ValueError('candidate label already used by another configuration')
    if default[0] not in blocks:raise ValueError('default boot entry missing')
    source=blocks[default[0]]
    lines=[f'LABEL {LABEL}','\tMENU LABEL UAVLab DAMIAO V2 Orin NX 16GB']
    for key in ('LINUX','INITRD','APPEND'):lines.append('\t'+key+' '+field(source,key))
    lines.extend(['\tFDT '+dtb,'\tOVERLAYS '+overlay])
    result=re.sub(r'^\s*DEFAULT\s+\S+\s*$',f'DEFAULT {LABEL}',text,count=1,flags=re.M)
    return result.rstrip()+'\n\n'+'\n'.join(lines)+'\n'

def command(args):
    return subprocess.run(args,capture_output=True,text=True,check=True,timeout=25).stdout.strip()

def read_fdt(path,node,prop,type='s'):
    return command(['fdtget','-t',type,str(path),node,prop])

def build_candidate(carrier,reference,target):
    compatible=read_fdt(reference,'/','compatible').split()
    if 'nvidia,p3767-0000' not in compatible:raise ValueError('NX16 reference DTB required')
    shutil.copy2(carrier,target)
    command(['fdtput','-t','s',str(target),'/','compatible',*compatible])
    command(['fdtput','-t','s',str(target),'/','model','UAVLab DAMIAO V2 Jetson Orin NX 16GB'])
    for unit in ('nvdla0@15880000','nvdla1@158c0000','pva0@16000000'):
        node='/bus@0/host1x@13e00000/'+unit
        command(['fdtput','-t','s',str(target),node,'status',read_fdt(reference,node,'status')])
    for cluster in range(3):
        node=f'/opp-table-cluster{cluster}/opp-1510400000'
        parent=node.rsplit('/',1)[0]
        if 'opp-1510400000' not in command(['fdtget','-l',str(target),parent]).split():
            command(['fdtput','-c',str(target),node])
        for key in ('opp-hz','opp-peak-kBps'):
            values=read_fdt(reference,node,key,'x').split()
            command(['fdtput','-t','x',str(target),node,key,*values])

def digest(path):return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()

def atomic_write(path,data):
    path=pathlib.Path(path)
    with tempfile.NamedTemporaryFile(dir=path.parent,delete=False) as stream:
        stream.write(data);temporary=pathlib.Path(stream.name)
    try:
        temporary.chmod(0o644);os.replace(temporary,path)
    finally:temporary.unlink(missing_ok=True)

def prepare(apply=False):
    if os.geteuid()!=0:raise ValueError('root required to verify module EEPROM and backup boot files')
    part=pathlib.Path('/sys/bus/i2c/devices/0-0050/eeprom').read_bytes()[20:50].decode(errors='replace')
    if not part.startswith('699-13767-0000-'):raise ValueError('actual module is not NX16 P3767-0000')
    root=pathlib.Path('/tmp/zed_perception');root.mkdir(exist_ok=True)
    import fcntl
    with (root/'control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        state=json.loads((root/'supervisor.json').read_text()) if (root/'supervisor.json').exists() else {}
        if state.get('pid') and pathlib.Path('/proc',str(state['pid'])).exists():raise ValueError('release camera before boot deployment')
        with (root/'camera.lock').open('a') as camera_lock:
            fcntl.flock(camera_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            carrier=pathlib.Path('/boot/dtb/kernel_tegra234-j401-p3768-0000+p3767-0003-recomputer.dtb')
            reference=pathlib.Path('/boot/tegra234-p3768-0000+p3767-0000-nv-super.dtb')
            overlay=pathlib.Path('/boot/damiao-zed-mono.dtbo');extlinux=pathlib.Path('/boot/extlinux/extlinux.conf')
            target=pathlib.Path('/boot/dtb/uavlab-damiao-nx16-36.4.3.dtb')
            original=extlinux.read_text();updated=render_extlinux(original,str(target),str(overlay))
            with tempfile.TemporaryDirectory(prefix='uavlab-nx-boot-') as scratch:
                candidate=pathlib.Path(scratch)/'candidate.dtb';merged=pathlib.Path(scratch)/'merged.dtb'
                build_candidate(carrier,reference,candidate)
                command(['fdtoverlay','-i',str(candidate),'-o',str(merged),str(overlay)])
                dump=command(['dtc','-I','dtb','-O','dts',str(merged)])
                if not all(x in dump for x in ('zed-cam0-reset','output-high','zedx_right_0@20')):
                    raise ValueError('candidate is missing verified Mono/reset camera configuration')
                report={'apply':apply,'module_sku':'P3767-0000','candidate':str(target),'source_sha256':digest(carrier),'reference_sha256':digest(reference),'overlay_sha256':digest(overlay),'candidate_sha256':digest(candidate),'power_profile':'NX16 reference 25W / ID3','original_default':re.findall(r'^DEFAULT (\S+)',original,re.M)[0]}
                if apply:
                    backup=pathlib.Path('/var/backups')/('uavlab-nx16-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'));backup.mkdir()
                    for source,name in ((extlinux,'extlinux.conf'),(carrier,'carrier.dtb'),(overlay,'mono.dtbo'),(pathlib.Path('/var/lib/nvpmodel/status'),'nvpmodel-status')):
                        if source.exists():shutil.copy2(source,backup/name)
                    (backup/'nvpmodel-link.txt').write_text(os.path.realpath('/etc/nvpmodel.conf')+'\n')
                    power=pathlib.Path('/etc/nvpmodel/nvpmodel_p3767_0000_super.conf')
                    if not power.is_file():raise ValueError('matching NX power configuration missing')
                    atomic_write(target,candidate.read_bytes())
                    # Avoid nvpower's SKU relinking resetting status to its 40W default.
                    link=pathlib.Path('/etc/nvpmodel.conf.uavlab-new');link.unlink(missing_ok=True)
                    link.symlink_to(power);os.replace(link,'/etc/nvpmodel.conf')
                    # Runtime clocks are unchanged until the controlled next boot.
                    atomic_write('/var/lib/nvpmodel/status',b'pmode:0003')
                    atomic_write(extlinux,updated.encode())
                    report['backup']=str(backup)
                    atomic_write(backup/'deployment.json',(json.dumps(report,indent=2)+'\n').encode())
                return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    print(json.dumps(prepare(args.apply),indent=2))
