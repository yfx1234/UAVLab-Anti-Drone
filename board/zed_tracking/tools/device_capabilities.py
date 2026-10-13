"""Read platform facts without treating a marketing model string as a SKU."""
import pathlib,re,os,subprocess

SKUS={'0000':('Orin NX',16),'0001':('Orin NX',8),'0003':('Orin Nano',8),'0004':('Orin Nano',4),'0005':('Orin Nano',8)}
def classify_module(compatible,memory_kib,part_number=None):
    if isinstance(compatible,str):compatible=compatible.replace('\0',' ').split()
    dt_skUs={match.group(1) for value in compatible if (match:=re.fullmatch(r'nvidia,p3767-(\d{4})',value))}
    dt_sku=next(iter(dt_skUs)) if len(dt_skUs)==1 else None
    part=re.search(r'699-13767-(\d{4})-',part_number or '')
    sku=part.group(1) if part else dt_sku
    family,size=SKUS.get(sku,('unknown',None))
    available_gib=memory_kib/1048576
    memory_matches=size is not None and size*.70<available_gib<size*1.15
    return {'module_sku':None if sku not in SKUS else 'P3767-'+sku,'module_family':family,
            'module_name':family+' '+str(size)+'GB' if size else 'unknown','memory_nominal_gib':size,
            'identification_source':'module_eeprom' if part else 'device_tree_and_memory',
            'device_tree_sku':None if dt_sku is None else 'P3767-'+dt_sku,
            'runtime_identity_consistent':sku in SKUS and sku==dt_sku and memory_matches}

def collect(root=pathlib.Path('/')):
    root=pathlib.Path(root)
    def read(path):
        try:return (root/path.lstrip('/')).read_text(errors='replace').strip('\0 \n')
        except OSError:return ''
    mem={x.split(':')[0]:x.split(':')[1].strip() for x in read('/proc/meminfo').splitlines() if ':' in x}
    memory_kib=int(mem.get('MemTotal','0 kB').split()[0]);compatible=read('/proc/device-tree/compatible').replace('\0',' ').split()
    part=None
    try:part=(root/'sys/bus/i2c/devices/0-0050/eeprom').read_bytes()[20:50].decode(errors='replace')
    except OSError:pass
    facts=classify_module(compatible,memory_kib,part)
    version={}
    for header in (root/'usr/local/zed/include/sl').glob('*.hpp'):
        for line in header.read_text(errors='replace').splitlines():
            match=re.match(r'#define\s+ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION\s+(\d+)',line)
            if match:version[match.group(1)]=match.group(2)
    sdk='.'.join(version[k] for k in ('MAJOR','MINOR','PATCH')) if len(version)==3 else None
    power_path=root/'etc/nvpmodel.conf';power_target=os.path.realpath(power_path)
    consistent=facts['runtime_identity_consistent'] and facts['module_sku'] is not None and facts['module_sku'].lower().replace('-','_') in pathlib.Path(power_target).name
    facts.update(compatible=compatible,memory=mem,sdk_version=sdk,l4t=read('/etc/nv_tegra_release'),
        online_cpus=read('/sys/devices/system/cpu/online'),nvpmodel_config=power_target,
        platform_configuration_consistent=consistent,default_profile='lossless',
        video_devices=[str(p) for p in (root/'dev').glob('video*')],
        nvenc_device_present=(root/'dev/v4l2-nvenc').exists(),nvenc_verified=False,
        nx16_ready=False,performance_acceptance='not_accepted')
    return facts
