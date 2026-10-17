"""A PC UTC reference plus boot-scoped monotonic time; requires no clock privilege."""
import math
import time

def reference(pc_unix_s,boot_id):
    if type(pc_unix_s) not in (int,float) or not math.isfinite(pc_unix_s) or not 1704067200<=pc_unix_s<4102444800:raise ValueError('invalid PC time reference')
    return {'pc_unix_s':pc_unix_s,'board_monotonic_s':time.monotonic(),'boot_id':boot_id,'source':'pc_reference_monotonic'}

def estimate(value,boot_id):
    if value.get('boot_id')!=boot_id:return None
    try:return value['pc_unix_s']+time.monotonic()-value['board_monotonic_s']
    except (KeyError,TypeError):return None
