def resolve_codec(requested,facts,proof):
    if requested in (None,'LOSSLESS'):return 'LOSSLESS'
    if requested!='H264_LOSSLESS':raise ValueError('unsupported recording codec')
    matching=(isinstance(facts,dict) and isinstance(proof,dict) and facts.get('module_sku')=='P3767-0000' and facts.get('platform_configuration_consistent') is True
              and proof.get('codec')==requested and proof.get('timestamp_complete') is True
              and all(facts.get(k) and facts.get(k)==proof.get(k) for k in ('module_sku','sdk_version','kernel_version')))
    if not matching:raise ValueError('当前板子尚未验证高速录像，请选择训练采集')
    return requested
