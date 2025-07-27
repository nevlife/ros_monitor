#!/usr/bin/env python3
import subprocess
import re
import time

#sudo apt install ifstat

def get_sys_net_stat(iface, sys):
    """시스템 네트워크 통계 읽기"""
    cmd = f'cat /sys/class/net/{iface}/statistics/{sys}'
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, stderr = p.communicate()
        return (p.returncode, stdout.decode().strip())
    except:
        return (1, "")

def get_sys_net(iface, sys):
    """시스템 네트워크 설정 읽기"""
    cmd = f'cat /sys/class/net/{iface}/{sys}'
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, stderr = p.communicate()
        return (p.returncode, stdout.decode().strip())
    except:
        return (1, "")

def get_network_traffic():
    """ifstat으로 실시간 트래픽 정보 수집"""
    try:
        p = subprocess.Popen('ifstat -q -S 1 1', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, stderr = p.communicate()
        
        if p.returncode != 0:
            return None, f"ifstat start failed: {stderr.decode()}"

        rows = stdout.decode().split('\n')
        if len(rows) < 3:
            return None, "ifstat output format error"

        ifaces = rows[0].split()
        
        # KB/s
        data = rows[2].split()
        
        traffic_info = {}
        for i in range(len(ifaces)):
            if i * 2 + 1 < len(data):
                kb_in = float(data[i * 2]) if data[i * 2].replace('.', '').isdigit() else 0
                kb_out = float(data[i * 2 + 1]) if data[i * 2 + 1].replace('.', '').isdigit() else 0
                
                traffic_info[ifaces[i]] = {
                    'input_traffic_kbps': kb_in,
                    'output_traffic_kbps': kb_out,
                    'input_traffic_mbps': kb_in / 1024,
                    'output_traffic_mbps': kb_out / 1024
                }
        
        return traffic_info, None
        
    except Exception as e:
        return None, f"Traffic collection failed: {e}"

def get_complete_network_info(net_capacity=128):
    """
    
    Args:
        net_capacity (float): network capacity (MB/s, default 128)

    Returns:
        dict: Complete network information
        {
            'traffic': {...},           # Real-time traffic information
            'interfaces': {...},        # Detailed information for each interface
            'errors': [...],           # Occurred errors
            'warnings': [...],         # Warnings
            'summary': {...}           # Overall summary information
        }
    """
    
    result = {
        'traffic': {},
        'interfaces': {},
        'errors': [],
        'warnings': [],
        'summary': {
            'total_interfaces': 0,
            'active_interfaces': 0,
            'down_interfaces': 0,
            'high_usage_interfaces': 0
        }
    }
    
    # real-time traffic information
    traffic_info, traffic_error = get_network_traffic()
    if traffic_error:
        result['errors'].append(f"Traffic collection failed: {traffic_error}")
        return result
    
    result['traffic'] = traffic_info
    
    # collect detailed information for each interface
    for iface_name in traffic_info.keys():
        iface_info = {
            'name': iface_name,
            'traffic': traffic_info[iface_name],
            'state': 'unknown',
            'mtu': 'unknown',
            'statistics': {},
            'errors': {},
            'is_ethernet': False,
            'usage_level': 'ok'  # ok, warning, error
        }
        
        # check interface state
        retcode, state = get_sys_net(iface_name, 'operstate')
        if retcode == 0:
            iface_info['state'] = state

            # check if Ethernet interface is down/dormant
            if re.match(r'eth[0-9]+', iface_name):
                iface_info['is_ethernet'] = True
                if state in ['down', 'dormant']:
                    result['errors'].append(f"Ethernet interface {iface_name} state: {state}")
                    iface_info['usage_level'] = 'error'
        
        # MTU info
        retcode, mtu = get_sys_net(iface_name, 'mtu')
        if retcode == 0:
            iface_info['mtu'] = mtu

        # Cumulative statistics
        stats_to_check = ['rx_bytes', 'tx_bytes', 'rx_packets', 'tx_packets']
        for stat in stats_to_check:
            retcode, value = get_sys_net_stat(iface_name, stat)
            if retcode == 0 and value.isdigit():
                if 'bytes' in stat:
                    iface_info['statistics'][f'{stat}_mb'] = float(value) / 1024 / 1024
                iface_info['statistics'][stat] = int(value)

        # Error statistics
        error_stats = ['collisions', 'rx_errors', 'tx_errors', 'rx_dropped', 'tx_dropped']
        for stat in error_stats:
            retcode, value = get_sys_net_stat(iface_name, stat)
            if retcode == 0 and value.isdigit():
                iface_info['errors'][stat] = int(value)

        # Check network usage
        traffic = traffic_info[iface_name]
        net_usage_in = traffic['input_traffic_mbps'] / net_capacity
        net_usage_out = traffic['output_traffic_mbps'] / net_capacity
        
        if net_usage_in > 0.95 or net_usage_out > 0.95:  # net_level_warn
            result['warnings'].append(f"{iface_name} High network utilization: IN={net_usage_in*100:.1f}%, OUT={net_usage_out*100:.1f}%")
            iface_info['usage_level'] = 'warning'
        
        iface_info['usage_percentage'] = {
            'input': net_usage_in * 100,
            'output': net_usage_out * 100
        }
        
        result['interfaces'][iface_name] = iface_info

        # Update summary information
        result['summary']['total_interfaces'] += 1
        if iface_info['state'] == 'up':
            result['summary']['active_interfaces'] += 1
        elif iface_info['state'] in ['down', 'dormant']:
            result['summary']['down_interfaces'] += 1
        
        if iface_info['usage_level'] == 'warning':
            result['summary']['high_usage_interfaces'] += 1
    
    return result

def get_network_summary():
    """Get a brief summary of the network status."""
    full_info = get_complete_network_info()
    
    summary = {
        'active_interfaces': [],
        'total_traffic_mbps': {'in': 0, 'out': 0},
        'interface_count': full_info['summary']['total_interfaces'],
        'error_count': len(full_info['errors']),
        'warning_count': len(full_info['warnings'])
    }
    
    for iface_name, iface_info in full_info['interfaces'].items():
        if iface_info['state'] == 'up':
            summary['active_interfaces'].append(iface_name)
            
        traffic = iface_info['traffic']
        summary['total_traffic_mbps']['in'] += traffic['input_traffic_mbps']
        summary['total_traffic_mbps']['out'] += traffic['output_traffic_mbps']
    
    return summary

# 사용 예시
if __name__ == "__main__":
    print("=== 완전한 네트워크 정보 ===")
    net_info = get_complete_network_info()
    
    # 트래픽 정보
    print("\n[실시간 트래픽]")
    for iface, traffic in net_info['traffic'].items():
        print(f"{iface}: IN={traffic['input_traffic_mbps']:.2f}MB/s, OUT={traffic['output_traffic_mbps']:.2f}MB/s")
    
    # 인터페이스 상세 정보
    print("\n[인터페이스 상세 정보]")
    for iface, info in net_info['interfaces'].items():
        print(f"\n{iface}:")
        print(f"  상태: {info['state']}")
        print(f"  MTU: {info['mtu']}")
        print(f"  사용률: IN={info['usage_percentage']['input']:.1f}%, OUT={info['usage_percentage']['output']:.1f}%")
        
        if 'rx_bytes_mb' in info['statistics']:
            print(f"  총 수신: {info['statistics']['rx_bytes_mb']:.1f}MB")
        if 'tx_bytes_mb' in info['statistics']:
            print(f"  총 송신: {info['statistics']['tx_bytes_mb']:.1f}MB")
        
        # 에러 정보
        errors = info['errors']
        if any(errors.values()):
            print(f"  에러: 충돌={errors.get('collisions', 0)}, RX에러={errors.get('rx_errors', 0)}, TX에러={errors.get('tx_errors', 0)}")
    
    # 경고 및 에러
    if net_info['warnings']:
        print(f"\n[경고] {len(net_info['warnings'])}개")
        for warning in net_info['warnings']:
            print(f"  - {warning}")
    
    if net_info['errors']:
        print(f"\n[에러] {len(net_info['errors'])}개")
        for error in net_info['errors']:
            print(f"  - {error}")
    
    # 요약
    print(f"\n[요약]")
    summary = net_info['summary']
    print(f"전체 인터페이스: {summary['total_interfaces']}")
    print(f"활성 인터페이스: {summary['active_interfaces']}")
    print(f"비활성 인터페이스: {summary['down_interfaces']}")
    print(f"높은 사용률 인터페이스: {summary['high_usage_interfaces']}")