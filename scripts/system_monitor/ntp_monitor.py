#!/usr/bin/env python

import socket
import re
from subprocess import Popen, PIPE
from typing import Dict, Optional, Union

#sudo apt install ntpdate

def get_ntp_time_data(ntp_host: str = 'ntp.ubuntu.com') -> Dict[str, Union[float, str, bool, None]]:
    """
    A function to collect time offset data with an NTP server
    
    Args:
        ntp_host (str): NTP server hostname (default: ntp.ubuntu.com)
    
    Returns:
        dict: NTP time related data
            - 'success' (bool): successful data collection
            - 'offset_seconds' (float): time difference (seconds)
            - 'offset_milliseconds' (float): time difference (milliseconds)
            - 'offset_microseconds' (float): time difference (microseconds)
            - 'ntp_host' (str): NTP server address
            - 'local_host' (str): local hostname
    """
    
    result = {
        'success': False,
        'offset_seconds': None,
        'offset_milliseconds': None,
        'offset_microseconds': None,
        'ntp_host': ntp_host,
        'local_host': socket.gethostname()
    }
    
    try:
        # ntpdate 명령 실행
        p = Popen(["ntpdate", "-q", ntp_host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
        res = p.wait()
        (output, error) = p.communicate()
        
        # Convert bytes to string (Python 3 compatibility)
        if isinstance(output, bytes):
            output = output.decode('utf-8')
        
        if res == 0:
            # Parse offset (e.g., "offset -0.123456")
            offset_match = re.search(r"offset ([-\d.]+)", output)
            if offset_match:
                offset_seconds = float(offset_match.group(1))
                result['success'] = True
                result['offset_seconds'] = offset_seconds
                result['offset_milliseconds'] = offset_seconds * 1000
                result['offset_microseconds'] = offset_seconds * 1000000
                
    except Exception:
        # If an error occurs, keep the default value
        pass
    
    return result


def get_ntp_offset(ntp_host: str = 'ntp.ubuntu.com') -> Optional[float]:
    """
    A simple function to return the NTP offset in microseconds
    
    Args:
        ntp_host (str): NTP server hostname
    
    Returns:
        float: offset (microseconds), None if failed
    """
    data = get_ntp_time_data(ntp_host)
    return data['offset_microseconds'] if data['success'] else None


# Example usage
if __name__ == "__main__":
    # 1. Get all time data
    time_data = get_ntp_time_data()
    print("NTP Time Data:")
    for key, value in time_data.items():
        print(f"  {key}: {value}")
    
    print("\n")
    
    # 2. Get only the offset
    offset = get_ntp_offset()
    if offset is not None:
        print(f"NTP Offset: {offset} microseconds")
    else:
        print("Failed to get NTP offset")