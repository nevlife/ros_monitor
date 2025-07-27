#!/usr/bin/env python3
import psutil

def get_memory_info():
    """
    Extracts and returns memory information.
    
    Returns:
        dict: A dictionary containing memory information.
        {
            'total': int,           # Total memory (bytes)
            'available': int,       # Available memory (bytes)
            'used': int,            # Used memory (bytes)
            'free': int,            # Free memory (bytes)
            'percent': float,       # Usage percentage (%)
            'total_gb': float,      # Total memory (GB)
            'used_gb': float,       # Used memory (GB)
            'available_gb': float   # Available memory (GB)
        }
    """
    memory = psutil.virtual_memory()
    
    return {
        'total': memory.total,
        'available': memory.available,
        'used': memory.used,
        'free': memory.free,
        'percent': memory.percent,
        'total_gb': memory.total / (1024**3),
        'used_gb': memory.used / (1024**3),
        'available_gb': memory.available / (1024**3)
    }
