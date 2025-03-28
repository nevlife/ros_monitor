#!/usr/bin/env python3
import sqlite3
import os

def initialize_db():
    db_path = "/home/pgw/catkin_ws/src/ros_monitor/data/data.db"
    os.makedirs(os.path.dirname(db_path), exist_ok=True) if "/" in db_path else None
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    # 시스템 전체 리소스 (CPU, GPU, Memory 등)
    cur.execute("""
    CREATE TABLE IF NOT EXISTS total_resource (
        timestamp REAL PRIMARY KEY,
        cpu_usage_percent REAL,
        cpu_temp REAL,
        mem_used REAL,
        mem_total REAL,
        mem_usage_percent REAL,
        gpu_usage_percent REAL,
        gpu_temp REAL,
        gpu_mem_usage REAL
    )
    """)

    # 노드별 리소스 사용량
    cur.execute("""
    CREATE TABLE IF NOT EXISTS node_resource (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp REAL,
        node_name TEXT,
        cpu REAL,
        mem TEXT
    )
    """)

    # 토픽별 주기/대역폭
    cur.execute("""
    CREATE TABLE IF NOT EXISTS topic_hzbw (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp REAL,
        topic_name TEXT,
        hz REAL,
        bw TEXT
    )
    """)

    # GPU 프로세스 사용 정보
    cur.execute("""
    CREATE TABLE IF NOT EXISTS gpu_pmon (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp REAL,
        pid TEXT,
        type TEXT,
        sm_usage REAL,
        mem_usage REAL,
        command TEXT
    )
    """)

    # 경고 메시지 로그
    cur.execute("""
    CREATE TABLE IF NOT EXISTS alert_log (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp REAL,
        level TEXT,
        source TEXT,
        message TEXT
    )
    """)

    conn.commit()
    conn.close()
    print(f"[✓] Database initialized at: {db_path}")

if __name__ == "__main__":
    initialize_db()
