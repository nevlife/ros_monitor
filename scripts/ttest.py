import subprocess

def parse_pmon_output(lines):
    """
    nvidia-smi pmon -c 1 출력 예시:
    # gpu       pid  type  sm   mem  enc  dec  jpg  ofa  command 
    # Idx         #   C/G   %    %    %    %    %    %   name 
        0      1234   C    50   30    -    -    -    -   python
        0      2345   G    -    20    -    -    -    -   Xorg
    """
    parsed_data = []
    
    # 실제 데이터 라인을 걸러낼 때, 주석(#)으로 시작하는 라인은 무시
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        
        # 공백 기준으로 분할
        cols = line.split()
        # 예시: ['0', '1234', 'C', '50', '30', '-', '-', '-', '-', 'python']
        # 인덱스/개수가 정확히 맞는지 확인 (10개)
        if len(cols) >= 10:
            gpu_idx = cols[0]
            pid = cols[1]
            process_type = cols[2]
            sm_usage = cols[3]
            mem_usage = cols[4]
            enc_usage = cols[5]
            dec_usage = cols[6]
            jpg_usage = cols[7]
            ofa_usage = cols[8]
            command = " ".join(cols[9:])  # command 부분에 공백이 포함될 수도 있으므로 join

            # '-' 로 표시된 항목은 None 처럼 처리
            sm_usage = None if sm_usage == '-' else float(sm_usage)
            mem_usage = None if mem_usage == '-' else float(mem_usage)
            enc_usage = None if enc_usage == '-' else float(enc_usage)
            dec_usage = None if dec_usage == '-' else float(dec_usage)
            jpg_usage = None if jpg_usage == '-' else float(jpg_usage)
            ofa_usage = None if ofa_usage == '-' else float(ofa_usage)

            parsed_data.append({
                'gpu_idx': gpu_idx,
                'pid': pid,
                'type': process_type,
                'sm_usage': sm_usage,
                'mem_usage': mem_usage,
                'enc_usage': enc_usage,
                'dec_usage': dec_usage,
                'jpg_usage': jpg_usage,
                'ofa_usage': ofa_usage,
                'command': command
            })
    
    return parsed_data

def get_pmon_data():
    cmd = ["nvidia-smi", "pmon", "-c", "1"]
    try:
        output = subprocess.check_output(cmd, universal_newlines=True)
    except subprocess.CalledProcessError as e:
        # nvidia-smi가 실패하거나 잘못된 옵션인 경우
        print("Error executing nvidia-smi pmon:", e)
        return []

    lines = output.split("\n")
    data = parse_pmon_output(lines)
    return data

if __name__ == "__main__":
    pmon_data = get_pmon_data()
    for entry in pmon_data:
        print(entry)
