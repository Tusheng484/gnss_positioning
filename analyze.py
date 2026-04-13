import pandas as pd
import numpy as np

def load_pos(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    data = []
    for line in lines:
        if line.strip().startswith('Wk'):
            continue
        parts = line.strip().split()
        if len(parts) < 14:
            continue
        wk = int(parts[0])
        sow = float(parts[1])
        x = float(parts[2])
        y = float(parts[3])
        z = float(parts[4])
        data.append([wk, sow, x, y, z])
    df = pd.DataFrame(data, columns=['Wk', 'SOW', 'X', 'Y', 'Z'])
    return df

def main():
    df_ref = load_pos('d:/gnss_positioning/参考数据/NovatelOEM20211114-01.pos')
    df_spp = load_pos('d:/gnss_positioning/output/spp_solution.txt')
    
    # 移除 NaN
    df_spp = df_spp.dropna()
    
    # 对齐历元
    df = pd.merge(df_spp, df_ref, on=['Wk', 'SOW'], suffixes=('_spp', '_ref'))
    
    # 计算差值
    df['dx'] = df['X_spp'] - df['X_ref']
    df['dy'] = df['Y_spp'] - df['Y_ref']
    df['dz'] = df['Z_spp'] - df['Z_ref']
    df['dist'] = np.sqrt(df['dx']**2 + df['dy']**2 + df['dz']**2)
    
    print(f"Total matched epochs: {len(df)}")
    print(f"Mean diff (m): X={df['dx'].mean():.3f}, Y={df['dy'].mean():.3f}, Z={df['dz'].mean():.3f}")
    print(f"Mean 3D dist (m): {df['dist'].mean():.3f}")
    
    # 分布
    d = df['dist']
    print("\nDistance distribution:")
    print(f"< 0.5m   : {(d < 0.5).sum() / len(d) * 100:.1f}%")
    print(f"0.5 - 2m : {((d >= 0.5) & (d < 2)).sum() / len(d) * 100:.1f}%")
    print(f"2 - 5m   : {((d >= 2) & (d < 5)).sum() / len(d) * 100:.1f}%")
    print(f"> 5m     : {(d >= 5).sum() / len(d) * 100:.1f}%")
    
if __name__ == '__main__':
    main()
