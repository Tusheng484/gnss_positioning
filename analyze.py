import pandas as pd
import numpy as np


FIXED_COLUMNS = [
    'Wk', 'SOW',
    'ECEF-X/m', 'ECEF-Y/m', 'ECEF-Z/m',
    'REF-ECEF-X/m', 'REF-ECEF-Y/m', 'REF-ECEF-Z/m',
    'EAST/m', 'NORTH/m', 'UP/m',
    'B/deg', 'L/deg', 'H/m',
    'VX/m', 'VY/m', 'VZ/m',
    'PDOP', 'SigmaP', 'SigmaV',
    'GS', 'BS', 'n',
]

REF_COLUMNS = {'REF-ECEF-X/m', 'REF-ECEF-Y/m', 'REF-ECEF-Z/m'}


def load_solution(file_path):
    data = []
    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith('Wk'):
                continue

            parts = stripped.split()
            if len(parts) < len(FIXED_COLUMNS):
                continue

            data.append(parts[:len(FIXED_COLUMNS)])

    df = pd.DataFrame(data, columns=FIXED_COLUMNS)
    for col in FIXED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    return df


def print_distribution(distance_series):
    d = distance_series.dropna()
    if d.empty:
        print("\nDistance distribution:")
        print("< 0.5m   : N/A")
        print("0.5 - 2m : N/A")
        print("2 - 5m   : N/A")
        print("> 5m     : N/A")
        return

    total = len(d)
    print("\nDistance distribution:")
    print(f"< 0.5m   : {(d < 0.5).sum() / total * 100:.1f}%")
    print(f"0.5 - 2m : {((d >= 0.5) & (d < 2)).sum() / total * 100:.1f}%")
    print(f"2 - 5m   : {((d >= 2) & (d < 5)).sum() / total * 100:.1f}%")
    print(f"> 5m     : {(d >= 5).sum() / total * 100:.1f}%")


def print_other_comparison(df_cmp):
    compare_columns = [
        'EAST/m', 'NORTH/m', 'UP/m',
        'B/deg', 'L/deg', 'H/m',
        'VX/m', 'VY/m', 'VZ/m',
        'PDOP', 'SigmaP', 'SigmaV',
        'GS', 'BS', 'n',
    ]

    available_columns = []
    for col in compare_columns:
        sol_col = f'{col}_sol'
        ref_col = f'{col}_ref'
        if sol_col in df_cmp.columns and ref_col in df_cmp.columns:
            diff = df_cmp[sol_col] - df_cmp[ref_col]
            if diff.notna().any():
                available_columns.append((col, diff.mean()))

    if not available_columns:
        print("\nOther value comparison:")
        print("No common numeric fields available for comparison.")
        return

    print("\nOther value comparison:")
    for col, mean_diff in available_columns:
        print(f"Mean diff: {col}={mean_diff:.6f}")

def main():
    df_ref = load_solution('D:\\gnss_positioning\\参考数据\\NovatelOEM20211114-01.pos')
    df_sol = load_solution('D:\\gnss_positioning\\output\\network\\spp_solution.txt')

    # 接收机位置误差直接使用结果文件内置参考坐标。
    position_columns = [
        'ECEF-X/m', 'ECEF-Y/m', 'ECEF-Z/m',
        'REF-ECEF-X/m', 'REF-ECEF-Y/m', 'REF-ECEF-Z/m',
    ]
    df_pos = df_sol.dropna(subset=position_columns).copy()
    df_pos['dx'] = df_pos['ECEF-X/m'] - df_pos['REF-ECEF-X/m']
    df_pos['dy'] = df_pos['ECEF-Y/m'] - df_pos['REF-ECEF-Y/m']
    df_pos['dz'] = df_pos['ECEF-Z/m'] - df_pos['REF-ECEF-Z/m']
    df_pos['dist'] = np.sqrt(df_pos['dx'] ** 2 + df_pos['dy'] ** 2 + df_pos['dz'] ** 2)

    print(f"Total matched epochs: {len(df_pos)}")
    if df_pos.empty:
        print("Mean diff (m): X=N/A, Y=N/A, Z=N/A")
        print("Mean 3D dist (m): N/A")
    else:
        print(
            f"Mean diff (m): X={df_pos['dx'].mean():.6f}, "
            f"Y={df_pos['dy'].mean():.6f}, Z={df_pos['dz'].mean():.6f}"
        )
        print(f"Mean 3D dist (m): {df_pos['dist'].mean():.6f}")

    print_distribution(df_pos['dist'])

    # 其余数值按历元与参考文件对比。
    compare_columns = [col for col in FIXED_COLUMNS if col not in REF_COLUMNS]
    df_cmp = pd.merge(
        df_sol[['Wk', 'SOW'] + compare_columns[2:]],
        df_ref[['Wk', 'SOW'] + compare_columns[2:]],
        on=['Wk', 'SOW'],
        suffixes=('_sol', '_ref'),
    )
    print_other_comparison(df_cmp)

if __name__ == '__main__':
    main()
