# %%
import multiprocessing
import os
import re
from concurrent.futures import ProcessPoolExecutor, as_completed
import gc
import pandas as pd
import matplotlib.pyplot as plt
from pyulog import ULog

# %%
# return the array of combined log file line
def getLogData(baseLogDir, iteration, testCase, model="iris"):
    logPath = os.path.join(baseLogDir, iteration, model, testCase, "log-combined.log_plain.log")
    if os.path.exists(logPath):
        with open(logPath, "r") as f:
            return f.readlines()
    return None

# %%
# return the ulg file name parsed from the combined log
def findUlgName(log):
    pattern = r"INFO\s+\[logger\]\s+Opened full log file:\s+(.*\.ulg)"
    match = re.search(pattern, log)
    return match.group(1) if match else ""

# %%
# return the ulog parsed from the ulg file
def getUlogData(baseUlgDir, ulgFileName):
    normPath = os.path.normpath(ulgFileName)
    ulgPath = os.path.join(baseUlgDir, normPath)
    return ULog(ulgPath) if os.path.exists(ulgPath) else None

# %%
def process_test_case(args):
    baseLogDir, baseUlgDir, testIteration, model, testCase = args

    try:
        # 1) combined log 읽기
        combinedLog = getLogData(baseLogDir, testIteration, testCase, model)
        ulgFileName = findUlgName("".join(combinedLog) if combinedLog else "")
        # 2) ULog 읽기
        ulog = getUlogData(baseUlgDir, ulgFileName)
        # 3) estimator_innovation_variance에서 gps_hpos 추출
        innov_df = pd.DataFrame(ulog.get_dataset("estimator_innovation_variances").data)
        # 4) actuator_armed에서 Arm 시점(timestamp) 추출
        armed_df = pd.DataFrame(ulog.get_dataset("actuator_armed").data)
        arm_events = armed_df[armed_df["armed"] == 1]
        arm_time = arm_events["timestamp"].iloc[0] if not arm_events.empty else None

        # 5) Arm 이후부터 로그 마지막까지 gps_hpos 통계 계산
        if arm_time is not None:
            flight_df = innov_df[innov_df["timestamp"] >= arm_time]
        else:
            flight_df = innov_df

        gps_Xseries = flight_df["gps_hpos[0]"]
        gps_Yseries = flight_df["gps_hpos[0]"]
        mean_Xgps = gps_Xseries.mean()
        mean_Ygps = gps_Xseries.mean()
        max_Xgps  = gps_Xseries.max()
        max_Ygps  = gps_Yseries.max()
        min_Xgps  = gps_Xseries.min()
        min_Ygps  = gps_Yseries.min()

        # 메모리 정리
        del ulog, innov_df, armed_df, flight_df
        gc.collect()

        return testIteration, testCase, {
            "gps_hpos[0]": {
                "mean": mean_Xgps,
                "max": max_Xgps,
                "min": min_Xgps
            },
            "gps_hpos[1]": {
                "mean": mean_Ygps,
                "max": max_Ygps,
                "min": min_Ygps
            }
        }

    except Exception as e:
        print(f"Error processing {testIteration}/{model}/{testCase}: {e}")
        return testIteration, testCase, {"error": str(e)}

# %%
def load_flight_data_parallel(baseLogDir, baseUlgDir, max_workers=None):
    if max_workers is None:
        max_workers = max(1, multiprocessing.cpu_count() // 2)

    flightData = {}
    tasks = []

    for testIteration in os.listdir(baseLogDir):
        iterationDir = os.path.join(baseLogDir, testIteration)
        flightData[testIteration] = {}

        for model_name in os.listdir(iterationDir):
            modelDir = os.path.join(iterationDir, model_name)

            for testCase in os.listdir(modelDir):
                # testCase가 normal*인 경우만 처리 (대소문자 구분 없이)
                if not testCase.lower().startswith("normal"):
                    continue
                tasks.append((baseLogDir, baseUlgDir, testIteration, model_name, testCase))

    total, done = len(tasks), 0
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        future_to_task = {executor.submit(process_test_case, t): t for t in tasks}
        for future in as_completed(future_to_task):
            it, tc, result = future.result()
            flightData[it][tc] = result
            done += 1
            if done % 10 == 0 or done == total:
                print(f"Progress: {done}/{total} ({done/total*100:.1f}%)")

    return flightData

# %%
# ulg, combined log 기본 위치
baseLogDir = os.path.expanduser("~/ws/PX4-Autopilot/logs/2025-03-21T21-42-37Z")
baseUlgDir = os.path.expanduser("~/ws/PX4-Autopilot/build/px4_sitl_default/tmp_mavsdk_tests/rootfs")

# 병렬 처리를 통한 데이터 로딩
flightData = load_flight_data_parallel(baseLogDir, baseUlgDir)

# 결과 확인
print("Data loading completed.")
# 모든 비행의 평균·최대·최소 수집
stats = []
for it, cases in flightData.items():
    for tc, res in cases.items():
        if "error" in res:
            continue
        stats.append({
            "iteration": it,
            "testCase": tc,
            "mean": res["mean_gps_hpos"],
            "max":  res["max_gps_hpos"],
            "min":  res["min_gps_hpos"]
        })

stats_df = pd.DataFrame(stats)
overall_mean = stats_df["mean"].mean()
overall_max  = stats_df["max"].max()
overall_min  = stats_df["min"].min()

print("=== 전체 비행 GPS hpos 통계 ===")
print(f"Average of means: {overall_mean:.3f}")
print(f"Maximum of maxima: {overall_max:.3f}")
print(f"Minimum of minima: {overall_min:.3f}")


