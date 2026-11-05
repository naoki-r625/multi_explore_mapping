#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

/**
 * LiDARギャップ抽出。docs/gap_extraction_spec.md (正解 = scan_gap_viewer.m の
 * extractGaps) の移植。中央値フィルタでノイズ(1サンプルの欠測/スパイク)を
 * 除去した上で、ABD(Adaptive Breakpoint Detector)により面の連続性を判定し、
 * 開口(inf の連続)と段差(距離の急変)の2種類のギャップを抽出する。
 *
 * 出力順序は MATLAB 版と同じ「open を先、jump を後」(抽出時の走査順のまま、
 * 方位角での並べ替えはしない)。golden データとの比較はこの順序を前提にする。
 * 方位角でのソートや既存の GapTarget{angle,dist,width} への変換は
 * sensor_processor.cpp 側の責務(コスト関数より下流は変更しない、仕様書8章)。
 */
namespace gap_extraction {

struct Params {
    int median_win = 3;              // 中央値フィルタの窓幅(奇数に丸める)
    int min_open_samples = 3;        // inf がこの数以上連続したら開口
    double min_gap_width = 0.6;      // 開口の最小幅 [m]
    double target_push = 0.5;        // 開口の目標点を奥へ押し出す距離 [m]
    double abd_lambda_deg = 10.0;    // ABD の入射角 [deg]
    double abd_sigma = 0.01;         // ABD の測距ノイズ [m]
    double min_jump = 0.5;           // 段差とみなす最小の距離差 [m]
    int min_support = 3;             // 段差/開口端に要求する連続サンプル数
    bool support_open_edges = false; // 開口の両端にも min_support を課すか

    enum class JumpTargetMode { kBetween, kCorner };
    JumpTargetMode jump_target_mode = JumpTargetMode::kBetween;
    double jump_ratio = 0.5;         // "between": 0=近い点, 1=遠い点
    double corner_offset = 0.35;     // "corner": 角から遠い側へずらす距離 [m]
};

enum class GapType { kOpen, kJump };

struct Gap {
    int idx_start;      // 0始まり、元のスキャン番号
    int idx_end;        // 0始まり、元のスキャン番号
    GapType type;
    double target_x;    // センサ座標系 [m] (x: 前方, y: 左)
    double target_y;
    double extent;      // open: 開口幅 [m] / jump: 距離差 [m]
};

// 1フレーム分のLaserScanからギャップを抽出する。360度スキャン(先頭と末尾が
// 隣接)を前提にしている。有効なギャップが1つもない場合は空のvectorを返す。
std::vector<Gap> extract_gaps(const sensor_msgs::msg::LaserScan& scan, const Params& params);

}  // namespace gap_extraction
