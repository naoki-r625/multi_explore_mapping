# ギャップ抽出の移植仕様書

MATLAB で検討・目視確認した LiDAR ギャップ抽出アルゴリズムを、ROS 2 のセンサーベース探査ノードへ移植するための仕様。
**正解(source of truth)は `scan_gap_viewer.m` の `extractGaps` とその部品関数**。この文書は読みやすくするための要約であり、食い違いがあれば MATLAB を優先する。

実装は [`include/multi_explore_mapping/gap_extraction.hpp`](../include/multi_explore_mapping/gap_extraction.hpp) / [`src/gap_extraction.cpp`](../src/gap_extraction.cpp)。
golden データ(`test/golden_gap_data/`)との照合は [`test/test_gap_extraction.cpp`](../test/test_gap_extraction.cpp) で行う(28フレーム・297ギャップ全一致を確認済み)。

## 0. 進め方(Claude Code への指示)

1. **まず既存コードを読む。** 現在のギャップ抽出(隣り合うサンプルの距離差の中心を目標にする方式、5度クラスタ)がどのノード・関数にあるか、その入出力、後段(コスト関数による目標選択)がギャップをどう受け取っているかを調べる。
2. 調べた結果と、置き換え方針(どのファイルをどう変えるか)を**先に提示して確認を取る**。言語(C++ / Python)とビルド方式は既存コードに合わせる。
3. 実装後、第7章の検証を行う。コスト関数・移動制御・パラメータの調整は**このタスクの範囲外**なので変更しない。

## 1. 目的

- 現行方式の問題: 1サンプルの誤差(欠測・スパイク)で誤抽出する。5度クラスタにしても改善が不十分。
- 新方式: 開口(inf の連続)と段差(距離の急変)の2種類を、ノイズ除去と足切りを入れて抽出する。
- 後段のコスト関数は変更しない。新しい抽出結果を、現行のギャップ表現に合わせて渡す。

## 2. 入出力

入力: `sensor_msgs/LaserScan` 1フレーム

| 項目 | 使い方 |
|---|---|
| `ranges` | 距離 [m]。inf(反射なし)・NaN を含む |
| `angle_min`, `angle_increment` | 各サンプルの角度 `th[i] = angle_min + i * angle_increment` |
| `range_min`, `range_max` | 有効距離の範囲 |

出力: ギャップのリスト(この順に出す: **open を先、jump を後**)

| フィールド | 内容 |
|---|---|
| `idx_start`, `idx_end` | ギャップのサンプル番号(open: inf 区間の最初と最後 / jump: 近い点と遠い点) |
| `target` | 目標点 [x, y](センサ座標系: x 前方, y 左)。既存コードが使う座標系(base_link など)へ変換して渡す |
| `type` | `open` または `jump` |
| `extent` | open: 開口の幅 [m] / jump: 距離差 [m] |

## 3. 前提(要確認)

- **MATLAB 版は 360 度スキャンを前提**にしている(先頭と末尾を隣り合うサンプルとして扱う)。実機の LiDAR が全周でない場合は、先頭と末尾を隣接とみなさない処理が必要。既存コードで画角を確認し、全周でなければ報告すること。
- `angle_increment > 0`、かつ `lambda > angle_increment` を前提とする。
- 検証に使ったデータの LiDAR: 約 1 度刻み・range_max 8 m・-180〜180 度。

## 4. アルゴリズム(0 始まりの擬似コード)

`n` = サンプル数、`inc` = `angle_increment`。浮動小数点の `inf - inf = NaN` は例外にせず、比較は false として扱う。

### 4.1 前処理

```
r[i] = inf   if isnan(r[i]) or r[i] < range_min or r[i] > range_max

# 巻き込みあり・窓幅 w(奇数)の中央値。h = (w-1)/2
rf[i]    = median( r[(i-h) mod n], ..., r[(i+h) mod n] )
isOpen[i] = isinf(rf[i])
```

- `[壁 inf 壁]` は壁に戻り(1サンプルの欠測を埋める)、`[inf 壁 inf]` は inf になる(1サンプルのスパイクを消す)。段差の位置はずれない。
- 中央値は、奇数窓なら中央の要素そのもの(平均しない)。

### 4.2 連続性の判定 `brk`

```
nx[i]     = (i + 1) mod n
dMaxO[i]  = min(rf[i], rf[nx[i]]) * sin(inc) / sin(lambda - inc) + 3 * sigma
brk[i]    = isOpen[i] or isOpen[nx[i]] or |rf[nx[i]] - rf[i]| > dMaxO[i]
```

`brk[i] = true` は「サンプル i と次のサンプルの間で面が途切れている」の意味。ABD(Adaptive Breakpoint Detector)の閾値を使っている。

### 4.3 並べ替え

360 度をまたぐギャップを1つの連続として扱うため、先頭が必ず非 inf になるように回転する。

```
k0 = 最初の !isOpen[i] の i        # 無ければ空のリストを返して終了
idx[m] = (k0 + m) mod n   (m = 0..n-1)
idx[n] = k0                        # 末尾に先頭を1つ足す(長さ n+1)

ro[m] = isOpen[idx[m]]
rr[m] = rf[idx[m]]
xx[m] = rr[m] * cos(th[idx[m]]),  yy[m] = rr[m] * sin(th[idx[m]])   # 非 inf のみ使う
```

`ro[0]` と `ro[n]` は必ず false なので、inf の区間は 1..n-1 の中に収まる。

### 4.4 開口ギャップ(open)

`ro` が true の連続区間 `[a, b]`(両端を含む)ごとに:

```
skip  if (b - a + 1) < minOpenSamples
pA = (xx[a-1], yy[a-1]);  pB = (xx[b+1], yy[b+1])     # 区間の両側の反射点
width = |pA - pB|
skip  if width < minGapWidth
if supportOpenEdges:
    skip if countSupport(idx[a-1], dir=-1) < minSupport
         or countSupport(idx[b+1], dir=+1) < minSupport

target = (pA + pB) / 2
target = target + targetPush * target / |target|       # ロボットから見て奥へ押し出す
出力: idx_start = idx[a], idx_end = idx[b], type = open, extent = width
```

2点を結ぶ線上は、間のレイが全て射程内に障害物なしなので空きだと言える。

### 4.5 段差ギャップ(jump)

`j = 0..n-1` の隣接ペア `(rr[j], rr[j+1])` ごとに:

```
rA = rr[j], rB = rr[j+1]
both  = !ro[j] and !ro[j+1]
dMax  = min(rA, rB) * sin(inc) / sin(lambda - inc) + 3 * sigma
diffR = |rB - rA|
isJump = both and diffR > dMax and diffR >= minJump
```

`isJump` のペアごとに:

```
if rA <= rB: near = j,   far = j+1,  dirNear = -1, dirFar = +1
else:        near = j+1, far = j,    dirNear = +1, dirFar = -1

skip  if countSupport(idx[near], dirNear) < minSupport
      or countSupport(idx[far],  dirFar)  < minSupport

pNear = (xx[near], yy[near]);  pFar = (xx[far], yy[far])

jumpTargetMode == "between":
    target = pNear + jumpRatio * (pFar - pNear)
jumpTargetMode == "corner":
    radial = pNear / |pNear|;  tang = (-radial.y, radial.x)
    if dot(tang, pFar - pNear) < 0: tang = -tang
    target = pNear + cornerOffset * tang

出力: idx_start = idx[near], idx_end = idx[far], type = jump, extent = diffR
```

### 4.6 `countSupport(s, dir)` : 途切れず連続するサンプルの数

```
c = 1
while c < minSupport:
    if dir > 0: t = (s + 1) mod n;       cut = brk[s]
    else:       t = (s - 1 + n) mod n;   cut = brk[t]
    if cut: break
    c += 1;  s = t
return c
```

`s` は **元のスキャンの番号**(`idx[...]` を通した値)を渡す。意図は「壁に浅い角度で当たってサンプルが疎な所は、全部の間で `brk` が true になり、連続数が1のまま」なので、そこから出る段差ギャップを捨てられること。

## 5. パラメータ(ROS 2 パラメータとして宣言する)

| 名前(例) | 初期値 | 意味 |
|---|---|---|
| `median_win` | 3 | 中央値フィルタの窓幅(奇数にそろえる: `2*floor(w/2)+1`) |
| `min_open_samples` | 3 | inf がこの数以上連続したら開口とみなす |
| `min_gap_width` | 0.60 m | 開口の最小幅(ロボット幅 + マージン) |
| `target_push` | 0.50 m | 開口の目標点を奥へ押し出す距離 |
| `abd_lambda_deg` | 10 deg | ABD の入射角。内部ではラジアンに変換して使う |
| `abd_sigma` | 0.01 m | ABD の測距ノイズ |
| `min_jump` | 0.50 m | 段差とみなす最小の距離差 |
| `min_support` | 3 | 段差の両側に必要な、途切れず連続したサンプル数 |
| `support_open_edges` | false | true で開口の両端にも同じ条件を課す |
| `jump_target_mode` | "between" | 段差の目標点: "between" または "corner" |
| `jump_ratio` | 0.5 | "between" 用。0 = 近い点、1 = 遠い点 |
| `corner_offset` | 0.35 m | "corner" 用。角から遠い側へずらす距離 |

実機とシミュレーションで値を変えられるよう、ハードコードせず YAML から読めるようにする。

実際の ROS2 パラメータ名は `sensor_explore_node` / `sensor_mb_explore_node` 側で `gap_` プレフィックスを付けている
(`gap_median_win`, `gap_min_open_samples`, `gap_min_width`, `gap_target_push`, `gap_abd_lambda_deg`,
`gap_abd_sigma`, `gap_min_jump`, `gap_min_support`, `gap_support_open_edges`, `gap_jump_target_mode`,
`gap_jump_ratio`, `gap_corner_offset`)。デフォルト値はすべて上表(golden データでの検証に使った値)と一致している。

## 6. 実装上の注意

- MATLAB は 1 始まり、C++ / Python は 0 始まり。上の擬似コードは 0 始まり。正解データの `idx_start`, `idx_end` は **1 始まり**なので、比較時に 1 を引く。
- `ranges` は float32。MATLAB 版は double に変換して計算している。結果の比較は誤差を許容する(第7章)。
- 毎フレーム呼ばれる。`n` は 360 前後なので計算量は問題にならないが、フレームごとの動的メモリ確保は避ける。
- 目標点の座標系はセンサ座標系。既存コードが期待する座標系への変換を忘れない。

## 7. 検証

### 7.1 正解データとの比較

MATLAB で書き出した `golden_*.csv`(`golden_export_snippet.m` で作成)を使い、単体テストにする。

- `golden_scans.csv`: 各行 = `frame, angle_min, angle_inc, range_min, range_max, ranges...`
- `golden_gaps.csv`: 各行 = `frame, type(1=open, 2=jump), idx_start, idx_end, target_x, target_y, extent`
- `golden_counts.csv`: 各行 = `frame, ギャップ数`(ギャップが 0 個のフレームの確認用)
- `golden_params.json`: 書き出し時のパラメータ

ranges には `Inf` / `NaN` が文字列で入っている。パーサが読めることを確認する。

合格条件: 全フレームでギャップ数が一致し、順序どおりに `type`、`idx`(1 を引いて比較)が一致し、`target` と `extent` の差が 1e-4 m 以下。

**結果**: 28フレーム・297ギャップ全て一致(`test_gap_extraction`実行済み)。

### 7.2 合成データでの単体テスト

360 サンプル・1 度刻み・range_max 8 m の合成スキャンで次を確認する(Python で同じ手順を実装して確認済み)。

| ケース | 期待する結果 |
|---|---|
| 1 サンプルだけ inf(欠測) | ギャップが出ない |
| 1 サンプルだけ遠くにあるスパイク | ギャップが出ない |
| 30 サンプル連続の inf(2 m の面と 4 m の面の間) | open が 1 個 |
| 2 m の面と 5 m の面が接する段差(各面 30 サンプル) | jump が 1 個 |
| 段差が ±180 度をまたぐ位置にある | jump が 1 個(巻き込みで検出) |
| 浅い角度の壁(2 サンプルずつ疎に並ぶ)、`min_support = 3` | その壁の段差は全て捨てられる。`min_support = 1` では 10 個出る |

### 7.3 実データ

rosbag を再生して新旧の抽出結果を比べる。今回の変更で期待する変化は、1 サンプル誤差による誤抽出と、細い通路での連続したギャップが減ることである。

## 8. 範囲外

- コスト関数・移動制御・パラメータ調整は変更しない。
- 他ロボットや自機の点を除外する処理は今回は入れない(別タスク)。
