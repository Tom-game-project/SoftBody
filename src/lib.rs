use num_traits::{AsPrimitive, FromPrimitive};
use std::fmt::Debug;

struct Point{
    //mass:i32,
    position:(f32, f32),
    //velocity:(i32, i32),
}

struct SoftBody{
    shape: Vec<Point>,
    // 他のフィールド
}

impl SoftBody {
    /// SoftBodyの中心座標を基準に、指定された位置へ移動させる
    pub fn move_to(&mut self, new_center: (f32, f32)) {
        // 現在の中心を計算 (全頂点の平均)
        let mut current_center = (0.0, 0.0);
        for p in &self.shape {
            current_center.0 += p.position.0;
            current_center.1 += p.position.1;
        }
        let num_points = self.shape.len() as f32;
        current_center.0 /= num_points;
        current_center.1 /= num_points;

        // 移動量を計算
        let dx = new_center.0 - current_center.0;
        let dy = new_center.1 - current_center.1;

        // 全ての点を移動
        for p in &mut self.shape {
            p.position.0 += dx;
            p.position.1 += dy;
        }
    }
}

#[derive(Debug, Copy, Clone)]
struct Line<T> {
    start: (T, T),
    end: (T, T),
}

/// スライスの要素を循環的にペアリングするイテレータ。
/// 例: `[a, b, c]` -> `(a, b), (b, c), (c, a)`
#[derive(Debug)]
pub struct CircularWindows<'a, T> {
    slice: &'a [T],
    index: usize,
}

// Iteratorトレイトを実装
impl<'a, T> Iterator for CircularWindows<'a, T> {
    // イテレータが返す値の型。要素のペアへの参照。
    type Item = (&'a T, &'a T);

    fn next(&mut self) -> Option<Self::Item> {
        // スライスが空、またはすべての要素を処理し終えたら終了
        if self.slice.is_empty() || self.index >= self.slice.len() {
            return None;
        }

        // 現在の要素への参照を取得
        let first = &self.slice[self.index];

        // 次の要素のインデックスを計算（末尾なら先頭に戻る）
        let second_index = (self.index + 1) % self.slice.len();
        let second = &self.slice[second_index];

        // 次の呼び出しのためにインデックスを進める
        self.index += 1;

        // ペアを返す
        Some((first, second))
    }
}

/// `circular_windows()` メソッドをスライスで使えるようにするための拡張トレイト
pub trait CircularWindowsExt<T> {
    fn circular_windows(&self) -> CircularWindows<'_, T>;
}

// すべてのスライス `[T]` に対して上記トレイトを実装
impl<T> CircularWindowsExt<T> for [T] {
    fn circular_windows(&self) -> CircularWindows<'_, T> {
        CircularWindows {
            slice: self,
            index: 0,
        }
    }
}

/// 2つの線分の交点を計算します（ジェネリック版）。
///
/// # 型パラメータ
/// * `T`: 線分の座標を表す数値プリミティブ型
///
/// # トレイト境界
/// * `T: Copy + Debug`: 値のコピーとデバッグ表示が可能であること。
/// * `T: AsPrimitive<f64>`: 計算のために `f64` に変換可能であること。
/// * `T: FromPrimitive`: 計算結果の `f64` から元の型 `T` に変換可能であること。
fn inter_section<T>(l1: Line<T>, l2: Line<T>) -> Option<(T, T)>
where
    T: Copy + Debug + AsPrimitive<f64> + FromPrimitive,
{
    // T 型を計算用の f64 型に変換
    let (x1, y1) = (l1.start.0.as_(), l1.start.1.as_());
    let (x2, y2) = (l1.end.0.as_(), l1.end.1.as_());
    let (x3, y3) = (l2.start.0.as_(), l2.start.1.as_());
    let (x4, y4) = (l2.end.0.as_(), l2.end.1.as_());

    let dx1 = x2 - x1;
    let dy1 = y2 - y1;
    let dx2 = x4 - x3;
    let dy2 = y4 - y3;

    let denominator = dx1 * dy2 - dy1 * dx2;

    if denominator == 0.0 {
        return None;
    }

    let t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / denominator;
    let u = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / denominator;

    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        let intersect_x = x1 + t * dx1;
        let intersect_y = y1 + t * dy1;
        // 計算結果の f64 を元の型 T に安全に変換
        // FromPrimitive::from_f64 は Option<T> を返すため、`?` で変換失敗時に None を返す
        let x = T::from_f64(intersect_x.round())?;
        let y = T::from_f64(intersect_y.round())?;
        Some((x, y))
    } else {
        None
    }
}

/// SoftBodyと線分のすべての交点を計算する
fn find_all_intersections(softbody: &SoftBody, line: &Line<f32>) -> Vec<(f32, f32)> {
    let mut points = Vec::new();

    // SoftBodyの各辺をイテレート
    for (p1, p2) in softbody.shape.circular_windows() {
        let body_segment = Line { start: p1.position, end: p2.position };
        
        // 交点があれば結果のVecに追加
        if let Some(intersection_point) = inter_section::<f32>(*line, body_segment) {
            points.push(intersection_point);
        }
    }
    points
}

/// 点と線分の間の最短距離の2乗と、線分上の最近傍点を計算する
fn distance_sq_point_to_segment(point: (f32, f32), segment: Line<f32>) -> (f32, (f32, f32)) {
    let (px, py) = point;
    let (x1, y1) = segment.start;
    let (x2, y2) = segment.end;

    let dx = x2 - x1;
    let dy = y2 - y1;

    // 線分の長さの2乗が0、つまり点である場合
    if dx == 0.0 && dy == 0.0 {
        let dist_sq = (px - x1).powi(2) + (py - y1).powi(2);
        return (dist_sq, (x1, y1));
    }

    // 線分上の最近傍点の射影パラメータtを計算
    // t = ((P - A)・(B - A)) / |B - A|^2
    let t = ((px - x1) * dx + (py - y1) * dy) / (dx.powi(2) + dy.powi(2));

    let closest_point = if t < 0.0 {
        // 始点Aが最近傍
        (x1, y1)
    } else if t > 1.0 {
        // 終点Bが最近傍
        (x2, y2)
    } else {
        // 線分上の点が最近傍
        (x1 + t * dx, y1 + t * dy)
    };

    let dist_sq = (px - closest_point.0).powi(2) + (py - closest_point.1).powi(2);
    (dist_sq, closest_point)
}

/// ある点に最も近いSoftBody上の辺（線分）を見つける
fn find_nearest_segment(softbody: &SoftBody, point: (f32, f32)) -> Option<(Line<f32>, (f32, f32))> {
    if softbody.shape.is_empty() {
        return None;
    }

    softbody
        .shape
        .circular_windows()
        .map(|(p1, p2)| {
            let segment = Line { start: p1.position, end: p2.position };
            let (dist_sq, closest_point) = distance_sq_point_to_segment(point, segment);
            (dist_sq, segment, closest_point)
        })
        // `min_by`で最小の距離を持つ要素を見つける
        .min_by(|(d1, _, _), (d2, _, _)| d1.partial_cmp(d2).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(_, segment, closest_point)| (segment, closest_point))
}


#[cfg(test)]
mod tests {
    use super::*;
    use macroquad::prelude::*;

    #[test]
    fn it_works01()
    {
        println!("--- i32 の場合 ---");
        // i32 でテスト
        let line1_i32 = Line { start: (0, 0), end: (10, 10) };
        let line2_i32 = Line { start: (0, 10), end: (10, 0) };
        println!("交差する例: {:?}", inter_section(line1_i32, line2_i32)); // Expected: Some((5, 5))

        println!("\n--- f64 の場合 ---");
        // f64 でテスト
        let line1_f64 = Line { start: (0.0, 0.0), end: (10.0, 10.0) };
        let line2_f64 = Line { start: (0.0, 10.0), end: (10.0, 0.0) };
        println!("交差する例: {:?}", inter_section(line1_f64, line2_f64)); // Expected: Some((5.0, 5.0))

        // f64 で小数点を含むテスト
        let line3_f64 = Line { start: (1.5, 2.5), end: (8.5, 9.5) };
        let line4_f64 = Line { start: (1.5, 9.5), end: (8.5, 2.5) };
        println!("交差する例 (小数点): {:?}", inter_section(line3_f64, line4_f64)); // Expected: Some((5.0, 6.0))
    }

    #[test]
    fn test_circular()
    {
        let data = vec![(0, 0), (1, 0), (1, 1), (0, 1)];

        println!("要素数が4の場合:");
        // 拡張トレイトにより、どんなスライス/Vecからでも呼び出せる
        for (current, next) in data.circular_windows() {
            println!("  Comparing {:?} and {:?}", current, next);
        }
        // 出力:
        //   Comparing (0, 0) and (1, 0)
        //   Comparing (1, 0) and (1, 1)
        //   Comparing (1, 1) and (0, 1)
        //   Comparing (0, 1) and (0, 0)

        println!("\n要素数が1の場合:");
        let single_element = vec![(10, 10)];
        for (current, next) in single_element.circular_windows() {
            // 自分自身とペアになる
            println!("  Comparing {:?} and {:?}", current, next);
        }
        // 出力:
        //   Comparing (10, 10) and (10, 10)

        println!("\n要素数が0の場合:");
        let empty: Vec<(i32, i32)> = vec![];
        for (current, next) in empty.circular_windows() {
            // ループの中は実行されない
            println!("  Comparing {:?} and {:?}", current, next);
        }
        println!("  (ループは実行されません)");
    }

    /// SoftBodyを描画するヘルパー関数
    fn draw_softbody(body: &SoftBody, color: Color) {
        if body.shape.len() < 2 {
            return;
        }
        // 隣り合う点同士を線で結ぶ
        for i in 0..body.shape.len() {
            let p1 = body.shape[i].position;
            let p2 = body.shape[(i + 1) % body.shape.len()].position; // 循環させる
            draw_line(p1.0, p1.1, p2.0, p2.1, 2.0, color);
        }
    }

    /// インタラクティブなテストウィンドウを起動する
    async fn interactive_test_main() {
        // テスト用のSoftBodyを2つ準備
        let mut body1 = SoftBody {
            shape: vec![
                Point { position: (100.0, 100.0) },
                Point { position: (300.0, 100.0) },
                Point { position: (300.0, 250.0) },
                Point { position: (100.0, 250.0) },
            ],
        };

        let mut body2 = SoftBody {
            shape: vec![
                Point { position: (400.0, 300.0) },
                Point { position: (600.0, 300.0) },
                Point { position: (550.0, 450.0) },
            ],
        };

        loop {
            // --- 更新処理 ---
            // body2をマウスカーソルの位置に追従させる
            let mouse_pos = mouse_position();
            body2.move_to(mouse_pos);

            // ここに重なり判定などのテストしたいロジックを記述する
            // let is_colliding = check_collision(&body1, &body2);

            // --- 描画処理 ---
            clear_background(WHITE);

            draw_text("Interactive Test: Press ESC to finish", 20.0, 20.0, 20.0, DARKGRAY);

            // body1を描画 (固定)
            draw_softbody(&body1, BLUE);

            // body2を描画 (マウスに追従)
            draw_softbody(&body2, RED);

            // (オプション) 重なり判定の結果を描画
            // if is_colliding {
            //     draw_text("COLLIDING!", 350.0, 40.0, 40.0, RED);
            // }

            // Escキーでテスト終了
            if is_key_pressed(KeyCode::Escape) {
                break;
            }

            next_frame().await;
        }
    }

    async fn interactive_test_main01() {
        // テスト用のSoftBodyを準備
        let softbody = SoftBody {
            shape: vec![
                // 頂点を順番に結んでいく
                Point { position: (400.0, 100.0) }, // 外側の頂点 1 (上)
                Point { position: (350.0, 250.0) }, // 内側の頂点 1 (凹み)
                Point { position: (500.0, 300.0) }, // 外側の頂点 2 (右)
                Point { position: (400.0, 350.0) }, // 内側の頂点 2 (凹み)
                Point { position: (300.0, 500.0) }, // 外側の頂点 3 (下)
                Point { position: (250.0, 350.0) }, // 内側の頂点 3 (凹み)
                Point { position: (100.0, 300.0) }, // 外側の頂点 4 (左)
                Point { position: (250.0, 250.0) }, // 内側の頂点 4 (凹み)
            ],
        };

        // メインループ
        loop {
            // --- 更新処理 ---
            let line_start = (400.0, 400.0);
            let mouse_pos = mouse_position();

            let test_line = Line {
                start: line_start,
                end: mouse_pos,
            };

            // SoftBodyとテスト線分の交点をすべて計算
            let intersections = find_all_intersections(&softbody, &test_line);

            // --- 描画処理 ---
            clear_background(WHITE);

            // 1. SoftBodyを描画
            draw_softbody(&softbody, BLUE);

            // 2. テスト用の線分を描画
            draw_line(test_line.start.0, test_line.start.1, test_line.end.0, test_line.end.1, 2.0, RED);
            draw_circle(test_line.start.0, test_line.start.1, 5.0, RED);

            // 3. 見つかったすべての交点を描画
            for point in &intersections {
                draw_circle(point.0, point.1, 8.0, GOLD);
            }

            // 4. ヘルプテキストを描画
            draw_text("Move mouse to find intersections.", 20.0, 30.0, 24.0, DARKGRAY);
            draw_text(&format!("Intersections found: {}", intersections.len()), 20.0, 60.0, 24.0, DARKGRAY);

            next_frame().await;
        }
    }

    async fn interactive_test_main02() {
        let softbody = SoftBody {
            shape: vec![
                Point { position: (400.0, 100.0) },
                Point { position: (350.0, 250.0) },
                Point { position: (500.0, 300.0) },
                Point { position: (400.0, 350.0) },
                Point { position: (300.0, 500.0) },
                Point { position: (250.0, 350.0) },
                Point { position: (100.0, 300.0) },
                Point { position: (250.0, 250.0) },
            ],
        };

        loop {
            let mouse_pos = mouse_position();
            
            // マウス位置に最も近い辺を探索
            let nearest = find_nearest_segment(&softbody, mouse_pos);

            clear_background(WHITE);

            // SoftBodyを描画
            for (p1, p2) in softbody.shape.circular_windows() {
                draw_line(p1.position.0, p1.position.1, p2.position.0, p2.position.1, 3.0, BLUE);
            }

            if let Some((nearest_segment, closest_point)) = nearest {
                // 1. 最近傍の辺をハイライト
                draw_line(
                    nearest_segment.start.0, nearest_segment.start.1,
                    nearest_segment.end.0, nearest_segment.end.1,
                    4.0, // 少し太くする
                    GREEN,
                );

                // 2. マウスから最近傍点への線を描画
                draw_line(mouse_pos.0, mouse_pos.1, closest_point.0, closest_point.1, 1.0, GRAY);
                
                // 3. 最近傍点自体をマーク
                draw_circle(closest_point.0, closest_point.1, 6.0, RED);
            }
            
            draw_text("Move mouse to find the nearest segment", 20.0, 30.0, 24.0, DARKGRAY);

            next_frame().await;
        }
    }

    #[test] 
    fn run_interactive_visualization_test() {
        // macroquadの設定
        let config = Conf {
            window_title: "Interactive SoftBody Test".to_string(),
            window_width: 800,
            window_height: 600,
            ..Default::default()
        };

        // macroquadのウィンドウをテスト内で起動
        macroquad::Window::from_config(config, interactive_test_main());
    }

    #[test] 
    fn run_interactive_visualization_test01() {
        // macroquadの設定
        let config = Conf {
            window_title: "Interactive SoftBody Test".to_string(),
            window_width: 800,
            window_height: 600,
            ..Default::default()
        };

        // macroquadのウィンドウをテスト内で起動
        macroquad::Window::from_config(config, interactive_test_main01());
    }

    #[test] 
    fn run_interactive_visualization_test02() {
        // macroquadの設定
        let config = Conf {
            window_title: "Interactive SoftBody Test".to_string(),
            window_width: 800,
            window_height: 600,
            ..Default::default()
        };

        // macroquadのウィンドウをテスト内で起動
        macroquad::Window::from_config(config, interactive_test_main02());
    }

}
