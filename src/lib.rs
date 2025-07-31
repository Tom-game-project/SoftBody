use num_traits::{AsPrimitive, FromPrimitive};
use std::fmt::Debug;

pub mod circular;
use circular::CircularWindowsExt;

pub struct Point{
    pub mass: f32,
    pub position:(f32, f32),
    pub velocity:(f32, f32),
}

pub struct SoftBody{
    pub shape: Vec<Point>,
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
pub struct Line<T> {
    pub start: (T, T),
    pub end: (T, T),
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
pub fn inter_section<T>(l1: Line<T>, l2: Line<T>) -> Option<(T, T)>
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
pub fn find_all_intersections(softbody: &SoftBody, line: &Line<f32>) -> Vec<(f32, f32)> {
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
pub fn find_nearest_segment(softbody: &SoftBody, point: (f32, f32)) -> Option<(Line<f32>, (f32, f32))> {
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

/// 点A, B, Pを受け取り、Pが直線AB上に移動した後の
/// 各点の新しい状態をタプル (Point, Point, Point) として返します。
///
/// # 引数
/// * `a` - 点Aの参照
/// * `b` - 点Bの参照
/// * `p` - 点Pの参照
pub fn move_p_to_line_ab(a: &Point, b: &Point, p: &Point, _alpha_degrees: f32) -> (Point, Point, Point) {
    // === 1. 初期値の定義 ===
    let pos_a0 = a.position;
    let pos_b0 = b.position;
    let pos_p0 = p.position;

    let m0 = a.mass;
    let m1 = b.mass;
    let m2 = p.mass;
    let total_mass = m0 + m1 + m2;

    // === 2. 垂線の足 Q0 の座標を計算 ===
    let vec_ab = (pos_b0.0 - pos_a0.0, pos_b0.1 - pos_a0.1);
    let vec_ap = (pos_p0.0 - pos_a0.0, pos_p0.1 - pos_a0.1);

    let dot_product = vec_ap.0 * vec_ab.0 + vec_ap.1 * vec_ab.1;
    let len_sq_ab = vec_ab.0 * vec_ab.0 + vec_ab.1 * vec_ab.1;

    // AとBが同じ点にある場合を考慮
    let pos_q0 = if len_sq_ab == 0.0 {
        pos_a0 // Q0はAと同じ位置になる
    } else {
        let t = dot_product / len_sq_ab;
        (pos_a0.0 + t * vec_ab.0, pos_a0.1 + t * vec_ab.1)
    };

    // === 3. P0からQ0へのベクトルを計算 ===
    let vec_p0q0 = (pos_q0.0 - pos_p0.0, pos_q0.1 - pos_p0.1);

    // === 4. 各点の変位（移動）ベクトルを計算 ===
    // AとBの変位ベクトル d = -(m2 / M) * P0Q0
    let factor_ab = -m2 / total_mass;
    let displacement_ab = (factor_ab * vec_p0q0.0, factor_ab * vec_p0q0.1);

    // Pの変位ベクトル dP = (m0 + m1 / M) * P0Q0
    let factor_p = (m0 + m1) / total_mass;
    let displacement_p = (factor_p * vec_p0q0.0, factor_p * vec_p0q0.1);

    // === 5. 新しい位置を計算 ===
    let pos_af = (pos_a0.0 + displacement_ab.0, pos_a0.1 + displacement_ab.1);
    let pos_bf = (pos_b0.0 + displacement_ab.0, pos_b0.1 + displacement_ab.1);
    let pos_pf = (pos_p0.0 + displacement_p.0, pos_p0.1 + displacement_p.1);

    // === 6. 新しいPointオブジェクトを作成して返す ===
    let new_a = Point { mass: m0, velocity:a.velocity, position: pos_af };
    let new_b = Point { mass: m1, velocity:b.velocity, position: pos_bf };
    let new_p = Point { mass: m2, velocity:p.velocity, position: pos_pf };

    (new_a, new_b, new_p)
}



