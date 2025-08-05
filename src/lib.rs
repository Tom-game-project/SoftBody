//! # Soft Body Physics Simulation Library
//!
//! Position Based Dynamics (PBD) のアプローチに基づいています。
//!
//! ## 主な構成要素
//!
//! - `Vec2`: 2次元ベクトル。
//! - `Mat2`: 2x2行列。
//! - `Particle`: 質点。位置、速度、質量などの物理的プロパティを持ちます。
//! - `Spring`: 2つの質点を結ぶバネ。距離ベースの拘束を表現します。
//! - `ShapeMatchingConstraint`: 質点の集合が初期形状を維持しようとする拘束。
//! - `SoftBody`: 質点、バネ、形状維持拘束から構成されるソフトボディ（柔体）。
//! - `Simulation`: シミュレーション全体を管理するコンテナ。重力や境界などのグローバルな設定を持ち、
//!   シミュレーションのステップ実行を担います。
//!
//! ## 使い方
//!
//! 1. `SimulationConfig` でシミュレーションのグローバル設定を定義します。
//! 2. `Simulation::new()` でシミュレーションインスタンスを作成します。
//! 3. `SoftBodyConfig` で作成したいソフトボディの設定を定義します。
//! 4. `simulation.add_soft_body()` でシミュレーションにソフトボディを追加します。
//! 5. ループ内で `simulation.step()` を呼び出し、シミュレーションを時間経過させます。
//! 6. `simulation.particles()` などからシミュレーションの状態を取得し、描画や分析に利用します。
//!
//! ```no_run
//! use soft_body_sim::core::*;
//!
//! // 1. シミュレーション設定
//! let config = SimulationConfig {
//!     gravity: Vec2::new(0.0, 980.0),
//!     solver_iterations: 8,
//!     ..Default::default()
//! };
//!
//! // 2. シミュレーション作成
//! let mut sim = Simulation::new(config);
//!
//! // 3. ソフトボディ設定
//! let body_config = SoftBodyConfig {
//!     center: Vec2::new(300.0, 100.0),
//!     size: Vec2::new(100.0, 100.0),
//!     rows: 5,
//!     cols: 5,
//!     stiffness: 0.5,
//!     shape_stiffness: 0.1,
//!     ..Default::default()
//! };
//!
//! // 4. ソフトボディ追加
//! sim.add_soft_body(&body_config);
//!
//! // 5. シミュレーションループ
//! for _ in 0..100 {
//!     sim.step(1.0 / 60.0);
//! }
//!
//! // 6. 結果の取得
//! for particle in sim.particles() {
//!     println!("Particle at: {:?}", particle.pos);
//! }
//! ```

// モジュールを定義してコードを整理します。
pub mod core {
    use std::ops::{Add, AddAssign, Mul, Sub, SubAssign};

    /// 2次元ベクトルを表す構造体。
    #[derive(Debug, Copy, Clone, PartialEq, Default)]
    pub struct Vec2 {
        pub x: f64,
        pub y: f64,
    }

    impl Vec2 {
        /// 新しい `Vec2` を作成します。
        pub const fn new(x: f64, y: f64) -> Self {
            Self { x, y }
        }

        /// ベクトルの長さを計算します。
        pub fn length(&self) -> f64 {
            self.x.hypot(self.y)
        }

        /// ベクトルの長さの2乗を計算します。
        /// `sqrt` の呼び出しを避けるため、長さの比較などに利用すると高速です。
        pub fn length_squared(&self) -> f64 {
            self.x * self.x + self.y * self.y
        }

        /// ベクトルを正規化（長さを1に）します。
        /// 長さが0の場合はゼロベクトルを返します。
        pub fn normalize(&self) -> Self {
            let len = self.length();
            if len > f64::EPSILON {
                *self * (1.0 / len)
            } else {
                Vec2::new(0.0, 0.0)
            }
        }

        /// 2つのベクトルの内積を計算します。
        pub fn dot(a: Self, b: Self) -> f64 {
            a.x * b.x + a.y * b.y
        }

        /// 2つのベクトルの外積（2Dではスカラー値）を計算します。
        pub fn cross(a: Self, b: Self) -> f64 {
            a.x * b.y - a.y * b.x
        }
    }

    // --- 演算子のオーバーロード ---
    impl Add for Vec2 {
        type Output = Self;
        fn add(self, rhs: Self) -> Self::Output {
            Self::new(self.x + rhs.x, self.y + rhs.y)
        }
    }

    impl AddAssign for Vec2 {
        fn add_assign(&mut self, rhs: Self) {
            self.x += rhs.x;
            self.y += rhs.y;
        }
    }

    impl Sub for Vec2 {
        type Output = Self;
        fn sub(self, rhs: Self) -> Self::Output {
            Self::new(self.x - rhs.x, self.y - rhs.y)
        }
    }

    impl SubAssign for Vec2 {
        fn sub_assign(&mut self, rhs: Self) {
            self.x -= rhs.x;
            self.y -= rhs.y;
        }
    }

    impl Mul<f64> for Vec2 {
        type Output = Self;
        fn mul(self, rhs: f64) -> Self::Output {
            Self::new(self.x * rhs, self.y * rhs)
        }
    }

    /// 2x2 行列を表す構造体。列ベクトルでデータを保持します。
    #[derive(Debug, Copy, Clone, PartialEq, Default)]
    pub struct Mat2 {
        pub c1: Vec2, // 1列目
        pub c2: Vec2, // 2列目
    }

    impl Mat2 {
        /// 新しい `Mat2` を作成します。
        pub const fn new(c1: Vec2, c2: Vec2) -> Self {
            Self { c1, c2 }
        }

        /// 行列とベクトルの乗算を行います。
        pub fn mul_vec(&self, v: Vec2) -> Vec2 {
            Vec2::new(
                self.c1.x * v.x + self.c2.x * v.y,
                self.c1.y * v.x + self.c2.y * v.y,
            )
        }

        /// 行列の極分解（Polar Decomposition）を行い、回転行列 `R` を抽出します。
        /// このメソッドは、形状維持拘束（Shape Matching）において、
        /// 変形した形状から最適な回転を求めるために使用されます。
        pub fn polar_decomposition(&self) -> Self {
            // Shoemake's method for robust polar decomposition
            let x = Vec2::new(self.c1.x + self.c2.y, self.c1.y - self.c2.x);
            let y = Vec2::new(self.c1.x - self.c2.y, self.c1.y + self.c2.x);
            let len_x = x.length();
            let len_y = y.length();

            if len_x > len_y {
                let inv_len_x = 1.0 / len_x;
                let c1 = Vec2::new(x.x * inv_len_x, x.y * inv_len_x);
                let c2 = Vec2::new(-x.y * inv_len_x, x.x * inv_len_x);
                Mat2::new(c1, c2)
            } else if len_y > f64::EPSILON {
                let inv_len_y = 1.0 / len_y;
                let c1 = Vec2::new(y.x * inv_len_y, y.y * inv_len_y);
                let c2 = Vec2::new(y.y * inv_len_y, -y.x * inv_len_y);
                Mat2::new(c1, c2)
            } else {
                // A がゼロ行列の場合、単位行列を返す
                Mat2::new(Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0))
            }
        }
    }
    
    // --- 演算子のオーバーロード ---
    impl Add for Mat2 {
        type Output = Self;
        fn add(self, rhs: Self) -> Self::Output {
            Self::new(self.c1 + rhs.c1, self.c2 + rhs.c2)
        }
    }

    impl Mul<f64> for Mat2 {
        type Output = Self;
        fn mul(self, rhs: f64) -> Self::Output {
            Self::new(self.c1 * rhs, self.c2 * rhs)
        }
    }
    
    /// 質点を表す構造体。
    #[derive(Debug, Clone, PartialEq)]
    pub struct Particle {
        pub pos: Vec2,
        pub prev_pos: Vec2,
        pub vel: Vec2,
        /// 質量の逆数。`0.0` の場合は固定質点（無限大の質量）を表します。
        pub inv_mass: f64,
        pub radius: f64,
        pub is_fixed: bool,
    }

    impl Particle {
        /// 新しい `Particle` を作成します。
        pub fn new(x: f64, y: f64) -> Self {
            Self {
                pos: Vec2::new(x, y),
                prev_pos: Vec2::new(x, y),
                vel: Vec2::new(0.0, 0.0),
                inv_mass: 1.0,
                radius: 8.0,
                is_fixed: false,
            }
        }
    }

    /// 2つの質点を結ぶバネを表す構造体。距離拘束として機能します。
    ///
    /// 質点への直接の参照を持つ代わりに、シミュレーション全体の質点リストに対する
    /// インデックスを保持することで、Rustの借用規則に準拠します。
    #[derive(Debug, Clone, PartialEq)]
    pub struct Spring {
        pub p1_index: usize,
        pub p2_index: usize,
        pub rest_length: f64,
        pub stiffness: f64,
    }

    impl Spring {
        /// 新しい `Spring` を作成します。
        /// `particles` スライスから初期位置を取得し、静止長を計算します。
        pub fn new(p1_index: usize, p2_index: usize, stiffness: f64, particles: &[Particle]) -> Self {
            let rest_length = (particles[p1_index].pos - particles[p2_index].pos).length();
            Self { p1_index, p2_index, rest_length, stiffness }
        }

        /// バネ拘束を解決し、質点の位置を修正します。
        ///
        /// # Arguments
        ///
        /// * `particles` - シミュレーション内の全質点を含む可変スライス。
        pub fn solve(&self, particles: &mut [Particle]) {
            // インデックスのペアを安全にミュータブルに借用するためのテクニック
            let (p1_slice, p2_slice) = if self.p1_index < self.p2_index {
                let (s1, s2) = particles.split_at_mut(self.p2_index);
                (&mut s1[self.p1_index], &mut s2[0])
            } else {
                let (s1, s2) = particles.split_at_mut(self.p1_index);
                (&mut s2[0], &mut s1[self.p2_index])
            };

            let total_inv_mass = p1_slice.inv_mass + p2_slice.inv_mass;
            if total_inv_mass < f64::EPSILON {
                return;
            }

            let diff = p1_slice.pos - p2_slice.pos;
            let dist = diff.length();
            if dist < f64::EPSILON {
                return;
            }

            let correction = diff * ((dist - self.rest_length) / dist);
            let correction_vec = correction * (self.stiffness / total_inv_mass);

            p1_slice.pos -= correction_vec * p1_slice.inv_mass;
            p2_slice.pos += correction_vec * p2_slice.inv_mass;
        }
    }

    /// 形状維持拘束（Shape Matching Constraint）を表す構造体。
    /// 質点の集合が初期形状を維持しようとする力をモデル化します。
    #[derive(Debug, Clone, PartialEq)]
    pub struct ShapeMatchingConstraint {
        pub particle_indices: Vec<usize>,
        pub stiffness: f64,
        /// 初期形状における、重心からの相対位置ベクトル群。
        initial_shape: Vec<Vec2>,
        /// 現在のフレームでの重心。
        center_of_mass: Vec2,
    }

    impl ShapeMatchingConstraint {
        /// 新しい形状維持拘束を作成します。
        pub fn new(particle_indices: Vec<usize>, stiffness: f64, particles: &[Particle]) -> Self {
            let mut initial_shape = Vec::with_capacity(particle_indices.len());
            
            // 初期形状の重心を計算
            let mut center = Vec2::new(0.0, 0.0);
            let mut total_mass = 0.0;
            for &i in &particle_indices {
                let p = &particles[i];
                let mass = if p.inv_mass > f64::EPSILON { 1.0 / p.inv_mass } else { 0.0 };
                center += p.pos * mass;
                total_mass += mass;
            }

            let initial_center = if total_mass > f64::EPSILON {
                center * (1.0 / total_mass)
            } else {
                Vec2::new(0.0, 0.0)
            };

            // 重心からの相対位置を保存
            for &i in &particle_indices {
                initial_shape.push(particles[i].pos - initial_center);
            }

            Self {
                particle_indices,
                stiffness,
                initial_shape,
                center_of_mass: initial_center,
            }
        }
        
        /// 現在の重心を計算して更新します。
        fn calculate_center_of_mass(&mut self, particles: &[Particle]) {
            let mut center = Vec2::new(0.0, 0.0);
            let mut total_mass = 0.0;
            for &i in &self.particle_indices {
                let p = &particles[i];
                let mass = if p.inv_mass > f64::EPSILON { 1.0 / p.inv_mass } else { 0.0 };
                center += p.pos * mass;
                total_mass += mass;
            }
            self.center_of_mass = if total_mass > f64::EPSILON {
                center * (1.0 / total_mass)
            } else {
                self.center_of_mass // 質量がない場合は動かさない
            };
        }

        /// 形状維持拘束を解決し、質点の位置を修正します。
        pub fn solve(&mut self, particles: &mut [Particle]) {
            self.calculate_center_of_mass(particles);

            let mut a_pq = Mat2::default();
            for (i, &p_idx) in self.particle_indices.iter().enumerate() {
                let q = self.initial_shape[i]; // 初期形状の相対ベクトル
                let p = particles[p_idx].pos - self.center_of_mass; // 現在の相対ベクトル
                
                a_pq.c1.x += p.x * q.x;
                a_pq.c1.y += p.y * q.x;
                a_pq.c2.x += p.x * q.y;
                a_pq.c2.y += p.y * q.y;
            }

            let r = a_pq.polar_decomposition();

            for (i, &p_idx) in self.particle_indices.iter().enumerate() {
                let particle = &mut particles[p_idx];
                if particle.is_fixed {
                    continue;
                }

                let goal_pos = self.center_of_mass + r.mul_vec(self.initial_shape[i]);
                let correction = (goal_pos - particle.pos) * self.stiffness;
                particle.pos += correction;
            }
        }
    }

    /// ソフトボディを構成する要素の集合。
    /// 実際の質点データは `Simulation` が所有し、`SoftBody` はインデックスで管理します。
    #[derive(Debug, Clone)]
    pub struct SoftBody {
        pub particle_indices: Vec<usize>,
        pub springs: Vec<Spring>,
        pub shape_constraint: Option<ShapeMatchingConstraint>,
    }

    /// シミュレーション全体の環境と状態を管理する構造体。
    #[derive(Debug, Clone)]
    pub struct Simulation {
        pub particles: Vec<Particle>,
        soft_bodies: Vec<SoftBody>,
        config: SimulationConfig,
    }
    
    /// `SoftBody` を生成するための設定。ビルダーパターンのように使用します。
    #[derive(Debug, Clone, PartialEq)]
    pub struct SoftBodyConfig {
        pub center: Vec2,
        pub size: Vec2,
        pub rows: usize,
        pub cols: usize,
        pub stiffness: f64,
        pub shape_stiffness: f64,
        pub is_fixed: bool,
        pub particle_radius: f64,
        pub particle_inv_mass: f64,
    }

    impl Default for SoftBodyConfig {
        fn default() -> Self {
            Self {
                center: Vec2::new(0.0, 0.0),
                size: Vec2::new(100.0, 100.0),
                rows: 5,
                cols: 5,
                stiffness: 0.2,
                shape_stiffness: 0.2,
                is_fixed: false,
                particle_radius: 8.0,
                particle_inv_mass: 1.0,
            }
        }
    }

    /// シミュレーションのグローバル設定。
    #[derive(Debug, Clone, PartialEq)]
    pub struct SimulationConfig {
        pub gravity: Vec2,
        pub damping: f64,
        pub solver_iterations: usize,
        /// 境界。`Some(min, max)` で設定。`None` の場合は境界なし。
        pub bounds: Option<(Vec2, Vec2)>,
    }

    impl Default for SimulationConfig {
        fn default() -> Self {
            Self {
                gravity: Vec2::new(0.0, 270.0),
                damping: 0.99,
                solver_iterations: 8,
                bounds: None,
            }
        }
    }

    impl Simulation {
        /// 新しいシミュレーション環境を作成します。
        pub fn new(config: SimulationConfig) -> Self {
            Self {
                particles: Vec::new(),
                soft_bodies: Vec::new(),
                config,
            }
        }

        /// シミュレーションにソフトボディを追加します。
        /// 質点と拘束を生成し、シミュレーションの状態に統合します。
        pub fn add_soft_body(&mut self, config: &SoftBodyConfig) {
            let start_index = self.particles.len();
            let mut particle_indices = Vec::new();

            let spacing_x = if config.cols > 1 { config.size.x / (config.cols - 1) as f64 } else { 0.0 };
            let spacing_y = if config.rows > 1 { config.size.y / (config.rows - 1) as f64 } else { 0.0 };
            let top_left = config.center - Vec2::new(config.size.x * 0.5, config.size.y * 0.5);

            for i in 0..config.rows {
                for j in 0..config.cols {
                    let x = top_left.x + j as f64 * spacing_x;
                    let y = top_left.y + i as f64 * spacing_y;
                    let mut p = Particle::new(x, y);
                    p.radius = config.particle_radius;

                    if config.is_fixed {
                        p.is_fixed = true;
                        p.inv_mass = 0.0;
                    } else {
                         p.inv_mass = config.particle_inv_mass;
                    }
                    
                    particle_indices.push(self.particles.len());
                    self.particles.push(p);
                }
            }
            
            let mut springs = Vec::new();
            if config.stiffness > 0.0 {
                for i in 0..config.rows {
                    for j in 0..config.cols {
                        let p_idx = start_index + i * config.cols + j;
                        // 右の質点とのバネ
                        if j < config.cols - 1 {
                            let p2_idx = start_index + i * config.cols + (j + 1);
                            springs.push(Spring::new(p_idx, p2_idx, config.stiffness, &self.particles));
                        }
                        // 下の質点とのバネ
                        if i < config.rows - 1 {
                            let p2_idx = start_index + (i + 1) * config.cols + j;
                            springs.push(Spring::new(p_idx, p2_idx, config.stiffness, &self.particles));
                        }
                    }
                }
            }

            let shape_constraint = if config.shape_stiffness > 0.0 {
                Some(ShapeMatchingConstraint::new(particle_indices.clone(), config.shape_stiffness, &self.particles))
            } else {
                None
            };
            
            self.soft_bodies.push(SoftBody {
                particle_indices,
                springs,
                shape_constraint,
            });
        }

        /// シミュレーションを 1 ステップ進めます。
        ///
        /// # Arguments
        ///
        /// * `dt` - タイムステップ（例: `1.0 / 60.0`）。
        pub fn step(&mut self, dt: f64) {
            // 1. 力を適用 (Verlet積分)
            for p in &mut self.particles {
                if p.is_fixed { continue; }
                p.vel += self.config.gravity * dt;
                p.prev_pos = p.pos;
                p.pos += p.vel * dt;
            }

            // 2. 拘束を解決 (反復法)
            for _ in 0..self.config.solver_iterations {
                for sb in &mut self.soft_bodies {
                    for spring in &sb.springs {
                        spring.solve(&mut self.particles);
                    }
                    if let Some(sc) = &mut sb.shape_constraint {
                        sc.solve(&mut self.particles);
                    }
                }
                self.solve_collisions();
                self.apply_boundary_conditions();
            }

            // 3. 速度を更新
            for p in &mut self.particles {
                if p.is_fixed {
                    p.vel = Vec2::new(0.0, 0.0);
                    continue;
                }
                let new_vel = (p.pos - p.prev_pos) * (1.0 / dt);
                p.vel = new_vel * self.config.damping;
            }
        }

        /// 質点間の衝突を解決します。
        fn solve_collisions(&mut self) {
            let n = self.particles.len();
            for i in 0..n {
                for j in i + 1..n {
                    let (p1, p2) = self.particles.split_at_mut(j);
                    let (p1, p2) = (&mut p1[i], &mut p2[0]);
                    
                    let diff = p1.pos - p2.pos;
                    let dist_sq = diff.length_squared();
                    let min_dist = p1.radius + p2.radius;

                    if dist_sq < min_dist * min_dist {
                        let dist = dist_sq.sqrt();
                        let total_inv_mass = p1.inv_mass + p2.inv_mass;
                        if total_inv_mass < f64::EPSILON { continue; }

                        let correction = diff.normalize() * ((min_dist - dist) / total_inv_mass);
                        p1.pos += correction * p1.inv_mass;
                        p2.pos -= correction * p2.inv_mass;
                    }
                }
            }
        }

        /// 境界条件を適用します。
        fn apply_boundary_conditions(&mut self) {
            if let Some((min, max)) = self.config.bounds {
                for p in &mut self.particles {
                    p.pos.x = p.pos.x.max(min.x + p.radius).min(max.x - p.radius);
                    p.pos.y = p.pos.y.max(min.y + p.radius).min(max.y - p.radius);
                }
            }
        }
        
        // --- 外部からシミュレーション状態を読み取るためのゲッター ---
        
        /// 全ての質点のスライスを返します。
        pub fn particles(&self) -> &[Particle] {
            &self.particles
        }
        
        /// 全てのソフトボディのスライスを返します。
        pub fn soft_bodies(&self) -> &[SoftBody] {
            &self.soft_bodies
        }
        
        /// シミュレーション設定への参照を返します。
        pub fn config(&self) -> &SimulationConfig {
            &self.config
        }
        
        /// シミュレーション設定を可変で取得します。
        pub fn config_mut(&mut self) -> &mut SimulationConfig {
            &mut self.config
        }
    }
}
