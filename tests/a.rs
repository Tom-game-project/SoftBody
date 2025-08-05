use macroquad::color::{Color, GRAY, RED, WHITE};
use macroquad::input::{is_mouse_button_pressed, is_mouse_button_released, mouse_position, MouseButton};
use macroquad::shapes::{draw_circle, draw_line};
use macroquad::text::draw_text;
use macroquad::time::{get_fps, get_frame_time};
use macroquad::window::{clear_background, next_frame, screen_height, screen_width, Conf};

use softbody::core::{Simulation, SimulationConfig, SoftBodyConfig, Vec2};


/// 描画色を定義
const PARTICLE_COLOR: Color = Color::new(0.28, 0.82, 0.78, 1.0); // #4ECDC4
const FIXED_PARTICLE_COLOR: Color = Color::new(1.0, 0.42, 0.42, 1.0); // #FF6B6B
const SPRING_BASE_COLOR: Color = Color::new(0.3, 0.7, 0.6, 1.0);
const SPRING_STRETCH_COLOR: Color = RED;
const BACKGROUND_COLOR: Color = Color::new(0.13, 0.13, 0.16, 1.0); // Dark background

/// Macroquadアプリケーションのエントリポイントを定義
async fn test() {
    // --- 1. シミュレーションの初期設定 ---
    
    // ウィンドウサイズに基づいて境界を設定
    let sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: Vec2::new(0.0, 600.0),
        solver_iterations: 8,
        ..Default::default()
    };
    
    let mut sim = Simulation::new(sim_config);

    // 落下するボディ
    let falling_body = SoftBodyConfig {
        center: Vec2::new(screen_width() as f64 * 0.5, screen_height() as f64 * 0.25),
        size: Vec2::new(120.0, 120.0),
        rows: 8,
        cols: 8,
        stiffness: 0.25,
        shape_stiffness: 0.3,
        ..Default::default()
    };

    // 地面として機能するボディ
    let ground_body = SoftBodyConfig {
        center: Vec2::new(screen_width() as f64 * 0.5, screen_height() as f64 * 0.8),
        size: Vec2::new(screen_width() as f64 * 0.6, 60.0),
        rows: 4,
        cols: 20,
        stiffness: 0.3,
        shape_stiffness: 0.4,
        ..Default::default()
    };
    
    // 固定されたアンカーボディ
    let fixed_anchor = SoftBodyConfig {
        center: Vec2::new(screen_width() as f64 * 0.8, screen_height() as f64 * 0.2),
        size: Vec2::new(50.0, 50.0),
        rows: 3,
        cols: 3,
        stiffness: 0.8,
        shape_stiffness: 0.0, // バネだけで形状を維持
        is_fixed: true,
        ..Default::default()
    };

    sim.add_soft_body(&falling_body);
    sim.add_soft_body(&ground_body);
    sim.add_soft_body(&fixed_anchor);

    // マウスドラッグ用の状態変数
    let mut dragged_particle_index: Option<usize> = None;


    // --- 2. メインループ ---
    loop {
        // --- 2a. 入力処理 (マウスドラッグ) ---
        let (mx, my) = mouse_position();
        let mouse_pos = Vec2::new(mx as f64, my as f64);

        if is_mouse_button_pressed(MouseButton::Left) {
            let mut closest_dist_sq = 400.0; // 20pxの半径内
            let mut closest_idx: Option<usize> = None;

            for (i, p) in sim.particles.iter().enumerate() {
                if p.is_fixed { continue; }
                let dist_sq = (p.pos - mouse_pos).length_squared();
                if dist_sq < closest_dist_sq {
                    closest_dist_sq = dist_sq;
                    closest_idx = Some(i);
                }
            }
            dragged_particle_index = closest_idx;
        }

        if is_mouse_button_released(MouseButton::Left) {
            dragged_particle_index = None;
        }
        
        // ドラッグ中の質点の位置を更新
        if let Some(idx) = dragged_particle_index {
            if let Some(p) = sim.particles.get_mut(idx) {
                p.pos = mouse_pos;
                // 速度と前の位置をリセットして、不自然な飛び出しを防ぐ
                p.vel = Vec2::new(0.0, 0.0);
                p.prev_pos = mouse_pos;
            }
        }
        
        // --- 2b. 物理演算の更新 ---
        // 可変フレームレートに対応するため、get_frame_time() を使用
        // より安定したシミュレーションには固定タイムステップの導入を検討
        let dt = get_frame_time() as f64;
        sim.step(dt);


        // --- 2c. 描画処理 ---
        clear_background(BACKGROUND_COLOR);

        // バネの描画
        for sb in sim.soft_bodies() {
            for spring in &sb.springs {
                let p1 = &sim.particles[spring.p1_index];
                let p2 = &sim.particles[spring.p2_index];
                
                let dist = (p1.pos - p2.pos).length();
                let stretch = (dist - spring.rest_length).abs() / spring.rest_length;
                
                // 伸び率に応じて色を線形補間
                let intensity = (stretch * 3.0).min(1.0) as f32;
                let color = Color {
                    r: SPRING_BASE_COLOR.r * (1.0 - intensity) + SPRING_STRETCH_COLOR.r * intensity,
                    g: SPRING_BASE_COLOR.g * (1.0 - intensity) + SPRING_STRETCH_COLOR.g * intensity,
                    b: SPRING_BASE_COLOR.b * (1.0 - intensity) + SPRING_STRETCH_COLOR.b * intensity,
                    a: 1.0,
                };

                draw_line(
                    p1.pos.x as f32, p1.pos.y as f32,
                    p2.pos.x as f32, p2.pos.y as f32,
                    2.0, color
                );
            }
        }

        // 質点の描画
        for p in &sim.particles {
            let color = if p.is_fixed { FIXED_PARTICLE_COLOR } else { PARTICLE_COLOR };
            draw_circle(p.pos.x as f32, p.pos.y as f32, p.radius as f32, color);
        }
        
        // UI情報の描画
        let info_text = format!("FPS: {} | Particles: {}", get_fps(), sim.particles.len());
        draw_text(&info_text, 10.0, 20.0, 24.0, WHITE);
        draw_text("Click and drag particles to interact", 10.0, 45.0, 20.0, GRAY);

        // 次のフレームを待つ
        next_frame().await
    }
}


/// ```
/// cargo test run_soft
/// ```
#[test]
fn run_soft() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test());
}
