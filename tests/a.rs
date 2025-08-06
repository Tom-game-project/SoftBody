use macroquad::color::{Color, GRAY, RED, WHITE};
use macroquad::prelude::rand;
use macroquad::input::{is_key_down, is_key_pressed, is_mouse_button_pressed, is_mouse_button_released, mouse_position, KeyCode, MouseButton};
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
                                                                               
const LIGHTGRAY:Color = Color::new(0.13, 0.13, 0.16, 1.0);
const BLUE: Color = Color::new(0.0 , 0.0, 1.0, 1.0);
const VIOLET:Color = Color::new(0.56, 0.00, 1.00, 1.0);

/// Macroquadアプリケーションのエントリポイントを定義
async fn test00() {
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

async fn test01() 
{
     // --- 1. シミュレーションの初期設定 ---
    let sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: Vec2::new(0.0, 800.0),
        solver_iterations: 4, // オブジェクトが多いので少し減らす
        damping: 0.99,
        use_wire_collisions: false,
        use_volumetric_collisions:true
    };

    let mut sim = Simulation::new(sim_config);

    // --- ここでたくさんのキューブを生成 ---
    let grid_cols = 5;
    let grid_rows = 5;
    let cube_size = 30.0;
    let spacing = 40.0;
    let start_x = (screen_width() - (grid_cols - 1) as f32 * spacing) / 2.0;
    let start_y = 100.0;

    for i in 0..grid_rows {
        for j in 0..grid_cols {
            let x = start_x as f64 + j as f64 * spacing as f64;
            let y = start_y as f64 + i as f64 * spacing as f64;

            let cube_config = SoftBodyConfig {
                center: Vec2::new(x, y),
                size: Vec2::new(cube_size, cube_size),
                rows: 4, // 小さなキューブは3x3の質点で構成
                cols: 4,
                stiffness: 0.25,
                shape_stiffness: 0.2,
                particle_radius: 5.0, // 質点を小さくする
                ..Default::default()
            };
            sim.add_soft_body(&cube_config);
        }
    }

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

        if let Some(idx) = dragged_particle_index {
            if let Some(p) = sim.particles.get_mut(idx) {
                p.pos = mouse_pos;
                p.vel = Vec2::new(0.0, 0.0);
                p.prev_pos = mouse_pos;
            }
        }

        // --- 2b. 物理演算の更新 ---
        let dt = (get_frame_time() as f64).min(1.0 / 30.0); // フレームレート低下時の爆発を防ぐ
        sim.step(dt);


        // --- 2c. 描画処理 ---
        clear_background(BACKGROUND_COLOR);

        for sb in sim.soft_bodies() {
            for spring in &sb.springs {
                let p1 = &sim.particles[spring.p1_index];
                let p2 = &sim.particles[spring.p2_index];

                let dist = (p1.pos - p2.pos).length();
                let stretch = (dist - spring.rest_length).abs() / spring.rest_length;

                let intensity = (stretch * 3.0).min(1.0) as f32;
                let color = Color {
                    r: SPRING_BASE_COLOR.r * (1.0 - intensity) + SPRING_STRETCH_COLOR.r * intensity,
                    g: SPRING_BASE_COLOR.g * (1.0 - intensity) + SPRING_STRETCH_COLOR.g * intensity,
                    b: SPRING_BASE_COLOR.b * (1.0 - intensity) + SPRING_STRETCH_COLOR.b * intensity,
                    a: 0.8, // 少し透明にする
                };

                //draw_line(
                //    p1.pos.x as f32, p1.pos.y as f32,
                //    p2.pos.x as f32, p2.pos.y as f32,
                //    1.5, color
                //);
            }
        }

        for p in &sim.particles {
            draw_circle(p.pos.x as f32, p.pos.y as f32, p.radius as f32, PARTICLE_COLOR);
        }

        let info_text = format!("FPS: {} | Particles: {}", get_fps(), sim.particles.len());
        draw_text(&info_text, 10.0, 20.0, 24.0, WHITE);
        draw_text("Drag the cubes!", 10.0, 45.0, 20.0, GRAY);

        next_frame().await
    }
}

async fn test02() {
    let initial_gravity = Vec2::new(0.0, 0.0); // ★ 最初は無重力

    let sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: initial_gravity,
        solver_iterations: 6,
        damping: 0.99,
        use_wire_collisions: false,
        use_volumetric_collisions:true
    };
    
    let mut sim = Simulation::new(sim_config);

    // ... キューブの配置ロジックは変更なし ...
    let grid_cols = 4;
    let grid_rows = 4;
    let cube_size = 40.0;
    let spacing = 65.0;
    let start_x = (screen_width() - (grid_cols - 1) as f32 * spacing) / 2.0;
    let start_y = (screen_height() - (grid_rows - 1) as f32 * spacing) / 2.0;
    for i in 0..grid_rows {
        for j in 0..grid_cols {
            let x = start_x as f64 + j as f64 * spacing as f64;
            let y = start_y as f64 + i as f64 * spacing as f64;
            let cube_config = SoftBodyConfig {
                center: Vec2::new(x, y),
                size: Vec2::new(cube_size, cube_size),
                rows: 5, cols: 5, stiffness: 0.6, shape_stiffness: 0.5, particle_radius: 5.0,
                ..Default::default()
            };
            sim.add_soft_body(&cube_config);
        }
    }
    
    let mut dragged_particle_index: Option<usize> = None;

    // ★ 1. つまみUIの状態変数を定義
    let knob_base_pos = Vec2::new(screen_width() as f64 - 100.0, screen_height() as f64 - 100.0);
    let knob_radius = 60.0;
    let handle_radius = 25.0;
    let mut knob_handle_pos = knob_base_pos; // ハンドルの初期位置は中心
    let mut is_dragging_knob = false;
    let max_gravity_force = 1200.0;


    loop {
        let (mx, my) = mouse_position();
        let mouse_pos = Vec2::new(mx as f64, my as f64);

        // ★ 2. つまみUIの入力処理
        // マウスが押された瞬間
        if is_mouse_button_pressed(MouseButton::Left) {
            // つまみの上でクリックされたか、またはすでにつまみをドラッグ中か
            if !is_dragging_knob && (mouse_pos - knob_base_pos).length() < knob_radius {
                is_dragging_knob = true;
            }
        }
        
        // マウスボタンが離されたらドラッグ終了
        if is_mouse_button_released(MouseButton::Left) {
            is_dragging_knob = false;
        }

        if is_dragging_knob {
            let delta = mouse_pos - knob_base_pos;
            let dist = delta.length();

            if dist > knob_radius {
                // ハンドルがベース円の外に出ないように位置を制限
                knob_handle_pos = knob_base_pos + delta.normalize() * knob_radius;
            } else {
                knob_handle_pos = mouse_pos;
            }
        } else {
            // ドラッグ中でなければハンドルは中心に戻る
            knob_handle_pos = knob_base_pos;
        }

        // ★ 3. UIの状態から重力を計算し、シミュレーションに適用
        let gravity_vec = knob_handle_pos - knob_base_pos;
        let gravity_ratio = gravity_vec.length() / knob_radius; // 0.0 ~ 1.0 の強さの割合
        let new_gravity = gravity_vec.normalize() * max_gravity_force * gravity_ratio;
        sim.config_mut().gravity = new_gravity;


        // ... (パーティクルのドラッグ処理は変更なし、ただしUI操作と競合しないようにする) ...
        if is_mouse_button_pressed(MouseButton::Left) && !is_dragging_knob {
             if dragged_particle_index.is_none() {
                let mut closest_dist_sq = 400.0;
                let mut closest_idx: Option<usize> = None;
                for (i, p) in sim.particles.iter().enumerate() {
                    if (p.pos - mouse_pos).length_squared() < closest_dist_sq {
                        closest_dist_sq = (p.pos - mouse_pos).length_squared();
                        closest_idx = Some(i);
                    }
                }
                dragged_particle_index = closest_idx;
            }
        }

        if is_mouse_button_released(MouseButton::Left) {
             dragged_particle_index = None;
        }

        if let Some(idx) = dragged_particle_index {
             if let Some(p) = sim.particles.get_mut(idx) {
                p.pos = mouse_pos;
                p.vel = Vec2::new(0.0, 0.0);
                p.prev_pos = mouse_pos;
            }
        }
        
        let dt = (get_frame_time() as f64).min(1.0 / 30.0);
        sim.step(dt);

        clear_background(BACKGROUND_COLOR);

        // ... (オブジェクトの描画処理は変更なし) ...
        for sb in sim.soft_bodies() {
            for spring in &sb.springs {
                let p1 = &sim.particles[spring.p1_index]; let p2 = &sim.particles[spring.p2_index];
                let dist = (p1.pos - p2.pos).length(); let stretch = (dist - spring.rest_length).abs() / spring.rest_length;
                let intensity = (stretch * 3.0).min(1.0) as f32;
                let color = Color { r: SPRING_BASE_COLOR.r * (1.0 - intensity) + SPRING_STRETCH_COLOR.r * intensity, g: SPRING_BASE_COLOR.g * (1.0 - intensity) + SPRING_STRETCH_COLOR.g * intensity, b: SPRING_BASE_COLOR.b * (1.0 - intensity) + SPRING_STRETCH_COLOR.b * intensity, a: 0.8, };
                draw_line(p1.pos.x as f32, p1.pos.y as f32, p2.pos.x as f32, p2.pos.y as f32, 1.5, color);
            }
        }
        for p in &sim.particles { draw_circle(p.pos.x as f32, p.pos.y as f32, p.radius as f32, PARTICLE_COLOR); }
        
        // ★ 4. つまみUIを描画
        // ベース円
        draw_circle(knob_base_pos.x as f32, knob_base_pos.y as f32, knob_radius as f32, Color::from_rgba(50, 50, 60, 150));
        // ハンドル円
        draw_circle(knob_handle_pos.x as f32, knob_handle_pos.y as f32, handle_radius as f32, LIGHTGRAY);

        // UI情報の描画
        let info_text = format!("FPS: {} | Particles: {}", get_fps(), sim.particles.len());
        draw_text(&info_text, 10.0, 20.0, 24.0, WHITE);
        draw_text("Drag the bottom-right knob to control gravity.", 10.0, 45.0, 20.0, GRAY);

        next_frame().await
    }
}


//#[macroquad::main("Soft Body Simulation - Wire Collisions")]
async fn test03() {
    let mut use_wire_collision = true;
    let mut sim = create_simulation(use_wire_collision);
    
    loop {
        if is_key_down(KeyCode::W) {
            use_wire_collision = !use_wire_collision;
            sim.config_mut().use_wire_collisions = use_wire_collision;
        }
        if is_key_down(KeyCode::R) {
            sim = create_simulation(use_wire_collision);
        }

        // (入力処理、ステップ更新は以前のコードと同様)
        let dt = (get_frame_time() as f64).min(1.0 / 30.0);
        sim.step(dt);

        clear_background(Color::from_rgba(20, 20, 30, 255));

        // 描画
        for sb in sim.soft_bodies() {
            let color = if sb.outline_wires.is_some() { VIOLET } else { BLUE };
            for &p_idx in &sb.particle_indices {
                let p = &sim.particles()[p_idx];
                draw_circle(p.pos.x as f32, p.pos.y as f32, p.radius as f32, color);
            }
            if let Some(wires) = &sb.outline_wires {
                for &(p1_idx, p2_idx) in wires {
                    let p1 = &sim.particles()[p1_idx];
                    let p2 = &sim.particles()[p2_idx];
                    draw_line(p1.pos.x as f32, p1.pos.y as f32, p2.pos.x as f32, p2.pos.y as f32, 2.5, WHITE);
                }
            }
        }
        
        // UIテキスト
        let info_text = format!("FPS: {} | Particles: {}", get_fps(), sim.particles().len());
        draw_text(&info_text, 10.0, 20.0, 24.0, WHITE);
        let mode_text = format!("Wire Collision: {} (Press 'W' to toggle)", if use_wire_collision { "ON" } else { "OFF" });
        draw_text(&mode_text, 10.0, 45.0, 20.0, WHITE);
        draw_text("Press 'R' to reset", 10.0, 65.0, 20.0, GRAY);

        next_frame().await;
    }
}


async fn test04() {
    let mut sim = create_simulation2();

    loop {
        if is_key_pressed(KeyCode::R) { sim = create_simulation2(); }

        let dt = (get_frame_time() as f64).min(1.0 / 30.0);
        sim.step(dt);

        clear_background(Color::from_rgba(20, 20, 30, 255));

        for sb in sim.soft_bodies() {
            if let Some(wires) = &sb.outline_wires {
                for &(p1_idx, p2_idx) in wires {
                    let p1 = &sim.particles()[p1_idx]; let p2 = &sim.particles()[p2_idx];
                    draw_line(p1.pos.x as f32, p1.pos.y as f32, p2.pos.x as f32, p2.pos.y as f32, 2.5, WHITE);
                }
            }
            for &p_idx in &sb.particle_indices {
                let p = &sim.particles()[p_idx];
                draw_circle(p.pos.x as f32, p.pos.y as f32, p.radius as f32, VIOLET);
            }
        }

        draw_text("Press 'R' to reset", 10.0, 20.0, 24.0, GRAY);
        next_frame().await;
    }
}

fn create_simulation2() -> Simulation {
    let mut sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: Vec2::new(0.0, 300.0),
        solver_iterations: 12,
        use_wire_collisions: true,
        ..Default::default()
    };
    let mut sim = Simulation::new(sim_config);

    let star_points = |center: Vec2, r_outer: f64, r_inner: f64, n_points: usize| {
        (0..n_points * 2).map(|i| {
            let r = if i % 2 == 0 { r_outer } else { r_inner };
            let angle = (i as f64 / (n_points * 2) as f64) * 2.0 * std::f64::consts::PI;
            center + Vec2::new(angle.cos() * r, angle.sin() * r)
        }).collect::<Vec<_>>()
    };

    // 1. 上から落ちてくる星
    let star1_conf = SoftBodyConfig {
        stiffness: 0.3, shape_stiffness: 0.7, is_fixed: false, // is_fixed: false
        particle_radius: 6.0, particle_inv_mass: 0.1, // 有限の質量
        ..Default::default()
    };
    sim.add_convex_body(&star_points(Vec2::new(500.0, 150.0), 80.0, 40.0, 5), &star1_conf).unwrap();

    // 2. 下で待ち受ける星
    let star2_conf = SoftBodyConfig {
        stiffness: 0.3, shape_stiffness: 0.7, is_fixed: false, // is_fixed: false
        particle_radius: 6.0, particle_inv_mass: 0.1, // 有限の質量
        ..Default::default()
    };
    sim.add_convex_body(&star_points(Vec2::new(500.0, 400.0), 100.0, 50.0, 7), &star2_conf).unwrap();

    sim
}

// シーンを生成するヘルパー関数
fn create_simulation(use_wire_collision: bool) -> Simulation {
    let mut sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: Vec2::new(0.0, 600.0),
        solver_iterations: 10,
        use_wire_collisions: use_wire_collision,
        ..Default::default()
    };
    let mut sim = Simulation::new(sim_config);

    // 1. 固定された凹型の受け皿を生成
    let container_points = vec![
        Vec2::new(100.0, 500.0), Vec2::new(150.0, 300.0),
        Vec2::new(350.0, 250.0), Vec2::new(450.0, 250.0),
        Vec2::new(650.0, 300.0), Vec2::new(700.0, 500.0),
        Vec2::new(600.0, 550.0), Vec2::new(200.0, 550.0),
    ];
    let container_conf = SoftBodyConfig {
        stiffness: 0.8, shape_stiffness: 0.8, is_fixed: true, particle_radius: 6.0,
        ..Default::default()
    };
    sim.add_convex_body(&container_points, &container_conf).unwrap();

    // 2. 落下する五角形を生成
    let pentagon_points = (0..5).map(|i| {
        let angle = (i as f64 / 5.0) * 2.0 * std::f64::consts::PI;
        Vec2::new(400.0 + angle.cos() * 50.0, 150.0 + angle.sin() * 50.0)
    }).collect::<Vec<_>>();
    let pentagon_conf = SoftBodyConfig {
        stiffness: 0.4, shape_stiffness: 0.9, particle_radius: 8.0,
        ..Default::default()
    };
    sim.add_convex_body(&pentagon_points, &pentagon_conf).unwrap();
    
    sim
}


// 星の色をいくつか定義
const STAR_COLORS: [Color; 5] = [
    Color::new(1.0, 0.8, 0.4, 1.0), // Yellow
    Color::new(1.0, 0.5, 0.8, 1.0), // Pink
    Color::new(0.6, 1.0, 0.8, 1.0), // Mint
    Color::new(0.7, 0.7, 1.0, 1.0), // Lavender
    Color::new(1.0, 0.7, 0.5, 1.0), // Peach
];

//#[macroquad::main("Falling Stars")]
async fn test05() {
    let mut sim = create_simulation05();
    
    loop {
        // 'R'キーでリセット
        if is_key_pressed(KeyCode::R) { 
            sim = create_simulation05(); 
        }

        // シミュレーションを1ステップ進める
        let dt = (get_frame_time() as f64).min(1.0 / 30.0);
        sim.step(dt);

        // 描画
        clear_background(Color::from_rgba(30, 30, 45, 255));

        // 各オブジェクトを描画
        for (i, sb) in sim.soft_bodies().iter().enumerate() {
            // ワイヤーフレームを持つオブジェクト（星）を描画
            if let Some(wires) = &sb.outline_wires {
                let color = STAR_COLORS[i % STAR_COLORS.len()];
                for &(p1_idx, p2_idx) in wires {
                    let p1 = &sim.particles()[p1_idx];
                    let p2 = &sim.particles()[p2_idx];
                    draw_line(p1.pos.x as f32, p1.pos.y as f32, p2.pos.x as f32, p2.pos.y as f32, 2.5, color);
                }
            }
        }
        
        draw_text("Press 'R' to reset", 10.0, 20.0, 24.0, GRAY);
        next_frame().await;
    }
}

/// シミュレーションシーンを生成するヘルパー関数
fn create_simulation05() -> Simulation {
    // シミュレーションの基本設定
    let sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: Vec2::new(0.0, 500.0),
        solver_iterations: 15,
        use_wire_collisions: true, // ワイヤー衝突を有効化
        ..Default::default()
    };
    let mut sim = Simulation::new(sim_config);

    // 星の形状を生成するクロージャ
    let star_points = |center: Vec2, r_outer: f64, r_inner: f64, n_points: usize| {
        (0..n_points * 2).map(|i| {
            let r = if i % 2 == 0 { r_outer } else { r_inner };
            let angle = (i as f64 / (n_points * 2) as f64) * 2.0 * std::f64::consts::PI + rand::gen_range(-0.1, 0.1);
            center + Vec2::new(angle.cos() * r, angle.sin() * r)
        }).collect::<Vec<_>>()
    };
    // ぷるぷるした星を複数生成
    let num_stars = 8;
    for i in 0..num_stars {
        let center_x = screen_width() as f64 * (0.2 + 0.6 * rand::gen_range(0.0, 1.0));
        let center_y = screen_height() as f64 * (0.1 + 0.2 * rand::gen_range(0.0, 1.0)) - (i as f64 * 20.0);
        
        // "ぷるぷる"感を出すために剛性を低めに設定
        let star_conf = SoftBodyConfig {
            stiffness: 0.1,         // バネの硬さ
            shape_stiffness: 0.2,   // 形状維持の強さ
            is_fixed: false,
            particle_radius: 7.0,
            particle_inv_mass: 0.2, // 少し重め
            ..Default::default()
        };
        
        let points = star_points(Vec2::new(center_x, center_y), 60.0, 30.0, 5);
        sim.add_convex_body(&points, &star_conf).unwrap();
    }
    
    sim
}

//#[macroquad::main("Falling Stars - Gravity Fun")]
async fn test06() {
    let mut sim = create_simulation06();

    // ★ 1. つまみUIの状態変数を定義
    let knob_base_pos = Vec2::new(screen_width() as f64 - 100.0, screen_height() as f64 - 100.0);
    let knob_radius = 60.0;
    let handle_radius = 25.0;
    let mut knob_handle_pos = knob_base_pos;
    let mut is_dragging_knob = false;
    let max_gravity_force = 1500.0;

    loop {
        // 'R'キーでリセット
        if is_key_pressed(KeyCode::R) {
            sim = create_simulation06();
        }

        // ★ 2. つまみUIの入力処理
        let (mx, my) = mouse_position();
        let mouse_pos = Vec2::new(mx as f64, my as f64);

        if is_mouse_button_pressed(MouseButton::Left) {
            if !is_dragging_knob && (mouse_pos - knob_base_pos).length() < knob_radius {
                is_dragging_knob = true;
            }
        }
        if is_mouse_button_released(MouseButton::Left) {
            is_dragging_knob = false;
        }

        if is_dragging_knob {
            let delta = mouse_pos - knob_base_pos;
            if delta.length() > knob_radius {
                knob_handle_pos = knob_base_pos + delta.normalize() * knob_radius;
            } else {
                knob_handle_pos = mouse_pos;
            }
        } else {
            knob_handle_pos = knob_base_pos;
        }

        // ★ 3. UIの状態から重力を計算し、シミュレーションに適用
        let gravity_vec = knob_handle_pos - knob_base_pos;
        let gravity_ratio = gravity_vec.length() / knob_radius;
        let new_gravity = gravity_vec.normalize() * max_gravity_force * gravity_ratio;
        sim.config_mut().gravity = new_gravity;


        // シミュレーションを1ステップ進める
        let dt = (get_frame_time() as f64).min(1.0 / 30.0);
        sim.step(dt);

        // 描画
        clear_background(Color::from_rgba(30, 30, 45, 255));

        // 各オブジェクトを描画
        for (i, sb) in sim.soft_bodies().iter().enumerate() {
            if let Some(wires) = &sb.outline_wires {
                let color = STAR_COLORS[i % STAR_COLORS.len()];
                for &(p1_idx, p2_idx) in wires {
                    let p1 = &sim.particles()[p1_idx];
                    let p2 = &sim.particles()[p2_idx];
                    draw_line(p1.pos.x as f32, p1.pos.y as f32, p2.pos.x as f32, p2.pos.y as f32, 2.5, color);
                }
            }
        }

        // ★ 4. つまみUIを描画
        draw_circle(knob_base_pos.x as f32, knob_base_pos.y as f32, knob_radius as f32, Color::from_rgba(50, 50, 60, 150));
        draw_circle(knob_handle_pos.x as f32, knob_handle_pos.y as f32, handle_radius as f32, LIGHTGRAY);

        draw_text("Drag the knob to control gravity. Press 'R' to reset.", 10.0, 20.0, 20.0, GRAY);
        next_frame().await;
    }
}

/// シミュレーションシーンを生成するヘルパー関数
fn create_simulation06() -> Simulation {
    // ★ 5. 初期重力をゼロに設定
    let sim_config = SimulationConfig {
        bounds: Some((Vec2::new(0.0, 0.0), Vec2::new(screen_width() as f64, screen_height() as f64))),
        gravity: Vec2::new(0.0, 0.0),
        solver_iterations: 10,
        use_wire_collisions: true,
        ..Default::default()
    };
    let mut sim = Simulation::new(sim_config);

    let star_points = |center: Vec2, r_outer: f64, r_inner: f64, n_points: usize| {
        (0..n_points * 2).map(|i| {
            let r = if i % 2 == 0 { r_outer } else { r_inner };
            let angle = (i as f64 / (n_points * 2) as f64) * 2.0 * std::f64::consts::PI + rand::gen_range(-0.1, 0.1);
            center + Vec2::new(angle.cos() * r, angle.sin() * r)
        }).collect::<Vec<_>>()
    };

    let num_stars = 10; // 星の数を少し増やす
    for i in 0..num_stars {
        let center_x = screen_width() as f64 * (0.2 + 0.6 * rand::gen_range(0.0, 1.0));
        let center_y = screen_height() as f64 * (0.2 + 0.6 * rand::gen_range(0.0, 1.0));

        let star_conf = SoftBodyConfig {
            stiffness: 0.1,
            shape_stiffness: 0.2,
            is_fixed: false,
            particle_radius: 7.0,
            particle_inv_mass: 0.2,
            ..Default::default()
        };

        let points = star_points(Vec2::new(center_x, center_y), 60.0, 30.0, 5);
        sim.add_convex_body(&points, &star_conf).unwrap();
    }

    sim
}

/// ```
/// cargo test run_soft00
/// ```
#[test]
fn run_soft00() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test00());
}

/// ```
/// cargo test run_soft01
/// ```
#[test]
fn run_soft01() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 400,
        window_height: 400,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test01());
}

/// ```
/// cargo test run_soft02
/// ```
#[test]
fn run_soft02() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test02());
}

/// ```
/// cargo test run_soft03
/// ```
#[test]
fn run_soft03() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test03());
}

/// ```
/// cargo test run_soft04
/// ```
#[test]
fn run_soft04() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test04());
}

/// ```
/// cargo test run_soft05
/// ```
#[test]
fn run_soft05() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test05());
}

/// ```
/// cargo test run_soft06
/// ```
#[test]
fn run_soft06() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, test06());
}

