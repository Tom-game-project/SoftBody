use softbody::{SoftBody, Point, Line, find_all_intersections, find_nearest_segment, move_p_to_line_ab, circular::CircularWindowsExt};
use macroquad::prelude::*;


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


fn init_point_with_zero_v(x: f32, y: f32) -> Point
{
    Point { position: (x, y) , velocity: (0.0, 0.0), mass: 1.0}
}

/// インタラクティブなテストウィンドウを起動する
async fn interactive_test_main() {
    // テスト用のSoftBodyを2つ準備
    let body1 = SoftBody {
        shape: vec![
            init_point_with_zero_v(100.0, 100.0),
            init_point_with_zero_v(300.0, 100.0),
            init_point_with_zero_v(300.0, 250.0),
            init_point_with_zero_v(100.0, 250.0),
        ],
    };

    let mut body2 = SoftBody {
        shape: vec![
            init_point_with_zero_v(400.0, 300.0),
            init_point_with_zero_v(600.0, 300.0),
            init_point_with_zero_v(550.0, 450.0),
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

/// 交点をすべて集めるテスト
async fn interactive_test_main01() {
    // テスト用のSoftBodyを準備
    let softbody = SoftBody {
        shape: vec![
            // 頂点を順番に結んでいく
             init_point_with_zero_v(400.0, 100.0) , // 外側の頂点 1 (上)
             init_point_with_zero_v(350.0, 250.0)  , // 内側の頂点 1 (凹み)
             init_point_with_zero_v(500.0, 300.0) , // 外側の頂点 2 (右)
             init_point_with_zero_v(400.0, 350.0) , // 内側の頂点 2 (凹み)
             init_point_with_zero_v(300.0, 500.0) , // 外側の頂点 3 (下)
             init_point_with_zero_v(250.0, 350.0) , // 内側の頂点 3 (凹み)
             init_point_with_zero_v(100.0, 300.0) , // 外側の頂点 4 (左)
             init_point_with_zero_v(250.0, 250.0) , // 内側の頂点 4 (凹み)
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

/// 最近傍の線分を発見する
async fn interactive_test_main02() {
    let softbody = SoftBody {
        shape: vec![
             init_point_with_zero_v(400.0, 100.0) ,
             init_point_with_zero_v(350.0, 250.0) ,
             init_point_with_zero_v(500.0, 300.0) ,
             init_point_with_zero_v(400.0, 350.0) ,
             init_point_with_zero_v(300.0, 500.0) ,
             init_point_with_zero_v(250.0, 350.0) ,
             init_point_with_zero_v(100.0, 300.0) ,
             init_point_with_zero_v(250.0, 250.0) ,
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

/// 2つの点 p1 と p2 の間のユークリッド距離を計算します。
fn dist(p1: (f32, f32), p2: (f32, f32)) -> f32 {
    // 1. x座標とy座標の差をそれぞれ計算します。
    let dx = p2.0 - p1.0;
    let dy = p2.1 - p1.1;

    // 2. 三平方の定理に基づき、差の2乗の和の平方根を求めます。
    (dx.powi(2) + dy.powi(2)).sqrt()
}

async fn interactive_test_main03() {
    let mut points = vec![
        Point { position: (200.0, 400.0), velocity: (0.0, 0.0), mass: 1.0 }, // Point A
        Point { position: (600.0, 300.0), velocity: (0.0, 0.0), mass: 1.0 }, // Point B 
        Point { position: (350.0, 150.0), velocity: (0.0, 0.0), mass: 1.0 }, // Point P
    ];

    let mut dragging_point_idx: Option<usize> = None;
    let point_radius = 15.0;

    loop {
        // --- 1. 入力処理 (ドラッグ) ---
        let mouse_pos = mouse_position();
        if is_mouse_button_pressed(MouseButton::Left) {
            for (i, p) in points.iter().enumerate() {
                if dist(mouse_pos, p.position) < point_radius {
                    dragging_point_idx = Some(i);
                    break;
                }
            }
        }
        if is_mouse_button_down(MouseButton::Left) {
            if let Some(idx) = dragging_point_idx {
                points[idx].position = mouse_pos;
            }
        }
        if is_mouse_button_released(MouseButton::Left) {
            dragging_point_idx = None;
        }

        // --- 2. 計算 ---
        let (final_a, final_b, final_p) = move_p_to_line_ab(&points[0], &points[1], &points[2], 20.0);

        // --- 3. 描画 ---
        clear_background(WHITE);

        // Beforeの状態を描画
        draw_line(points[0].position.0, points[0].position.1, points[1].position.0, points[1].position.1, 1.0, LIGHTGRAY);
        draw_circle(points[0].position.0, points[0].position.1, point_radius, BLUE);
        draw_circle(points[1].position.0, points[1].position.1, point_radius, BLUE);
        draw_circle(points[2].position.0, points[2].position.1, point_radius, RED);
        draw_text("A", points[0].position.0 - 5.0, points[0].position.1 + 5.0, 24.0, WHITE);
        draw_text("B", points[1].position.0 - 5.0, points[1].position.1 + 5.0, 24.0, WHITE);
        draw_text("P", points[2].position.0 - 5.0, points[2].position.1 + 5.0, 24.0, WHITE);

        // Afterの状態を描画
        draw_line(final_a.position.0, final_a.position.1, final_b.position.0, final_b.position.1, 2.0, BLACK);
        draw_circle_lines(final_a.position.0, final_a.position.1, point_radius, 2.0, BLUE);
        draw_circle_lines(final_b.position.0, final_b.position.1, point_radius, 2.0, BLUE);
        draw_circle_lines(final_p.position.0, final_p.position.1, point_radius, 2.0, RED);

        // 各点の移動ベクトルを描画
        draw_line(points[0].position.0, points[0].position.1, final_a.position.0, final_a.position.1, 1.0, GRAY);
        draw_line(points[1].position.0, points[1].position.1, final_b.position.0, final_b.position.1, 1.0, GRAY);
        draw_line(points[2].position.0, points[2].position.1, final_p.position.0, final_p.position.1, 1.0, GRAY);
        
        // UI
        draw_text("Drag points A, B, P to test the function.", 10.0, 20.0, 20.0, DARKGRAY);
        draw_text("Solid: Before | Outlined: After", 10.0, 40.0, 20.0, DARKGRAY);
        draw_text(&format!("A mass: {:.1}, B mass: {:.1}", points[0].mass, points[1].mass), 10.0, 60.0, 20.0, DARKGRAY);

        next_frame().await;
    }
}


#[test]
fn run_interactive_visualization_test00() {
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

#[test]
fn run_interactive_visualization_test03() {
    // macroquadの設定
    let config = Conf {
        window_title: "Interactive SoftBody Test".to_string(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    };
    // macroquadのウィンドウをテスト内で起動
    macroquad::Window::from_config(config, interactive_test_main03());
}
