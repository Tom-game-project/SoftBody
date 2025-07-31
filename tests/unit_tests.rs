
use softbody::{Line, inter_section};

#[test]
fn it_works01()
{
    println!("--- i32 の場合 ---");
    // i32 でテスト
    let line1_i32 = Line { start: (0, 0), end: (10, 10) };
    let line2_i32 = Line { start: (0, 10), end: (10, 0) };
    assert_eq!(inter_section(line1_i32, line2_i32), Some((5, 5)));

    println!("
--- f64 の場合 ---");
    // f64 でテスト
    let line1_f64 = Line { start: (0.0, 0.0), end: (10.0, 10.0) };
    let line2_f64 = Line { start: (0.0, 10.0), end: (10.0, 0.0) };
    assert_eq!(inter_section(line1_f64, line2_f64), Some((5.0, 5.0)));

    // f64 で小数点を含むテスト
    let line3_f64 = Line { start: (1.5, 2.5), end: (8.5, 9.5) };
    let line4_f64 = Line { start: (1.5, 9.5), end: (8.5, 2.5) };
    assert_eq!(inter_section(line3_f64, line4_f64), Some((5.0, 6.0)));
}
