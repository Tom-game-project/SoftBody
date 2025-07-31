// Iteratorトレイトを実装
/// スライスの要素を循環的にペアリングするイテレータ。
/// 例: `[a, b, c]` -> `(a, b), (b, c), (c, a)`
#[derive(Debug)]
pub struct CircularWindows<'a, T> {
    slice: &'a [T],
    index: usize,
}

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
