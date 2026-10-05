# カメラ校正用チェッカーボード

- 印刷用: [PDF](checkerboard_9x6_25mm_A4.pdf)
- ベクター版: [SVG](checkerboard_9x6_25mm_A4.svg)
- 画像版: [PNG・300 dpi](checkerboard_9x6_25mm_A4.png)

## 寸法

| 項目 | 値 |
| --- | --- |
| 用紙 | A4・横向き（297×210 mm） |
| マスの数 | 横10×縦7 |
| 校正に指定する内角の数 | 横9×縦6、合計54点 |
| 1マス | 25×25 mm |
| 白黒パターンの全体 | 250×175 mm |

PDFには縮尺確認用の100 mmの目盛りがあります。
PDFのページ寸法・マス寸法と、生成画像からのOpenCVによる54内角の検出を確認済みです。

## 印刷と使用

1. PDFを **A4・横向き・倍率100%／実際のサイズ** で印刷します。
2. 「用紙に合わせる」「縮小して印刷」はオフにします。
3. 印刷後に定規で目盛りが100 mm、縦横とも1マスが25 mmか確認します。
4. 厚紙や平らな板へ貼り、反り・しわ・光の反射が出ないようにします。白い余白を残してください。
5. カメラの中央・四隅、複数の距離・傾きで撮影します。毎回、ボード全体を映してください。

PNGやSVGを別のソフトから印刷すると、倍率が変わることがあります。印刷にはPDFを推奨します。

## このリポジトリで使うコマンド

先に [はじめてのガイド](../../MOCOPI_START_GUIDE.md) のターミナル準備を行います。
今回のC270は `/dev/video2` が映像取得用のIDです。接続順が変わったらcamera-listで確認してください。

```bash
uv run --no-sync --active python -m telegrip.mocopi camera-capture \
  --camera /dev/video2 --output outputs/mocopi/checkerboard --count 20 --interval 2

uv run --no-sync --active python -m telegrip.mocopi calibrate-camera \
  --images outputs/mocopi/checkerboard --board 9 6 --square-m 0.025
```

25 mmのマスには `--square-m 0.025` を指定します。
別の大きさで印刷した場合は、実測した1マスの寸法をメートルで指定してください。
画像にはレンズの歪みや実際の撮影条件が含まれる必要があります。
配布PNGそのものを撮影画像の代わりに校正へ入力することはできません。
