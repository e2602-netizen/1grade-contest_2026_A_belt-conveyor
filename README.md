# ARRC - HAL- Lib

## 使用方法
### リポジトリ作成からクローン方法
右上の「Use this template」ボタンからリポジトリを新しく作成してください。
その後、以下のコマンドでクローンしてください。
```
git clone --recursive [新しいリポジトリのURL]
# ex: git clone --recursive https://github.com/Organization/example.git
```

### 書き方
メインのプログラムはCore/Src/cpp_main.cppに記述してください。

# ライブラリの作成方法・追加方法
## そのプロジェクトでしか使わないライブラリ  
USER_Originalディレクトリ以下にあるIncフォルダとSrcフォルダにそれぞれヘッダファイルとソースファイルを作成してください。  
作成後、USER_Original/CMakeLists.txtに追加したファイル名を追記してください。

## サブモジュールライブラリの追加方法
今後更新します。

## 変更した設定(Pinout & Configuration)
### GPIO
#### Mode
- 割り込みピン(GPIO_EXTIxx)のModeを「External Interrupt Mode with Rising/Falling Edge trigger detection」に変更
#### Interrupt (NVIC Settings)
- 割り込みピン(GPIO_EXTIxx)の割り込みを有効化

### CAN
#### Bit Timings Parameters (Parameter Settings)
- Prescaler: 17
- Time Quanta in Bit Segment 1: 8
- Time Quanta in Bit Segment 2: 1
#### Basic Parameters
- Automatic Bus-Off Management: Enable
- Automatic Retransmission: Enable
#### Interrupt (NVIC Settings)
- FDCAN interrupt 0のみ有効化

### I2C
#### Mode
- I2C Mode: I2C
#### Master Features (Parameter Settings)
- I2C Speed Mode: Fast Mode
- I2C Clock Speed: 400000
#### Interrupt (NVIC Settings)
- Event interruptのみ有効化

### USART
#### Mode
- USART Mode: Asynchronous
#### Basic Parameters (Parameter Settings)
- Baud Rate: 115200

## 変更した設定(Clock Configuration)
全て170MHzに設定

# 変更した設定(Project Manager)
### Project
- Toolchain / IDE: CMake
### Code Generator
- STM32Cube MCU packages...: Copy all used libraries into the project
- Generated files: Generate peripheral initialization as a...を有効化