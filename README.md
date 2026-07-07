# Predictive Maintenance — STM32G431RB 센서 수집 (예지보전)

STM32G431RB Nucleo에서 **MPU6050 + ADXL345(진동 가속도) + DS18B20(온도)** 3센서를 읽어 USART2(ST-LINK VCP)로 시리얼 출력하는 펌웨어. HAL + FreeRTOS, 클럭은 HSI 16MHz. (예지보전 프로젝트의 Phase 1 — 향후 ADXL345 1kHz 샘플링 → FFT → 진동 특징 추출로 확장 예정.)

```
[S] MPU0 x=-5 y=-2 z=954 | ADXL0 X=77 Y=124 Z=974 | T1=27.06C
```

## 하드웨어

| 부품 | 용도 |
|------|------|
| Nucleo-G431RB (STM32G431RB) | MCU |
| MPU6050 (GY-521) | IMU 가속도/자이로, I2C |
| ADXL345 (GY-291) | 진동 가속도계, I2C |
| DS18B20 방수 프로브 | 온도, 1-Wire |
| 10kΩ 저항 | DS18B20 1-Wire 풀업 |

## 배선도

### MPU6050 + ADXL345 → I2C1 공유 버스 (PB8/PB9)
두 센서를 **같은 I2C1 버스**에 병렬로 연결. 주소가 달라 충돌 없음(MPU6050=0x68, ADXL345=0x53).

```
   MPU6050 (GY-521)              ADXL345 (GY-291)            Nucleo-G431RB
   ┌──────────┐                  ┌──────────┐                ┌──────────────┐
   │ VCC ─────┼───── 3V3 ────────┼─ VCC     │                │ 3V3          │
   │ GND ─────┼───── GND ────────┼─ GND     │                │ GND          │
   │ SCL ─────┼─────┬────────────┼─ SCL     │ ───────────────┤ PB8 (I2C1_SCL)  CN10-3
   │ SDA ─────┼─────┴────────────┼─ SDA     │ ───────────────┤ PB9 (I2C1_SDA)  CN10-5
   └──────────┘                  │ SDO ─────┼── GND (주소 0x53)
                                 │ CS  ─────┼── 3V3 (I2C 모드 강제)
                                 └──────────┘
   ※ 두 모듈 모두 보드 내장 4.7k 풀업 → 버스 풀업 OK
```

### DS18B20 → 1-Wire (PB3) + 10kΩ 풀업
```
   DS18B20 방수 프로브                       Nucleo-G431RB
   ┌──────────┐                             ┌──────────┐
   │ 빨강 VCC ┼──────── 3V3 ────────────────┤ 3V3      │
   │ 파랑 GND ┼──────── GND ────────────────┤ GND      │
   │ 노랑 DQ ─┼────┬────────────────────────┤ PB3      │  CN10-31
   └──────────┘   │                         └──────────┘
                  └──[ 10kΩ ]── 3V3   ← 1-Wire 풀업 (필수)
```

### 디버그 시리얼 / LED
- **USART2 PA2(TX)** → ST-LINK VCP → `/dev/ttyACM0` (별도 배선 불필요, Nucleo USB 하나로)
- **LED PA5** (LD4) — 센서 루프마다 토글

## 핀 할당

| 핀 | 기능 | CN 위치 | 비고 |
|----|------|---------|------|
| PB8 | I2C1_SCL | CN10-3 | MPU6050 + ADXL345 공유 |
| PB9 | I2C1_SDA | CN10-5 | MPU6050 + ADXL345 공유 |
| PB3 | GPIO(1-Wire) | CN10-31 | DS18B20 DQ + 10kΩ 풀업 |
| PA2 | USART2_TX | (VCP) | 디버그 출력 → /dev/ttyACM0 |
| PA5 | GPIO(LED) | LD4 | 상태 표시 |

## ⚠️ 핵심 함정 (삽질 방지)

1. **G431에서 PB10/PB11은 I2C AF가 없다.** (데이터시트 DS12589 확인 — TIM2/USART3/LPUART1만 지원) → I2C2로 쓰면 버스 스캔이 `(none)`. 그래서 ADXL345를 I2C2가 아닌 **I2C1(PB8/PB9)에서 MPU6050과 공유**. (G431 I2C2 정상 핀은 PA9(SCL)/PA8(SDA).)
2. **클럭 = HSI 16MHz (PLL/HSE 미사용).** 이 보드에서 `HAL_RCC_OscConfig()`가 HSI 재설정 중 행하는 현상이 있어 `SystemClock_Config()`를 no-op로 두고 리셋 기본 HSI 16MHz 사용. `SystemCoreClock`·`configCPU_CLOCK_HZ` 모두 16M로 일치.
3. **`st-flash --reset` 필수.** 리셋 기본 클럭을 전제하므로 플래시 후 SRST가 안 걸리면 이전 클럭이 잔존해 동작이 깨짐.
4. **DS18B20 풀업 필수** (내부 풀업만으론 타이밍 불안정 → presence 실패). DQ–3V3 간 4.7~10kΩ.
5. **1-Wire 비트백은 open-drain + 직접 레지스터** (BSRR/IDR). 매 비트 `HAL_GPIO_Init` 모드 전환은 느려 read 슬롯 타이밍이 붕괴 → 0xFF 읽힘.
6. **ADXL345**: SDO→GND(주소 0x53), CS→3V3(I2C 모드).

## 빌드 / 플래시

```bash
# 빌드
make

# 플래시 (★ --reset 필수)
make flash
# 또는 직접:
st-flash --reset write build/predictive-maintenance.bin 0x08000000
```

요구: `arm-none-eabi-gcc`, `st-flash`(stlink-tools).

## 시리얼 모니터

```bash
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0
```

출력 예 (약 200ms 간격, 온도는 ~5s마다):
```
[B] I2C1(PB8/PB9) MPU6050+ADXL345 init done
[SCAN] I2C1: 0x53 0x68 (2 found)
[B] DS18B20(1-Wire PB3) init done
[S] MPU0 x=-5 y=-2 z=954 | ADXL0 X=77 Y=124 Z=974 | T1=27.06C
```

## 구조

```
Core/
  Inc/   main.h, mpu6050.h, adxl345.h, ds18b20.h, uart_debug.h, FreeRTOSConfig.h
  Src/   main.c, mpu6050.c, adxl345.c, ds18b20.c, uart_debug.c, stm32g4xx_it.c, system_stm32g4xx.c
Drivers/   STM32G4xx_HAL_Driver, CMSIS  (vendored)
Middlewares/FreeRTOS-Kernel            (vendored)
Makefile, STM32G431RBTX_FLASH.ld, startup_stm32g431xx.s
```

## 로드맵
- [x] Phase 1: 3센서 데이터 수집
- [x] Phase 2a: ADXL345 ~1kHz 샘플링 → 256-pt FFT → 진동 특징(RMS/kurtosis/대역에너지)
- [ ] Phase 2b: 이상탐지 임계 + 알림
- [ ] Phase 3: STM32 ↔ RPi3 프레임 프로토콜 → 클라우드 대시보드

## Phase 2a — 진동 분석 파이프라인

ADXL345 z축을 ~1kHz로 256샘플 채취 → Hann 창 → 자체 radix-2 FFT → 진단 특징 추출.

```
[VIB] rms=27 peak=110 p2p=190 kurt=1.5 crest=3.8 | f0=320.3Hz | lo=.. mid=.. hi=..
[BG] T1=28.38C
```

특징:
- **시간영역**: RMS(진동 심도), peak, peak-to-peak, kurtosis(초과 첨도 — Gaussian≈0, 충격성↑), crest factor
- **주파수영역**: 지배 주파수(f0), 대역 에너지(low 0-50Hz / mid 50-200Hz / high 200-500Hz)

설계 메모:
- 샘플링은 `vTaskDelayUntil(1ms)` 페이싱(RTOS 틱 평균 1kHz). TIM6 ISR 기반 정밀 샘플링은 이 보드에서 폴트/플러드 이슈가 있어 TODO.
- I2C1은 100kHz(MPU6050+ADXL345 공유). **부팅 시 I2C 버스 리커버리**(SCL 9클럭+STOP)로 슬레이브 락업 자동 복구.
- DS18B20 1-Wire 비트백은 **크리티컬 섹션**으로 묶어 고속 샘플링 태스크의 선점으로부터 μs 타이밍 보호.
- DSP는 자체 구현(`Core/Src/dsp.c`, 의존성 无). 클럭 HSI 16MHz, 빌드 `-O2`.
