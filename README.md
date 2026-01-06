# Dyson Cool BLE Controller

BLE peripheral for nRF54L15-DK that controls a Dyson Cool fan via IR. Send commands over BLE, outputs 38kHz modulated IR.

## Hardware
- **Board:** nRF54L15-DK
- **IR TX:** P1.14 (connect to IR LED with driver circuit)

## BOM

| Part | Link |
|------|------|
| Adafruit IR Remote Transceiver (STEMMA JST PH 2mm) | [Adafruit 5990](https://www.adafruit.com/product/5990) |
| JST PH 2mm 4-Pin to Female Socket Cable | [Adafruit 3950](https://www.adafruit.com/product/3950) |

<p align="center">
  <img src="https://cdn-shop.adafruit.com/970x728/5990-00.jpg" width="300" alt="IR Transceiver"/>
</p>

<p align="center">
  <img src="https://cdn-shop.adafruit.com/970x728/3950-02.jpg" width="300" alt="JST Cable"/>
</p>

## BLE Commands
Write single byte to characteristic `12345678-1234-5678-1234-56789abcdef1`:

| Command | Byte | Action |
|---------|------|--------|
| Power | 0x01 | Toggle power |
| Oscillate | 0x02 | Toggle oscillation |
| Speed Up | 0x03 | Increase speed |
| Speed Down | 0x04 | Decrease speed |
| Timer Up | 0x05 | Increase timer |
| Timer Down | 0x06 | Decrease timer |

## Build
```bash
west build -b nrf54l15dk/nrf54l15/cpuapp app
west flash
```

## Python Client
See `scripts/dyson_ble_client.py` for a bleak-based CLI client.
