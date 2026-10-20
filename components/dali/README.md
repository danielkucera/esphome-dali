The ESPHome custom component uses the MIT-licensed `esp-idf-dali` RMT driver for
DALI transmit and receive. Configure the ESP32 with the ESP-IDF framework.
The driver is fetched from the `esp-idf-5.5-support` branch of
`github.com/danielkucera/esp-idf-dali` (see `DALI_LIB_REPOSITORY` in
`__init__.py`), which adds ESP-IDF 5.x RMT support, a PlatformIO manifest and
`dali_init_with_inversion()`, used here with non-inverted TX for the board's
inverting transistor driver. Once merged upstream, point `DALI_LIB_REPOSITORY`
at `ferdinandkeil/esp-idf-dali`. The component adapts the driver's transaction
API to the DALI port interface used for lights and address discovery. The
driver's `resetBus()` pulse operation is not available through its RMT API.

The separate PlatformIO library/demo remains Arduino-compatible and builds the
core DALI protocol code independently of the ESPHome component.
