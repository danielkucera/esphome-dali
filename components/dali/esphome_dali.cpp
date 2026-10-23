#include <esphome.h>
#include <esp_task_wdt.h>
#include "esphome_dali.h"
#include "esphome_dali_light.h"
#include "esp_idf_dali_api.h"
#include "port.h"

using namespace esphome;
using namespace dali;

namespace {

class AppRegistrationAccessor : public esphome::Application {
public:
    using Application::register_component_;
};

class DynamicDaliLightState : public esphome::light::LightState {
public:
    using esphome::light::LightState::LightState;

    void configure_dynamic_entity(const char* name, const char* object_id, bool disabled_by_default) {
        uint32_t entity_fields = (static_cast<uint32_t>(disabled_by_default) << esphome::ENTITY_FIELD_DISABLED_BY_DEFAULT_SHIFT);
        this->configure_entity_(name, esphome::fnv1_hash_object_id(object_id, std::strlen(object_id)), entity_fields);
    }
};

}  // namespace

void DaliBusComponent::setup() {
    m_txPin->setup();
    m_rxPin->setup();
    const auto err = dali_init_with_inversion(
        static_cast<gpio_num_t>(m_rxPin->get_pin()),
        static_cast<gpio_num_t>(m_txPin->get_pin()),
        m_invertTx,
        m_invertRx);
    if (err != ESP_OK) {
        DALI_LOGE("Failed to initialize DALI RMT interface: %s", esp_err_to_name(err));
        this->mark_failed();
        return;
    }
    DALI_LOGI("DALI bus ready");

    if (m_discovery) {
        // Optional: reset devices on the bus so we are in a known-good state.
        // Can help if devices are not responding to anything.
        if (false) {
            this->resetBus();
            esp_task_wdt_reset();
        }

        if (dali.bus_manager.isControlGearPresent()) {
            DALI_LOGD("Detected control gear on bus");
        } else {
            DALI_LOGE("No DALI control gear detected on bus!");
            return; // Unlikely to get anything from discovery if no one responds to this
        }

        if (this->m_initialize_addresses != DaliInitMode::DiscoverOnly) {
            if (this->m_initialize_addresses == DaliInitMode::InitializeAll) {
                DALI_LOGI("Randomizing addresses for *all* DALI devices");
                dali.bus_manager.initialize(ASSIGN_ALL); 
            } 
            else if (this->m_initialize_addresses == DaliInitMode::InitializeUnassigned) {
                // Only randomize devices without an assigned short address
                DALI_LOGI("Randomizing addresses for unassigned DALI devices");
                dali.bus_manager.initialize(ASSIGN_UNINITIALIZED); 
            }

            dali.bus_manager.randomize();
            dali.bus_manager.terminate();

            // Seem to need a delay to allow time for devices to randomize...
            delayMilliseconds(50);
        }

        DALI_LOGI("Begin device discovery...");
        dali.bus_manager.startAddressScan(); // All devices

        // Keep track of short addresses to detect duplicates
        bool duplicate_detected = false;
        bool is_discovered[ADDR_SHORT_MAX+1];
        for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
            is_discovered[i] = false;
        }

        uint8_t count = 0;
        short_addr_t short_addr = 0xFF;
        uint32_t long_addr = 0;
        while (dali.bus_manager.findNextAddress(short_addr, long_addr)) {
            count++;
            delayMilliseconds(1); // yield to ESP stack
            esp_task_wdt_reset();

            if (short_addr <= ADDR_SHORT_MAX) {
                DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);

                // Duplicate detection
                if (is_discovered[short_addr]) {
                    if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                        DALI_LOGW("  WARNING: Duplicate short address detected!");
                        duplicate_detected = true;
                    }
                    else {
                        // Assign a new address for this
                        short_addr++;
                        DALI_LOGD("  Duplicate short address detected, assigning a new address: %.2x", short_addr);

                        if (!dali.bus_manager.programShortAddress(short_addr)) {
                            DALI_LOGE("  Could not program short address");
                            dali.bus_manager.withdrawCurrentDevice();
                            short_addr = 0xFF;
                            continue;
                        }
                    }
                }
                else {
                    is_discovered[short_addr] = true;
                }

                // Withdraw after address is confirmed (spec order: find → program → withdraw)
                dali.bus_manager.withdrawCurrentDevice();

                // Dynamic component creation (if not defined in YAML)
                if (m_addresses[short_addr]) {
                    DALI_LOGD("  Ignoring, already defined");
                }
                else {
                    m_addresses[short_addr] = long_addr;
                    create_light_component(short_addr, long_addr);
                }
            }
            else if (short_addr == 0xFF) {
                if (m_initialize_addresses == DaliInitMode::DiscoverOnly) {
                    DALI_LOGI("  Device %.6x @ --", long_addr);
                    DALI_LOGW("  No short address assigned!");
                    dali.bus_manager.withdrawCurrentDevice();
                    continue;
                }
                else {
                    short_addr = count;
                    DALI_LOGI("  Assigning short address: %.2x", short_addr);

                    if (!dali.bus_manager.programShortAddress(short_addr)) {
                        DALI_LOGE("  Could not program short address");
                        dali.bus_manager.withdrawCurrentDevice();
                        short_addr = 0xFF;
                        continue;
                    }

                    // Withdraw after successful address programming
                    dali.bus_manager.withdrawCurrentDevice();

                    DALI_LOGI("  Device %.6x @ %.2x", long_addr, short_addr);
                }
            }
        }

        DALI_LOGD("No more devices found!");
        dali.bus_manager.endAddressScan();

        if (duplicate_detected) {
            DALI_LOGW("Duplicate short addresses detected on the bus!");
            DALI_LOGW("  Devices may report inconsistent capabilities.");
            DALI_LOGW("  You should fix your address assignments.");
        }
    }

    // If no dynamic lights were discovered, disable loop() to avoid unnecessary CPU cycles.
    if (m_dynamic_lights.empty()) {
        this->disable_loop();
    }
}

void DaliBusComponent::create_light_component(short_addr_t short_addr, uint32_t long_addr) {
#ifdef USE_LIGHT
    DaliLight* dali_light = new DaliLight { this };
    dali_light->set_address(short_addr);

    const int MAX_STR_LEN = 20;
    char* name = new char[MAX_STR_LEN];
    char* id = new char[MAX_STR_LEN];
    snprintf(name, MAX_STR_LEN, "DALI Light %d", short_addr);
    snprintf(id, MAX_STR_LEN, "dali_light_%.6x", long_addr);
    // NOTE: Not freeing these strings, they will be owned by LightState.

    auto* light_state = new DynamicDaliLightState { dali_light };
    // set_component_source is codegen-only since ESPHome 2026.4 (uint8_t index into PROGMEM table);
    // dynamic components cannot participate, and it was cosmetic (log source tagging) only.
    light_state->configure_dynamic_entity(name, id, false);
    App.register_light(light_state);
    static_cast<AppRegistrationAccessor&>(App).register_component_(light_state);

    // Equivalent of the former LIGHT_RESTORE_DEFAULT_ON: on by default, restored state otherwise.
    light_state->set_state_callback([](light::LightStateRTCState& state, bool restored) {
        if (!restored) {
            state.state = true;
        }
    });
    light_state->add_effects({});

    // Initialize the DaliLight with the LightState:
    // queries device capabilities (min/max level, color temperature support)
    dali_light->setup_state(light_state);

    // Track for manual loop() driving — dynamic lights are not in ESPHome's
    // compile-time looping_components_ list so their loop() won't be called automatically.
    if (m_dynamic_lights.empty()) {
        this->enable_loop();  // only enable loop when we have lights to drive
    }
    m_dynamic_lights.push_back(light_state);

    DALI_LOGI("Created light component '%s' (%s)", name, id);
#else
    DALI_LOGE("Not compiled with light component. Add `light:` to YAML.");
#endif
}

void DaliBusComponent::loop() {
    for (auto* light : m_dynamic_lights) {
        light->loop();
    }
}

void DaliBusComponent::dump_config() {
    static const char *const TAG = "dali";
    ESP_LOGCONFIG(TAG, "DALI Bus:");
    LOG_PIN("  TX Pin: ", m_txPin);
    LOG_PIN("  RX Pin: ", m_rxPin);
    ESP_LOGCONFIG(TAG, "  Discovery: %s", m_discovery ? "enabled" : "disabled");
    ESP_LOGCONFIG(TAG, "  Control Gear: %s", dali.bus_manager.isControlGearPresent() ? "present" : "not present");
    bool any = false;
    for (int i = 0; i <= ADDR_SHORT_MAX; i++) {
        if (m_addresses[i] > 0) {
            if (!any) {
                ESP_LOGCONFIG(TAG, "  Addresses:");
                any = true;
            }
            ESP_LOGCONFIG(TAG, "    %.2u = %.6x", i, m_addresses[i]);
        }
    }
}

void DaliBusComponent::resetBus() {
    DALI_LOGW("DALI reset pulse is not available with the RMT transport");
}

void DaliBusComponent::sendForwardFrame(uint8_t address, uint8_t data) {
    this->runTransaction(address, data, false, 0);
}

uint8_t DaliBusComponent::receiveBackwardFrame(unsigned long timeout_ms) {
    (void) timeout_ms;
    DALI_LOGE("Backward frames must be received as part of a DALI transaction");
    return 0;
}

uint8_t DaliBusComponent::transactForwardFrame(uint8_t address, uint8_t data, unsigned long timeout_ms) {
    return this->runTransaction(address, data, true, timeout_ms);
}

uint8_t DaliBusComponent::runTransaction(uint8_t address, uint8_t data, bool expect_response, unsigned long timeout_ms) {
    (void) timeout_ms;
    DALI_LOGD("TX: %02X %02X", address, data);

    dali_addressType_t address_type;
    uint8_t dali_address;
    bool is_command = (address & 1) != 0;

    if (address == 0xFE || address == 0xFF) {
        address_type = DALI_ADDRESS_TYPE_BROADCAST;
        dali_address = 0;
    } else if (address >= 0xA0) {
        address_type = DALI_ADDRESS_TYPE_SPECIAL_CMD;
        dali_address = address;
    } else if ((address & 0x80) != 0) {
        address_type = DALI_ADDRESS_TYPE_GROUP;
        dali_address = (address >> 1) & 0x0F;
    } else {
        address_type = DALI_ADDRESS_TYPE_SHORT;
        dali_address = address >> 1;
    }

    int result = DALI_RESULT_NO_REPLY;
    const auto err = dali_transaction(
        address_type,
        dali_address,
        is_command,
        data,
        false,
        DALI_TX_TIMEOUT_DEFAULT_MS,
        expect_response ? &result : nullptr);
    if (err != ESP_OK) {
        DALI_LOGE("DALI RMT transaction failed: %s", esp_err_to_name(err));
        return 0;
    }

    if (!expect_response) {
        DALI_LOGD("RX: -- (not requested)");
        return 0;
    }
    if (result == DALI_RESULT_NO_REPLY) {
        DALI_LOGD("RX: -- (no reply)");
        return 0;
    }

    DALI_LOGD("RX: %02X", result);
    return static_cast<uint8_t>(result);
}
