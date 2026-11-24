#include <NimBLEDevice.h>
#include <esp_mac.h>

#include "Device.h"
#include "FurbleTypes.h"

namespace Furble {

Device::uuid128_t Device::m_Uuid;
char Device::m_StringID[DEVICE_ID_STR_MAX];
std::string Device::m_ID;

/**
 * Generate a 32-bit PRNG.
 */
static uint32_t xorshift(uint32_t x) {
  /* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
  x ^= x << 13;
  x ^= x << 17;
  x ^= x << 5;
  return x;
}

uint64_t Device::getEfuseMac(void) {
  uint64_t _chipmacid = 0LL;
  esp_efuse_mac_get_default((uint8_t *)(&_chipmacid));
  return _chipmacid;
}

void Device::init(esp_power_level_t power) {
  uint64_t mac = getEfuseMac();
  uint32_t chip_id = (uint32_t)mac;
  for (size_t i = 0; i < UUID128_AS_32_LEN; i++) {
    chip_id = xorshift(chip_id);
    m_Uuid.uint32[i] = chip_id;
  }

  // 4.9.2: BLE 名称固定为 sony-gps-XXXX，XXXX 取 MAC 低 16 位
  // （每台设备唯一、可复现，不随开机变化）
  snprintf(m_StringID, DEVICE_ID_STR_MAX, "sony-gps-%04X", (unsigned)(mac & 0xFFFF));

  m_ID = std::string(m_StringID);

  NimBLEDevice::init(m_ID);
  NimBLEDevice::setPower(power);

  // configure RPA (where possible) and distribute encryption key and ID key (IRK)
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT);
}

Device::uuid128_t Device::getUUID128(void) {
  return m_Uuid;
}

const std::string Device::getStringID(void) {
  return m_ID;
}

}  // namespace Furble
