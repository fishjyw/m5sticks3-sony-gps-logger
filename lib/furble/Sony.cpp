#include <NimBLEAddress.h>
#include <NimBLEAdvertisedDevice.h>
#include <NimBLEDevice.h>
#include <NimBLERemoteCharacteristic.h>
#include <NimBLERemoteService.h>
#include <NimBLEUtils.h>

#include "Sony.h"

namespace Furble {

Sony::Sony(const void *data, size_t len) : Camera(Type::SONY, PairType::SAVED) {
  if (len != sizeof(sony_t))
    abort();

  const sony_t *sony = static_cast<const sony_t *>(data);
  m_Name = std::string(sony->name);
  m_Address = NimBLEAddress(sony->address, sony->type);
  memcpy(&m_Uuid, &sony->uuid, sizeof(Device::uuid128_t));
}

Sony::Sony(const NimBLEAdvertisedDevice *pDevice) : Camera(Type::SONY, PairType::NEW) {
  m_Name = pDevice->getName();
  m_Address = pDevice->getAddress();
  ESP_LOGI(LOG_TAG, "Name = %s", m_Name.c_str());
  ESP_LOGI(LOG_TAG, "Address = %s", m_Address.toString().c_str());
  m_Uuid = Device::getUUID128();
}

bool Sony::matches(const NimBLEAdvertisedDevice *pDevice) {
  if (pDevice->haveManufacturerData()
      && (pDevice->getManufacturerData().size() >= sizeof(sony_adv_t))) {
    const sony_adv_t adv = pDevice->getManufacturerData<sony_adv_t>();
    if (adv.company_id == ADV_SONY_ID && adv.type == ADV_SONY_CAMERA) {
      uint8_t mode = (ADV_MODE22_PAIRING_SUPPORTED | ADV_MODE22_PAIRING_ENABLED);
      if ((adv.mode22 & mode) != mode) {
        return false;
      }
      // Remote-enabled: bit1 (ZV/A7M5 gen) or bit2 (A7R4 gen).
      // Also accept a camera in "location linkage" pairing mode (bit5),
      // which is how the A7R4 generation exposes GPS without remote ctrl.
      return (adv.mode22
              & (ADV_MODE22_REMOTE_ENABLED | ADV_MODE22_UNKNOWN_01 | ADV_MODE22_LOCATION_SUPPORTED))
             != 0;
    }
  }

  return false;
}

bool Sony::_connect(void) {
  ESP_LOGI(LOG_TAG, "Connecting");
  NimBLEDevice::setMTU(158);  // 参考实现 exchange_mtu(158)
  if (!m_Client->connect(m_Address)) {
    ESP_LOGI(LOG_TAG, "Connection failed!!!");
    return false;
  }

  ESP_LOGI(LOG_TAG, "Connected");
  m_Progress = 20;

  ESP_LOGI(LOG_TAG, "Securing");
  if (!m_Client->secureConnection()) {
    return false;
  }
  ESP_LOGI(LOG_TAG, "Secured!");
  // 放宽连接间隔（100~150ms + latency 4）：GPS 1Hz 推送足够，显著省电
  m_Client->updateConnParams(80, 120, 4, 400);
  m_Progress = 40;

  ESP_LOGI(LOG_TAG, "Retrieving control service!");
  auto *pSvc = m_Client->getService(CTRL_SVC_UUID);
  if (pSvc != nullptr) {
    m_Control = pSvc->getCharacteristic(CTRL_CHR_UUID);
    ESP_LOGI(LOG_TAG, "Control service: %s", m_Control ? "OK" : "no characteristic");
  } else {
    ESP_LOGW(LOG_TAG, "No control service (location-only link)");
  }
  m_Progress = 80;

  ESP_LOGI(LOG_TAG, "Retrieving location service");
  m_GeoSvc = m_Client->getService(GEO_SVC_UUID);
  if (m_GeoSvc != nullptr) {
    ESP_LOGI(LOG_TAG, "Location service FOUND");
  } else {
    ESP_LOGW(LOG_TAG, "NO location service on this camera/link");
  }
  if (m_Control == nullptr && m_GeoSvc == nullptr) {
    ESP_LOGE(LOG_TAG, "Neither control nor location service - unusable");
    return false;
  }

  // 新代机型（A7M5 等）需要 dd30/dd31 使能；旧代（A7R4）会拒绝但无害，
  // 旧代走 updateGeoData() 里的直接 dd11 写入。
  if (m_GeoSvc != nullptr) {
    m_GeoSvc->setValue(GEO_ALLOW_CHR_UUID, {0x00});   // 解锁
    m_GeoSvc->setValue(GEO_ENABLE_CHR_UUID, {0x01});  // 启用
  }

  m_Progress = 100;
  ESP_LOGI(LOG_TAG, "Done!");

  return true;
}

void Sony::_disconnect(void) {
  closeClient();
}

void Sony::shutterPress(void) {
  if (m_Control) m_Control->writeValue(SHUTTER_DOWN, true);
  return;
}

void Sony::shutterRelease(void) {
  if (m_Control) m_Control->writeValue(SHUTTER_UP, true);
  return;
}

void Sony::focusPress(void) {
  if (m_Control) m_Control->writeValue(FOCUS_DOWN, true);
  return;
}

void Sony::focusRelease(void) {
  if (m_Control) m_Control->writeValue(FOCUS_UP, true);
}

bool Sony::locationEnabled(void) {
  if (m_GeoSvc) {
    auto allowValue = m_GeoSvc->getValue(GEO_ALLOW_CHR_UUID);
    auto enabledValue = m_GeoSvc->getValue(GEO_ENABLE_CHR_UUID);
    uint8_t allow = allowValue.size() > 0 ? allowValue.data()[0] : 0xFF;
    uint8_t enabled = enabledValue.size() > 0 ? enabledValue.data()[0] : 0xFF;

    // 无条件尝试写入（R4 上读值可能为空）
    m_GeoSvc->setValue(GEO_ALLOW_CHR_UUID, {LOCATION_ALLOW});
    m_GeoSvc->setValue(GEO_ENABLE_CHR_UUID, {LOCATION_ENABLE});

    allowValue = m_GeoSvc->getValue(GEO_ALLOW_CHR_UUID);
    enabledValue = m_GeoSvc->getValue(GEO_ENABLE_CHR_UUID);
    if (allowValue.size() > 0 && enabledValue.size() > 0) {
      allow = allowValue.data()[0];
      enabled = enabledValue.data()[0];
      // ESP_LOGI(LOG_TAG, "1: allow = %x, enabled = %x", allow, enabled);
      if (allow != LOCATION_ALLOW && m_GeoSvc->setValue(GEO_ALLOW_CHR_UUID, {LOCATION_ALLOW})) {
        allowValue = m_GeoSvc->getValue(GEO_ALLOW_CHR_UUID);
        allow = allowValue.data()[0];
        ESP_LOGI(LOG_TAG, "Location service allowed!");
      }
      if (enabled != LOCATION_ENABLE
          && m_GeoSvc->setValue(GEO_ENABLE_CHR_UUID, {LOCATION_ENABLE})) {
        enabledValue = m_GeoSvc->getValue(GEO_ENABLE_CHR_UUID);
        enabled = enabledValue.data()[0];
        ESP_LOGI(LOG_TAG, "Location service enabled!");
      }

      // auto info = m_GeoSvc->getValue(GEO_INFO_CHR_UUID);
      // ESP_LOGI(LOG_TAG, "info = %s",
      // NimBLEUtils::dataToHexString(info.data(), info.size()).c_str());

      // ESP_LOGI(LOG_TAG, "2: allow = %x, enabled = %x", allow, enabled);
      ESP_LOGI(LOG_TAG, "geo after write: allow=0x%02x enable=0x%02x", allow, enabled);
      if (allow == LOCATION_ALLOW && enabled == LOCATION_ENABLE) {
        return true;
      }
    } else {
    }
  } else {
    ESP_LOGW(LOG_TAG, "locationEnabled: no GEO service");
  }

  return false;
}

void Sony::updateGeoData(const gps_t &gps, const timesync_t &timesync) {
  // R4 代机型不认 dd30/dd31 使能（读空、写拒），但直接写 dd11 有效
  // （参考 anoulis/sony_camera_bluetooth_external_gps 已验证实现）
  {
    sony_geo_t geo = {
        .prefix = {0x00, 0x5d, 0x08, 0x02, 0xfc, 0x03, 0x00, 0x00, 0x10, 0x10, 0x10},
        .latitude = (int32_t)__builtin_bswap32((uint32_t)(gps.latitude * 10000000)),
        .longitude = (int32_t)__builtin_bswap32((uint32_t)(gps.longitude * 10000000)),
        .year = __builtin_bswap16((uint16_t)timesync.year),
        .month = (uint8_t)timesync.month,
        .day = (uint8_t)timesync.day,
        .hour = (uint8_t)timesync.hour,
        .minute = (uint8_t)timesync.minute,
        .second = (uint8_t)timesync.second,
        .zero0 = {0x00},
        .offset = __builtin_bswap16(0x0000),
        .zero1 = {0x00},
    };
    // ESP_LOGI(LOG_TAG, "geo: %s", NimBLEUtils::dataToHexString((const uint8_t *)&geo,
    // sizeof(geo)).c_str());
    auto *pChr = m_GeoSvc->getCharacteristic(GEO_UPDATE_UUID);
    if (pChr == nullptr) {
      ESP_LOGW(LOG_TAG, "no GEO_UPDATE characteristic");
      return;
    }
    bool ok = pChr->writeValue((const uint8_t *)&geo, sizeof(geo), true);
    static int n = 0;
    if (n < 3 || (n % 60) == 0) {
      ESP_LOGI(LOG_TAG, "geo push #%d %s", n, ok ? "ok" : "FAILED");
    }
    n++;
  }
}

size_t Sony::getSerialisedBytes(void) const {
  return sizeof(sony_t);
}

bool Sony::serialise(void *buffer, size_t bytes) const {
  if (bytes != sizeof(sony_t)) {
    return false;
  }
  sony_t *x = static_cast<sony_t *>(buffer);
  strncpy(x->name, m_Name.c_str(), MAX_NAME);
  x->address = (uint64_t)m_Address;
  x->type = m_Address.getType();
  memcpy(&x->uuid, &m_Uuid, sizeof(Device::uuid128_t));

  return true;
}

}  // namespace Furble
