#pragma once

#include <Arduino.h>
#include <SPIFFS.h>

// Set serial for debug console (to the Serial Monitor)
#define SerialMon Serial
// Set serial for LoRa (to the module)
#define SerialLoRa Serial1

// E220-900T22S(JP)のLoRa設定ファイル
#define CONFIG_FILENAME "/e220900t22s_jp_lora_config.ini"

// E220-900T22S(JP)へのピンアサイン
#define LoRa_ModeSettingPin_M0 12
#define LoRa_ModeSettingPin_M1 13
#define LoRa_RxPin 26
#define LoRa_TxPin 27
#define LoRa_AUXPin 39

// E220-900T22S(JP)のbaud rate
#define LoRa_BaudRate 9600

// E220-900T22S(JP)の設定項目
struct LoRaConfigItem_t {
  uint16_t own_address;
  uint8_t baud_rate;
  uint8_t air_data_rate;
  uint8_t subpacket_size;
  uint8_t rssi_ambient_noise_flag;
  uint8_t transmitting_power;
  uint8_t own_channel;
  uint8_t rssi_byte_flag;
  uint8_t transmission_method_type;
  uint16_t wor_cycle;
  uint16_t encryption_key;
  uint16_t target_address;
  uint8_t target_channel;
};

struct RecvFrameE220900T22SJP_t {
  uint8_t recv_data[201];
  uint8_t recv_data_len;
  int rssi;
};

class CLoRa {
public:
  /**
   * @brief E220-900T22S(JP)のLoRa設定値を読み込む
   * @param filename 設定ファイル名
   * @param config 読み込んだLoRa設定値の格納先
   * @return 0:成功 1:失敗
   */
  int LoadConfigSetting(const char *filename, struct LoRaConfigItem_t &config);

  /**
   * @brief E220-900T22S(JP)のLoRa設定値をデフォルトの値にセット
   * @param config LoRa設定値の格納先
   */
  void SetDefaultConfigValue(struct LoRaConfigItem_t &config);

  /**
   * @brief E220-900T22S(JP)へLoRa初期設定を行う
   * @param config LoRa設定値の格納先
   * @return 0:成功 1:失敗
   */
  int InitLoRaModule(struct LoRaConfigItem_t &config);

  /**
   * @brief LoRa受信を行う
   * @param recv_data LoRa受信データの格納先
   * @return 0:成功 1:失敗
   */
  int receiveFrame(struct RecvFrameE220900T22SJP_t *recv_frame);

  /**
   * @brief LoRa送信を行う
   * @param config LoRa設定値の格納先
   * @param send_data 送信データ
   * @param size 送信データサイズ
   * @return 0:成功 1:失敗
   */
  int SendFrame(struct LoRaConfigItem_t &config, uint8_t *send_data, int size);

  /**
   * @brief ノーマルモード(M0=0,M1=0)へ移行する
   */
  void SwitchToNormalMode(void);

  /**
   * @brief WOR送信モード(M0=1,M1=0)へ移行する
   */
  void SwitchToWORSendingMode(void);

  /**
   * @brief WOR受信モード(M0=0,M1=1)へ移行する
   */
  void SwitchToWORReceivingMode(void);

  /**
   * @brief コンフィグモード(M0=1,M1=1)へ移行する
   */
  void SwitchToConfigurationMode(void);

private:
  uint16_t own_address_val;
  uint32_t baud_rate_val;
  uint16_t bw_val;
  uint8_t sf_val;
  uint8_t subpacket_size_val;
  uint8_t transmitting_power_val;
  uint8_t own_channel_val;
  uint16_t wor_cycle_val;
  uint16_t encryption_key_val;
  uint16_t target_address_val;
  uint8_t target_channel_val;

  int OpenConfigFile(const char *filename, struct LoRaConfigItem_t &config);
  int ReadConfigValue(const char *key, const char *val);
  int SetConfigValue(struct LoRaConfigItem_t &config);
};
