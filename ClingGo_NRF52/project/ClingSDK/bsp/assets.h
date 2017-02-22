/***************************************************************************//**
 * File: assets.h
 *
 * Created on Feb 28, 2014
 * 
 *****************************************************************************/

#include "standards.h"

#ifndef __ASSETS_HEADER__
#define __ASSETS_HEADER__

#include "assets\asset_len.h"
#include "assets\asset_pos.h"
#include "assets\asset_content.h"

#define ICON_DOW_IDX                         (132)
#define ICON_WEATHER_IDX                     (256+136)
#define ICON_CHARGING_IDX                    (155)
#define ICON_CHARGING_BATTERY_IDX            (156)

#define ICON_TOP_BATTERY_NOCHARGING_LEN      asset_len[128]
#define ICON_TOP_BATTERY_NOCHARGING          asset_pos[128]
#define ICON_TOP_BATTERY_CHARGING_LEN        asset_len[129]
#define ICON_TOP_BATTERY_CHARGING            asset_pos[129]
#define ICON_TOP_REMINDER_LEN                asset_len[256+128]
#define ICON_TOP_REMINDER                    asset_pos[256+128]
#define ICON_TOP_HEART_RATE_LEN              asset_len[256+129]
#define ICON_TOP_HEART_RATE                  asset_pos[256+129]
#define ICON_TOP_SKIN_TEMP_LEN               asset_len[256+130]
#define ICON_TOP_SKIN_TEMP                   asset_pos[256+130]
#define ICON_TOP_SLEEP_LEN                   asset_len[256+131]
#define ICON_TOP_SLEEP                       asset_pos[256+131]
#define ICON_TOP_STEPS_LEN                   asset_len[256+132]
#define ICON_TOP_STEPS                       asset_pos[256+132]
#define ICON_TOP_CALORIES_LEN                asset_len[256+133]
#define ICON_TOP_CALORIES                    asset_pos[256+133]
#define ICON_TOP_DISTANCE_LEN                asset_len[256+134]
#define ICON_TOP_DISTANCE                    asset_pos[256+134]
#define ICON_TOP_UV_LEN                      asset_len[256+135]
#define ICON_TOP_UV                          asset_pos[256+135]
#define ICON_TOP_W_SUNNY_LEN                 asset_len[256+136]
#define ICON_TOP_W_SUNNY                     asset_pos[256+136]
#define ICON_TOP_W_CLOUDY_LEN                asset_len[256+137]
#define ICON_TOP_W_CLOUDY                    asset_pos[256+137]
#define ICON_TOP_W_SNOWY_LEN                 asset_len[256+138]
#define ICON_TOP_W_SNOWY                     asset_pos[256+138]
#define ICON_TOP_W_RAINY_LEN                 asset_len[256+139]
#define ICON_TOP_W_RAINY                     asset_pos[256+139]
#define ICON_TOP_MESSAGE_LEN                 asset_len[256+140]
#define ICON_TOP_MESSAGE                     asset_pos[256+140]
#define ICON_TOP_RETURN_LEN                  asset_len[256+145]
#define ICON_TOP_RETURN                      asset_pos[256+145]
#define ICON_TOP_CALLS_LEN                   asset_len[256+160]
#define ICON_TOP_CALLS                       asset_pos[256+160]
#define ICON_TOP_OK_LEN                      asset_len[256+161]
#define ICON_TOP_OK                          asset_pos[256+161]

#define ICON_BOTTOM_IND_NO_CHARGING_LEN     asset_len[128]
#define ICON_BOTTOM_IND_NO_CHARGING         asset_pos[128]
#define ICON_BOTTOM_IND_CHARGING_LEN        asset_len[129]
#define ICON_BOTTOM_IND_CHARGING            asset_pos[129]
#define ICON_TOP_SMALL_BLE_LEN              asset_len[130]
#define ICON_TOP_SMALL_BLE                  asset_pos[130]
#define ICON_TOP_WORKOUT_START_TEXT_LEN     asset_len[143]
#define ICON_TOP_WORKOUT_START_TEXT         asset_pos[143]
#define ICON_TOP_WORKOUT_STOP_TEXT_LEN      asset_len[144]
#define ICON_TOP_WORKOUT_STOP_TEXT          asset_pos[144]
#define ICON_TOP_MORE_LEN                   asset_len[147]
#define ICON_TOP_MORE                       asset_pos[147]

#define ICON_TOP_WORKOUT_START_ICON_LEN     asset_len[256+147]
#define ICON_TOP_WORKOUT_START_ICON         asset_pos[256+147]
#define ICON_TOP_WORKOUT_STOP_ICON_LEN      asset_len[256+146]
#define ICON_TOP_WORKOUT_STOP_ICON          asset_pos[256+146]


#define ICON_MIDDLE_BLUETOOTH                         128
#define ICON_MIDDLE_CHARGING_BATT_IDX                 133
#define ICON_MIDDLE_AUTH_LINK_1                       151
#define ICON_MIDDLE_AUTH_LINK_2                       152
#define ICON_MIDDLE_AUTH_LINK_3                       150
#define ICON_MIDDLE_AUTH_LINK_4                       153
#define ICON_MIDDLE_AUTH_LINK_5                       154

#define ICON_BOTTOM_STEPS_LEN                asset_len[512+128]
#define ICON_BOTTOM_STEPS                    asset_pos[512+128]
#define ICON_BOTTOM_CALORIES_LEN             asset_len[512+129]
#define ICON_BOTTOM_CALORIES                 asset_pos[512+129]
#define ICON_BOTTOM_DISTANCE_LEN             asset_len[512+130]
#define ICON_BOTTOM_DISTANCE                 asset_pos[512+130]
#define ICON_BOTTOM_IDLE_LEN                 asset_len[512+131]
#define ICON_BOTTOM_IDLE                     asset_pos[512+131]

#define ICON_BOTTOM_IND_MON_LEN              asset_len[256+138]
#define ICON_BOTTOM_IND_MON                  asset_pos[256+138]

#define ICON_BOTTOM_IND_STEP_LEN                 asset_len[256+128]
#define ICON_BOTTOM_IND_STEP                     asset_pos[256+128]
#define ICON_BOTTOM_IND_CALS_LEN                 asset_len[256+129]
#define ICON_BOTTOM_IND_CALS                     asset_pos[256+129]
#define ICON_BOTTOM_IND_DIST_LEN                 asset_len[256+130]
#define ICON_BOTTOM_IND_DIST                     asset_pos[256+130]
#define ICON_BOTTOM_IND_HR_LEN                   asset_len[256+131]
#define ICON_BOTTOM_IND_HR                       asset_pos[256+131]
#define ICON_BOTTOM_IND_TEMP_LEN                 asset_len[256+132]
#define ICON_BOTTOM_IND_TEMP                     asset_pos[256+132]
#define ICON_BOTTOM_IND_SLEEP_LEN                asset_len[256+133]
#define ICON_BOTTOM_IND_SLEEP                    asset_pos[256+133]
#define ICON_BOTTOM_IND_WEATHER_LEN              asset_len[256+134]
#define ICON_BOTTOM_IND_WEATHER                  asset_pos[256+134]
#define ICON_BOTTOM_IND_ALARM_LEN                asset_len[256+135]
#define ICON_BOTTOM_IND_ALARM                    asset_pos[256+135]
#define ICON_BOTTOM_IND_MENU_LEN                 asset_len[256+147]
#define ICON_BOTTOM_IND_MENU                     asset_pos[256+147]
#define ICON_BOTTOM_IND_MAX_LEN                 asset_len[256+141]
#define ICON_BOTTOM_IND_MAX                     asset_pos[256+141]
#define ICON_BOTTOM_IND_MIN_LEN                 asset_len[256+142]
#define ICON_BOTTOM_IND_MIN                     asset_pos[256+142]

#define ICON32_LOGO_LEN                      asset_len[768+128]
#define ICON32_LOGO                          asset_pos[768+128]

#define ICON32_CLING_DEV_LEN                 asset_len[256+136]
#define ICON32_CLING_DEV                     asset_pos[256+136]
#define ICON32_CLING_DEV_IDX                 136
#define ICON32_AUTHING_LEN                   asset_len[256+148]
#define ICON32_AUTHING                       asset_pos[256+148]
#define ICON32_AUTHING_IDX                   148
#define ICON32_IPHONE_LEN                    asset_len[768+128]
#define ICON32_IPHONE                        asset_pos[768+128]
#define ICON32_IPHONE_IDX                    128

#define ICON32_PHONE_FINDER_1                142
#define ICON32_PHONE_FINDER_2                143
#define ICON32_PHONE_FINDER_3                144
#define ICON32_PHONE_FINDER_4                140

#define ICON32_BLUETOOTH_LEN                 asset_len[768+131]
#define ICON32_BLUETOOTH                     asset_pos[768+131]
#define ICON32_LINK_FAIL_LEN                 asset_len[768+132]
#define ICON32_LINK_FAIL                     asset_pos[768+132]
#define ICON32_LINK_OK_LEN                   asset_len[768+133]
#define ICON32_LINK_OK                       asset_pos[768+133]

#define ICON32_NONE_CAROUSEL                 0

#define ICON32_STEP_LEN                      asset_len[512+128]
#define ICON32_STEP                          asset_pos[512+128]
#define ICON32_STEP_IDX                      128
#define ICON32_CALORIES_LEN                  asset_len[512+129]
#define ICON32_CALORIES                      asset_pos[512+129]
#define ICON32_CALORIES_IDX                  129
#define ICON32_DISTANCE_LEN                  asset_len[512+130]
#define ICON32_DISTANCE                      asset_pos[512+130]
#define ICON32_DISTANCE_IDX                  130
#define ICON32_BALANCE_LEN                        asset_len[512+132]
#define ICON32_BALANCE                            asset_pos[512+132]
#define ICON32_BALANCE_IDX                        132
#define ICON32_TIME_LEN                      asset_len[512+135]
#define ICON32_TIME                          asset_pos[512+135]
#define ICON32_TIME_IDX                      135
#define ICON32_WEATHER_LEN                   asset_len[512+136]
#define ICON32_WEATHER                       asset_pos[512+136]
#define ICON32_WEATHER_IDX                   136
#define ICON32_WORKOUT_LEN                   asset_len[512+137]
#define ICON32_WORKOUT                       asset_pos[512+137]
#define ICON32_WORKOUT_IDX                   137
#define ICON32_STOPWATCH_LEN                 asset_len[512+138]
#define ICON32_STOPWATCH                     asset_pos[512+138]
#define ICON32_STOPWATCH_IDX                 138
#define ICON32_MESSAGE_LEN                   asset_len[512+139]
#define ICON32_MESSAGE                       asset_pos[512+139]
#define ICON32_MESSAGE_IDX                   139
#define ICON32_ALARM_LEN                 asset_len[512+141]
#define ICON32_ALARM                 asset_pos[512+141]
#define ICON32_ALARM_IDX                 141
#define ICON32_SETTING_LEN                 asset_len[512+150]
#define ICON32_SETTING                     asset_pos[512+150]
#define ICON32_SETTING_IDX                 150

#endif /* __ASSETS_HEADER__ */

/** @} */
