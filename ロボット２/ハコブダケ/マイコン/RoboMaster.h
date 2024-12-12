#ifndef ROBOMASTER_H_
#define ROBOMASTER_H_

#include "Utils.h"

#define ROBOMASTER_ID_1 0x200
#define ROBOMASTER_ID_2 0x1FF

namespace hakorobo2024
{
  /// @brief RoboMaster８台のCANメッセージを発行するクラス
  /// @note M3508(c620)とM2006(c610)をサポートする
  class RoboMaster
  {
    public:
    /// @brief RoboMaster８台の指令値を含む構造体
    /// @note ID1~4がbuf_1でID5~8がbuf_2
    struct Cmd
    {
      byte buf_1[8];
      byte buf_2[8];

      Cmd();
    };

    /// @brief RoboMaster８台のフィードバック値とIDを管理する構造体
    struct FeedBack
    {
      char buf[8];
      long unsigned int ID;

      FeedBack();
    };

    /// @brief モーターのタイプ
    /// @note Noneは初期化用
    enum Type
    {
      M2006,
      M3508,
      None,
    };

    /// @brief モーターのIDに対するタイプを紐付ける
    /// @param id モーターのID
    /// @param type モーターのタイプ
    void setType(uint8_t id, Type type);

    /// @brief あるIDのモーターの指令値[A]をセットする
    /// @param id モーターのID
    /// @param ampare 電流値
    void setCurrent(uint8_t id, int ampare);

    /// @brief CANで送信するバッファを作成する
    Cmd createCommand();

    /// @brief フィードバックを解析する
    /// @param data フィードバック構造体
    void setFeedBack(FeedBack data);

    /// @brief RPMフィードバックを取得する
    /// @param id モーターのID
    int getRPM(uint8_t id);

    /// @brief 角度フィードバックを取得する
    /// @param id モーターのID
    int getAngle(uint8_t id);

    private:
    int angle_[8];
    int rpm_[8];
    int amp_[8];
    Type type_[8];
    int16_t output[8];

    /// @brief 各モーターによって異なる電流値の最大値の制約を反映する
    /// @param id モーターのID
    int constrainCurrent(uint8_t id)
    {
      int16_t max_min = 0;
      if(type_[id] == Type::M2006)
      {
        max_min = 10000;
      }
      else if(type_[id] == Type::M3508)
      {
        max_min = 16384;
      }

      if(output[id] > 0)
      {
        if(output[id] > max_min)
        {
          return max_min;
        }
        else
        {
            return output[id];
        }
      }
      else if(output[id] < 0)
      {
        if(abs(output[id]) > max_min)
        {
          return -1*max_min;
        }
        else
        {
          return output[id];
        }
      }
      else
      {
        return 0;
      }
    }
  };
}
#endif