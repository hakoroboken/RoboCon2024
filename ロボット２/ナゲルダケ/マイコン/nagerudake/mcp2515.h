#ifndef MCP_2515_H_
#define MCP_2515_H_

#include <mcp_can.h>
#include <SPI.h>

extern MCP_CAN can0;

namespace hakorobo2024
{
  /// @brief MCP2515とSPI通信をしてCAN通信するクラス
  /// @note CSピンは6番を使う
  class MCP2515
  {
    public:
    /// @brief セットアップ関数
    /// @note setup内で実行する
    void initialize();
    
    /// @brief 送信する関数
    /// @param dest 送信相手のIDを指定する
    /// @param buf 8バイトの送信内容
    void send(unsigned long int dest, byte *buf);

    /// @brief 受信する関数
    /// @param id 送り元のIDを格納する
    /// @param len データの長さを格納する
    /// @param buf 受信内容をコピーする先
    void recv(unsigned long int *id, byte *len, char *buf);

    private:
    long Pre_millis;
  };
}

#endif