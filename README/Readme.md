# 说明
1. 程序启动时调用一次 HandshakeInit()。
2. 收到 FunctionCode == 7 时，调用 HandshakeData1(), head 指针作为 in_data。
3. 系统启动后，每秒调用一次 HandshakeData2() 获得数据并发送。
4. 获得的 out_data 使用 Navipack_TransportPacking() 函数进行打包，然后发送。

每个函数都是返回 true 为成功，返回 false 则无法使用。