●透過Mbed Studio編譯+執行
https://os.mbed.com/studio/

到底下網址下載sample code：
https://os.mbed.com/platforms/Renesas-GR-MANGO/

必須另外接一條USB-A到PC端，才能在IDE上看到print的東西
==============================================

●透過Mbed CLI編譯+執行

https://os.mbed.com/docs/mbed-os/v6.14/quick-start/build-with-mbed-cli.html
點選Next可以看編譯教學

用 conda prompt 建立 python 環境：
>> conda create --name xxx python=3.7
>> conda activate xxx
>> pip install mbed-cli

cd到，想要的資料夾路徑底下
>> mbed add mbed-os

-- 透過"mbed detect"指令找到連接裝置名稱
-- 編譯指令：mbed compile --target GR_MANGO --toolchain GCC_ARM --flash
-- toolchain 先用 GCC_ARM；ARM 好像是商業工具會需要 License。
-- 若是硬要做可以自行 download ARM compiler，並透過指令 mbed config -G ARM_PATH "C:\...." 加入變數。

必須另外接一條 USB-A 到 PC 端，才能在 terminal 上看到 print 的東西。(以下是指令)
>> mbed sterm -b 115200(後面數字要看mbed_app.json裡面如何設定)

=================================================

●修改 Mbed Studio 的 Compiler
到 C:\Users\[使用者名稱]\AppData\Local\Mbed Studio 底下新增 external-tools.json 檔案
詳細資訊請看 → https://os.mbed.com/docs/mbed-studio/current/installing/switching-to-gcc.html
