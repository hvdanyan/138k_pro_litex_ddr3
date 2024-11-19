# litex on Tang_Mega_138k_pro with ddr3 supported.

This module uses official IP cores instead of "LiteDRAM" to solve the problem that "Tang Mega 138k Pro" cannot initiate DDR3.

The file "withDDR_sipeed_tang_mega_138k_pro.py" is modified from [https://github.com/litex-hub/litex-boards/blob/master/litex_boards/targets/sipeed_tang_mega_138k_pro.py](https://github.com/litex-hub/litex-boards/blob/master/litex_boards/targets/sipeed_tang_mega_138k_pro.py).

After generating the target verilog file, also need to download generated ip cores "Gowin_PLL.v" and "DDR3_Memory_Interface_Top.v" from:[https://github.com/sipeed/TangMega-138KPro-example/tree/main/pro_ddr_test/src](https://github.com/sipeed/TangMega-138KPro-example/tree/main/pro_ddr_test/src)
