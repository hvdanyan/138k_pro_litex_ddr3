#!/usr/bin/env python3

from math import log2
from liteeth.phy.gw5rgmii import LiteEthPHYRGMII
from litex.build.io import DDROutput
from litex.gen import *

from litex.soc.cores.clock.gowin_gw5a import GW5APLL
from litedram.phy import GW5DDRPHY

from litex.soc.cores.led import LedChaser
from litex.soc.cores.video import VideoGowinHDMIPHY
from litex.soc.integration.common import get_mem_data
from litex.soc.integration.soc import SoCRegion
from litex.soc.interconnect import wishbone
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore

from litex_boards.platforms import sipeed_tang_mega_138k_pro

from litedram.modules import MT41J256M16

from litex.soc.integration.builder import *

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, cpu_clk_freq, with_ddr3):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_cpu = ClockDomain()
        self.cd_por = ClockDomain() 

        if with_ddr3:
            self.cd_memory = ClockDomain()

        # Clk
        self.clk50 = clk50 = platform.request("clk50")
        rst = platform.request("rst")

        # Power on reset
        por_count = Signal(16, reset = 2 ** 16 - 1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(clk50) # assign clk50 to cd_por
        self.comb += por_done.eq(por_count == 0) 
        self.sync.por += If(~por_done, por_count.eq(por_count - 1)) 

        # PLL
        self.pll = pll = GW5APLL(devicename = platform.devicename, device=platform.device) 

        self.comb += pll.reset.eq(~por_done | rst)

        pll.register_clkin(clk50, 50e6)
        if not with_ddr3:
            pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=not with_ddr3)
        pll.create_clkout(self.cd_cpu, cpu_clk_freq, with_reset=False)

        platform.toolchain.additional_cst_commands.append("INS_LOC \"PLL\" PLL_R[0]") # Magic incantation for Gowin-AE350 CPU :)

        # HDMI
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()
        pll.create_clkout(self.cd_hdmi5x, 125e6, margin=1e-3)
        self.specials += Instance("CLKDIV",
            p_DIV_MODE = "5",
            i_HCLKIN   = self.cd_hdmi5x.clk,
            i_RESETN   = 1, # Disable reset signal.
            i_CALIB    = 0, # No calibration.
            o_CLKOUT   = self.cd_hdmi.clk
        )
        self.specials += AsyncResetSynchronizer(self.cd_sys, self.cd_hdmi5x.rst)

class BaseSoC(SoCCore):
    def __init__(self, platform, sys_clk_freq=100e6,
                with_ddr3 = True, 
                eth_dynamic_ip      = False,
                with_ethernet       = True,
                with_etherbone      = False,
                local_ip            = "192.168.1.50",
                remote_ip           = "",
                **kwargs):
        
        platform = platform.Platform(toolchain = "gowin")

        # CRG
        self.crg = _CRG(platform,sys_clk_freq,
                        cpu_clk_freq = sys_clk_freq,
                        with_ddr3 = with_ddr3
                        )
        
        # SoCCore
        # # Limit internal SRAM size.
        # kwargs["integrated_sram_size"] = 0x1000
        # # Can only support minimal variant of vexriscv
        # if kwargs.get("cpu_type", "vexriscv") == "vexriscv":
        #     kwargs["cpu_variant"] = "minimal"
        # SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on RZ-EasyFPGA", **kwargs)

        SoCCore.__init__(self, platform, sys_clk_freq,
                         ident = "Special TEST on LiteX SoC with Tang Mega 138K Pro", 
                         
                         **kwargs)

        
        # HDMI
        
        hdmi_pads = platform.request("hdmi_in") # yes DVI_RX because DVI_TX seems not working
        self.comb += hdmi_pads.hdp.eq(1)
        self.videophy = VideoGowinHDMIPHY(hdmi_pads, clock_domain="hdmi")
        #self.add_video_colorbars(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
        self.add_video_terminal(phy=self.videophy, timings="640x480@75Hz", clock_domain="hdmi")
        # self.add_video_framebuffer(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
        

        # DDR3 SDRAM 

        if with_ddr3 and not self.integrated_main_ram_size:
            self.ddr3_module = DDR3_wishbone_default(
                platform=platform , 
                crg = self.crg,
                bus = self.bus)
            

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"),
                tx_delay   = 2e-9,
                rx_delay   = 2e-9)
            clk50_half = Signal()
            self.specials += Instance("CLKDIV",
                p_DIV_MODE = "2",
                i_HCLKIN   = platform.lookup_request("clk50"),
                i_RESETN   = 1,
                i_CALIB    = 0,
                o_CLKOUT   = clk50_half)
            self.specials += DDROutput(1, 0, platform.request("ephy_clk"), clk50_half)

            if with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip, data_width=32, software_debug=False)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, data_width=32)

            if local_ip:
                local_ip = local_ip.split(".")
                self.add_constant("LOCALIP1", int(local_ip[0]))
                self.add_constant("LOCALIP2", int(local_ip[1]))
                self.add_constant("LOCALIP3", int(local_ip[2]))
                self.add_constant("LOCALIP4", int(local_ip[3]))

            if remote_ip:
                remote_ip = remote_ip.split(".")
                self.add_constant("REMOTEIP1", int(remote_ip[0]))
                self.add_constant("REMOTEIP2", int(remote_ip[1]))
                self.add_constant("REMOTEIP3", int(remote_ip[2]))
                self.add_constant("REMOTEIP4", int(remote_ip[3]))



        # LED chaser

        self.leds = LedChaser(
            pads         = platform.request_all("led_n"),
            sys_clk_freq = sys_clk_freq
        )


from litex.soc.interconnect.csr import *

class DDR_Interface(LiteXModule):
    def __init__(self):
        self.cmd_ready = Signal() #o
        self.cmd = Signal(3)
        self.cmd_en = Signal()

        self.addr = Signal(29)

        self.wr_data_rdy = Signal()
        self.wr_data = Signal(256)
        self.wr_data_en = Signal()
        self.wr_data_end = Signal()
        self.wr_data_mask = Signal(32)

        self.rd_data = Signal(256)
        self.rd_data_valid = Signal()
        self.rd_data_end = Signal()
        


class DDR3_wishbone_default(LiteXModule):

    def __init__(self, platform, crg, bus):
        self.crg = crg
        self.bus = bus


        self.pll_lock = Signal()
        self.clkout0 = Signal()
        self.clkout1 = Signal()

        self.pads = platform.request("ddram")
        self.pll_stop = Signal()
        
        
        self.rst_n = self.crg.rst
        self.ddr_rst = Signal()
        self.init_calib_complete = Signal() 

        self.ddr_ddr_interface = DDR_Interface()
        self.porta_ddr_interface = DDR_Interface()
        self.portb_ddr_interface = DDR_Interface()


        self.sr_req = Signal()
        self.ref_req = Signal()
        self.sr_ack = Signal()
        self.ref_ack = Signal()
        self.burst = Signal(3)

        #DMA Control
        self.dma_select = Signal()


        # add the black boxes
        self.specials += [
                Instance("Gowin_PLL",
                    o_lock = self.pll_lock,
                    o_clkout0 = self.clkout0,
                    o_clkout1 = self.clkout1,
                    o_clkout2 = self.crg.cd_memory.clk,
                    i_clkin = self.crg.clk50,
                    i_enclk0 = 1,
                    i_enclk1 = 1,
                    i_enclk2 = self.pll_stop
                            ),


                Instance("DDR3_Memory_Interface_Top",
                    # i_clk=ClockSignal("sys"),
                    i_clk=self.crg.clk50,
                    o_pll_stop=self.pll_stop,
                    i_memory_clk=self.crg.cd_memory.clk,
                    i_pll_lock=self.pll_lock,
                    i_rst_n=1, # Need to be changed
                    o_clk_out=self.crg.cd_sys.clk, # 从DDR3输出的时钟信号作为sys时钟
                    o_ddr_rst=self.ddr_rst,
                    o_init_calib_complete=self.init_calib_complete,
                    o_cmd_ready=self.ddr_ddr_interface.cmd_ready,
                    i_cmd=self.ddr_ddr_interface.cmd,
                    i_cmd_en=self.ddr_ddr_interface.cmd_en,
                    i_addr=self.ddr_ddr_interface.addr,
                    o_wr_data_rdy=self.ddr_ddr_interface.wr_data_rdy,
                    i_wr_data=self.ddr_ddr_interface.wr_data,
                    i_wr_data_en=self.ddr_ddr_interface.wr_data_en,
                    i_wr_data_end=self.ddr_ddr_interface.wr_data_end,
                    i_wr_data_mask=self.ddr_ddr_interface.wr_data_mask,
                    o_rd_data=self.ddr_ddr_interface.rd_data,
                    o_rd_data_valid=self.ddr_ddr_interface.rd_data_valid,
                    o_rd_data_end=self.ddr_ddr_interface.rd_data_end,
                    i_sr_req=0,
                    i_ref_req=0,
                    o_sr_ack=self.sr_ack,
                    o_ref_ack=self.ref_ack,
                    i_burst=0,
                    o_O_ddr_addr=self.pads.a,
                    o_O_ddr_ba=self.pads.ba,
                    o_O_ddr_cs_n=self.pads.cs_n,
                    o_O_ddr_ras_n=self.pads.ras_n,
                    o_O_ddr_cas_n=self.pads.cas_n,
                    o_O_ddr_we_n=self.pads.we_n,
                    o_O_ddr_clk=self.pads.clk_p,
                    o_O_ddr_clk_n=self.pads.clk_n,
                    o_O_ddr_cke=self.pads.cke,
                    o_O_ddr_odt=self.pads.odt,
                    o_O_ddr_reset_n=self.pads.reset_n,
                    o_O_ddr_dqm=self.pads.dm,
                    io_IO_ddr_dq=self.pads.dq,
                    io_IO_ddr_dqs=self.pads.dqs_p,
                    io_IO_ddr_dqs_n=self.pads.dqs_n,
                )
            ]
        

        # Create Wishbone Slave.
        cache_l2 = wishbone.Interface(data_width=self.bus.data_width, address_width=32, addressing="word")
        self.bus.add_slave(name="main_ram", slave=cache_l2, region=SoCRegion(origin=0x40000000, size=0x40000000, mode="rwx"))


        # Create L2 Cache.

        l2_cache_size = 8192
        l2_cache_reverse = False
        l2_cache_full_memory_we = True
        l2_cache_size = l2_cache_size # Use minimal size if lower
        
        l2_cache_size = 2**int(log2(l2_cache_size))                  # Round to nearest power of 2
        l2_cache = wishbone.Cache(
            cachesize = l2_cache_size//4,
            master    = cache_l2,
            slave     = wishbone.Interface(data_width=256, address_width=32, addressing="word"),
            reverse   = l2_cache_reverse)
        if l2_cache_full_memory_we:
                l2_cache = FullMemoryWE()(l2_cache)
        self.l2_cache = l2_cache
        wb_sdram = self.l2_cache.slave
        #litedram_wb = self.l2_cache.slave
        # self.add_config("L2_SIZE", l2_cache_size)
        # input(self.bus.data_width)

        


        self.relative_addr = Signal(32)
        self.word_index = Signal(3)

        self.comb +=[
            self.relative_addr.eq(wb_sdram.adr << 3),#计算相对地址
            # self.word_index.eq((self.relative_addr >> 2) & 0x7)# 计算除以4后取最低3位
            # self.word_index.eq((self.relative_addr) & 0x7)
        ]

        self.fsm = fsm = FSM(reset_state="CMD")
        self.comb += [
            # self.addr.eq((self.relative_addr >> log2_int(256//8)) << 3),
            self.ddr_ddr_interface.addr.eq(self.relative_addr >> 3 << 3), # 不删除低位好像有问题
        ]
        
        fsm.act("CMD",
            If(wb_sdram.cyc & wb_sdram.stb & self.ddr_ddr_interface.wr_data_rdy &  wb_sdram.we & self.ddr_ddr_interface.cmd_ready,
                self.ddr_ddr_interface.cmd.eq(0b000),
                self.ddr_ddr_interface.cmd_en.eq(1),
                self.ddr_ddr_interface.wr_data_en.eq(1),
                self.ddr_ddr_interface.wr_data.eq(wb_sdram.dat_w),  # 左移操作
                self.ddr_ddr_interface.wr_data_mask.eq(0x0),  # 位移掩码
                # self.ddr_ddr_interface.wr_data_mask.eq(Mux(self.word_index == 0, 0xFFFFFFF0,
                #     Mux(self.word_index == 1, 0xFFFFFF0F,
                #     Mux(self.word_index == 2, 0xFFFFF0FF,
                #     Mux(self.word_index == 3, 0xFFFF0FFF,
                #     Mux(self.word_index == 4, 0xFFF0FFFF,
                #     Mux(self.word_index == 5, 0xFF0FFFFF,
                #     Mux(self.word_index == 6, 0xF0FFFFFF,
                #                                 0x0FFFFFFF)))))))),
                NextState("WRITE")
                ),
            If(wb_sdram.cyc & wb_sdram.stb & ~wb_sdram.we & self.ddr_ddr_interface.cmd_ready,
                self.ddr_ddr_interface.cmd.eq(0b001),
                self.ddr_ddr_interface.cmd_en.eq(1),
                NextState("READ")
                ),
        )

        self.comb += [
            self.ddr_ddr_interface.wr_data_end.eq(self.ddr_ddr_interface.wr_data_en),
        ]
        fsm.act("WRITE",
            self.ddr_ddr_interface.cmd_en.eq(0),
            self.ddr_ddr_interface.wr_data_en.eq(0),
                wb_sdram.ack.eq(1),
                NextState("CMD")
        )
        fsm.act("READ",
            self.ddr_ddr_interface.cmd_en.eq(0),
            If(self.ddr_ddr_interface.rd_data_valid,
                wb_sdram.ack.eq(1),
                wb_sdram.dat_r.eq(self.ddr_ddr_interface.rd_data),
                # wb_sdram.dat_r.eq(Mux(self.word_index == 0, self.ddr_ddr_interface.rd_data[0:32],
                #     Mux(self.word_index == 1, self.ddr_ddr_interface.rd_data[32:64],
                #     Mux(self.word_index == 2, self.ddr_ddr_interface.rd_data[64:96],
                #     Mux(self.word_index == 3, self.ddr_ddr_interface.rd_data[96:128],
                #     Mux(self.word_index == 4, self.ddr_ddr_interface.rd_data[128:160],
                #     Mux(self.word_index == 5, self.ddr_ddr_interface.rd_data[160:192],
                #     Mux(self.word_index == 6, self.ddr_ddr_interface.rd_data[192:224],
                #                         self.ddr_ddr_interface.rd_data[224:256])))))))), 
                    NextState("CMD")
            )
        )
        fsm.act("DONE",
            wb_sdram.ack.eq(0),
            self.ddr_ddr_interface.cmd_en.eq(0),
            NextState("CMD")
            )

# Build

def main():
    from litex.build.parser import LiteXArgumentParser

    platform = sipeed_tang_mega_138k_pro

    parser = LiteXArgumentParser(platform = platform.Platform)

    parser.add_target_argument("--sys-clk-freq",    default=100e6, type=float, help="System clock frequency.")

    parser.add_target_argument("--with-ddr3", action="store_true", default=True,    help="Enable optional DDR3 module.")

    ethopts = parser.target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",         default=False, action="store_true",      help="Enable Ethernet support.")
    ethopts.add_argument("--with-etherbone",        action="store_true",      help="Enable Etherbone support.")
    parser.add_target_argument("--remote-ip",       default="192.168.10.1",  help="Remote IP address of TFTP server.")
    parser.add_target_argument("--local-ip",        default="192.168.10.2",   help="Local IP address.")

    args = parser.parse_args()

    soc = BaseSoC(
        platform            = platform,
        sys_clk_freq        = args.sys_clk_freq,
        with_ddr3           = args.with_ddr3,
        with_ethernet       = args.with_ethernet,
        with_etherbone      = args.with_etherbone,
        local_ip            = args.local_ip,
        remote_ip           = args.remote_ip,
        **parser.soc_argdict
    )


    # Let the compiler won't start
    args.no_compile_gateware = True

    # args.no_compile = True
    # input("Press enter to continue.")

    builder = Builder(soc, **parser.builder_argdict)

    builder.build( **parser.toolchain_argdict)

if __name__ == "__main__":
    main()



