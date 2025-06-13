# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: C:\work\CPUDesignandVerification\250522_SPI_M_S\vitis\axi_spi_ms_system\_ide\scripts\systemdebugger_axi_spi_ms_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source C:\work\CPUDesignandVerification\250522_SPI_M_S\vitis\axi_spi_ms_system\_ide\scripts\systemdebugger_axi_spi_ms_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -filter {jtag_cable_name =~ "Digilent Basys3 210183BB7A2CA" && level==0 && jtag_device_ctx=="jsn-Basys3-210183BB7A2CA-0362d093-0"}
fpga -file C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/axi_spi_ms/_ide/bitstream/design_1_wrapper.bit
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2"  && jtag_cable_name =~ "Digilent Basys3 210183BB7A2CA" && jtag_device_ctx=="jsn-Basys3-210183BB7A2CA-0362d093-0"}
loadhw -hw C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_1_wrapper/export/design_1_wrapper/hw/design_1_wrapper.xsa -regs
configparams mdm-detect-bscan-mask 2
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2"  && jtag_cable_name =~ "Digilent Basys3 210183BB7A2CA" && jtag_device_ctx=="jsn-Basys3-210183BB7A2CA-0362d093-0"}
rst -system
after 3000
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2"  && jtag_cable_name =~ "Digilent Basys3 210183BB7A2CA" && jtag_device_ctx=="jsn-Basys3-210183BB7A2CA-0362d093-0"}
dow C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/axi_spi_ms/Debug/axi_spi_ms.elf
targets -set -nocase -filter {name =~ "*microblaze*#0" && bscan=="USER2"  && jtag_cable_name =~ "Digilent Basys3 210183BB7A2CA" && jtag_device_ctx=="jsn-Basys3-210183BB7A2CA-0362d093-0"}
con
