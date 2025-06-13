# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\work\CPUDesignandVerification\250522_SPI_M_S\vitis\design_1_wrapper\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\work\CPUDesignandVerification\250522_SPI_M_S\vitis\design_1_wrapper\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {design_1_wrapper}\
-hw {C:\work\CPUDesignandVerification\250522_SPI_M_S\vitis\design_1_wrapper.xsa}\
-fsbl-target {psu_cortexa53_0} -out {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis}

platform write
domain create -name {standalone_microblaze_0} -display-name {standalone_microblaze_0} -os {standalone} -proc {microblaze_0} -runtime {cpp} -arch {32-bit} -support-app {hello_world}
platform generate -domains 
platform active {design_1_wrapper}
platform generate -quick
platform generate
platform active {design_1_wrapper}
platform config -updatehw {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_2/design_2_wrapper.xsa}
platform generate -domains 
platform config -updatehw {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_1_wrapper.xsa}
platform generate -domains 
platform config -updatehw {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_1_wrapper.xsa}
platform generate -domains 
platform config -updatehw {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_1_wrapper.xsa}
platform generate -domains 
platform config -updatehw {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_1_wrapper.xsa}
platform generate -domains 
platform active {design_1_wrapper}
platform config -updatehw {C:/work/CPUDesignandVerification/250522_SPI_M_S/vitis/design_1_wrapper.xsa}
platform generate -domains 
platform generate -domains 
