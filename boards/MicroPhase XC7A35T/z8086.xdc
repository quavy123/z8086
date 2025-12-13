set_property PACKAGE_PIN J19 [get_ports clk50]
set_property IOSTANDARD LVCMOS33 [get_ports clk50]

create_clock -period 20.00 [get_ports clk50]

# 2 LEDs
set_property PACKAGE_PIN N18 [get_ports {led[7]}]
set_property PACKAGE_PIN M18 [get_ports {led[6]}]
# GPIO1 2P,2N,3P,3N,4P,4N
set_property PACKAGE_PIN D14 [get_ports {led[5]}]
set_property PACKAGE_PIN D15 [get_ports {led[4]}]
set_property PACKAGE_PIN E16 [get_ports {led[3]}]
set_property PACKAGE_PIN D16 [get_ports {led[2]}]
set_property PACKAGE_PIN D17 [get_ports {led[1]}]
set_property PACKAGE_PIN C17 [get_ports {led[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property PACKAGE_PIN AA1 [get_ports s0]
set_property PACKAGE_PIN W1 [get_ports s1]
set_property IOSTANDARD LVCMOS33 [get_ports s0]
set_property IOSTANDARD LVCMOS33 [get_ports s1]

# GPIO1 0P,0N,1P,1N
set_property PACKAGE_PIN F13 [get_ports TFT_SPI_CLK]
set_property PACKAGE_PIN F14 [get_ports TFT_RS]
set_property PACKAGE_PIN E13 [get_ports TFT_SPI_MOSI]
set_property PACKAGE_PIN E14 [get_ports TFT_SPI_CS]
set_property IOSTANDARD LVCMOS33 [get_ports TFT_SPI_CLK]
set_property IOSTANDARD LVCMOS33 [get_ports TFT_RS]
set_property IOSTANDARD LVCMOS33 [get_ports TFT_SPI_MOSI]
set_property IOSTANDARD LVCMOS33 [get_ports TFT_SPI_CS]
