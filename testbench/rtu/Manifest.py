target = "xilinx" #  "altera" # 
action = "simulation"

#fetchto = "../../ip_cores"

files = [
  "../../modules/wrsw_shared_types_pkg.vhd",
  "if_rtu_port.svh",
  "xwrsw_rtu_wrapper.svh",
  "xwrsw_rtu_wrapper.vhd",
  "main.sv",
  ]

vlog_opt="+incdir+./+incdir+../../ip_cores/wr-cores/sim +incdir+../../ip_cores/wr-cores/sim/fabric_emu +incdir+../../sim "

modules = {"local":
		[ 
		  "../../ip_cores/wr-cores",
		  "../../ip_cores/wr-cores/ip_cores/general-cores/modules/genrams/",
		  "../../modules/wrsw_rtu/",
		],

	  }
