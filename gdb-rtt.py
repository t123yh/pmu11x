import gdb
addr = int(gdb.parse_and_eval("_SEGGER_RTT").address)
gdb.execute("monitor rtt setup 0x{:x} 2048 \"SEGGER RTT\"".format(addr))
gdb.execute("monitor rtt start")
gdb.execute("monitor rtt server start 9090 0")
