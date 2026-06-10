import sys
sys.path.append('/home/pi/FASTCASST/libraries/Ublox')
sys.path.append('/home/pi/FASTCASST/libraries/Util')
import util
import ublox


if __name__ == "__main__":

    ubl = ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

    ubl.configure_poll_port()
    ubl.configure_poll(ublox.CLASS_CFG, ublox.MSG_CFG_USB)
    #ubl.configure_poll(ublox.CLASS_MON, ublox.MSG_MON_HW)

    ubl.configure_port(port=ublox.PORT_SERIAL1, inMask=1, outMask=0)
    ubl.configure_port(port=ublox.PORT_USB, inMask=1, outMask=1)
    ubl.configure_port(port=ublox.PORT_SERIAL2, inMask=1, outMask=0)
    ubl.configure_poll_port()
    ubl.configure_poll_port(ublox.PORT_SERIAL1)
    ubl.configure_poll_port(ublox.PORT_SERIAL2)
    ubl.configure_poll_port(ublox.PORT_USB)
    ubl.configure_solution_rate(rate_ms=1000)

    ubl.set_preferred_dynamic_model(None)
    ubl.set_preferred_usePPP(None)

    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSLLH, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_PVT, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_STATUS, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SOL, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELNED, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_SVINFO, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_VELECEF, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_POSECEF, 1)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_RAW, 1)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SFRB, 1)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_SVSI, 1)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_ALM, 1)
    ubl.configure_message_rate(ublox.CLASS_RXM, ublox.MSG_RXM_EPH, 1)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_TIMEGPS, 5)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_CLOCK, 5)
    ubl.configure_message_rate(ublox.CLASS_NAV, ublox.MSG_NAV_DGPS, 5)

    while True:
        msg = ubl.receive_message()
        if msg is None:
            if opts.reopen:
                ubl.close()
                ubl = ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                continue
            print(empty)
            break
        #print(msg.name())
        if msg.name() == "NAV_POSLLH":
            print(msg.name())
            print(str(msg))
            outstr = str(msg).split(",")[1:]
            outstr = "".join(outstr)
            print(outstr)
        if msg.name() == "NAV_STATUS":
            print(msg.name())
            print(str(msg))
            outstr = str(msg).split(",")[1:2]
            outstr = "".join(outstr)
            print(outstr)
        #print(str(msg))
